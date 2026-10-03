/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mmio.h"
#include "delay.h"
#include "arch_helpers.h" /* flush_dcache_range / inv_dcache_range 的声明 */

/* cvitek includes. */
#include "printf.h"
#include "rtos_cmdqu.h"
#include "fast_image.h"
#include "cvi_mailbox.h"
#include "intr_conf.h"
#include "top_reg.h"
#include "memmap.h"

#include "comm.h"
#include "cvi_spinlock.h"

/****************************************************************************
 * 跨核共享缓冲（命令队列之外的批量数据通道）
 *
 * 布局与大核侧 drivers/ipc/sg2002-cmdqu/src/shm.rs 逐字段对应：
 *   0x0000 头部（192 字节 = 3 条 cache line，按"谁写"分行）
 *   0x00C0 同步块（128 字节 = 2 条 cache line：大核一行、小核一行）
 *   0x0140 Ring A（大核 → 小核）
 *   0x1140 Ring B（小核 → 大核）  ← 上报任务写这个环
 *
 * 小核 D-cache 打开、大核按非缓存访问，因此写环条目必须先写载荷并 clean、
 * 再更新 head 并 clean，顺序颠倒会让大核读到指向旧载荷的 head。
 *
 * **为什么头部要占三条 cache line**：回写粒度是整条 line（`dcache.cpa`），
 * 只 clean 自己那几个字节也会把整行写回。早期版本头部只有一行，小核发布 head
 * 时会把同一行里大核的 b_tail 一起覆盖回旧值，大核按 tail 记账因此忽多忽少。
 * 现在按写入方分行：行 0 常量（大核初始化写一次）、行 1 大核写、行 2 小核写。
 ****************************************************************************/
#define CMDQU_SHM_BASE           0x8B200000UL
#define CMDQU_SHM_MAGIC          0x43515348UL
#define CMDQU_SHM_VERSION        3U   /* 必须与大核侧 shm.rs 的 SHM_VERSION 一致 */
#define CMDQU_RING_B_OFF         0x1140U  /* 与 shm.rs 的 RING_RTOS_TO_HOST_OFFSET 一致 */
/* 小核独占写的那条 cache line（头部行 2）在共享区内的偏移 */
#define CMDQU_SHM_RTOS_LINE_OFF  0x0080U
#define CMDQU_RING_ENTRY_SZ      64U

/* 大核用来设置上报周期的命令号 */
#define CMDQU_CMD_SET_REPORT     0x60U
/* 大核用来开关"小核侧 UART1 链路测试"的命令号（0=关、1=握手模式） */
#define CMDQU_CMD_UART1_MODE     0x61U
/* M3 第 3 步：周期控制。0x62 设目标速度（并以心跳维持），0x63 设控制周期 */
#define CMDQU_CMD_SET_TARGET      0x62U
#define CMDQU_CMD_SET_CTRL_PERIOD 0x63U
/* 0x64：设置心跳看门狗窗口（毫秒）。默认 500 ms；调大用于"看得见"的演示。 */
#define CMDQU_CMD_SET_WD         0x64U
/* 最小上报周期：本固件 tick 为 200 Hz（5 ms），更小的值会被 pdMS_TO_TICKS 算成 0 */
#define CMDQU_MIN_PERIOD_MS      5U
/* 环消息类型：状态快照 */
#define CMDQU_MSG_STATE_SNAPSHOT 1U
/* 环消息类型：UART1（底盘链路）握手状态，见 prvCmdquUart1Task 的说明 */
#define CMDQU_MSG_UART_LINK      6U
/* 环消息类型：UART1 命令自检（mode>=2 时每秒上报） */
#define CMDQU_MSG_UART_PROBE     7U
/* 环消息类型：UART1 周期控制状态（mode>=3 时每秒上报） */
#define CMDQU_MSG_UART_CTRL      8U
/* 环消息类型：UART1 控制周期分布（mode>=3 时每秒上报，累计值） */
#define CMDQU_MSG_UART_CTRL_HIST 9U

/****************************************************************************
 * 小核侧 UART1：直接与底盘控制板对话（焊盘 A18/A19）
 *
 * 为什么第一步不是"原样回显"：A18/A19 上 TX 与 RX 是两根彼此独立的线，对端是
 * 底盘控制板；小核发出去的字节只会到达底盘板，不可能回到另一个核的 RX。所以能
 * 证明"小核掌握了这条链路"的最小实验是**直接与底盘板握手**：发 CMD_INIT
 * （AA 55 01 00 01），看它是否回 ACK（AA 55 80 01 01 80）。宿主侧通过共享环看
 * 计数增长，拔掉 A18/A19 排线计数就停。
 *
 * **默认关闭**（g_cmdqu_uart1_mode = 0）：不碰 UART1 寄存器、不动 FMUX。厂商
 * Linux 上 AKA-00 应用正在使用 /dev/ttyS1，若小核同时往同一条链路发帧会互相
 * 干扰，所以必须由大核显式用 CMDQU_CMD_UART1_MODE(0x61) 打开。
 ****************************************************************************/
#define CMDQU_UART1_BASE         0x04150000UL
/* 底盘协议命令号（与 AKA-00 的 motor.rs / test_uart.py 一致） */
#define CMDQU_MOTOR_CMD_INIT     0x01U
#define CMDQU_MOTOR_CMD_CONFIG   0x02U
#define CMDQU_MOTOR_CMD_SET_SPEED 0x10U
#define CMDQU_MOTOR_CMD_STOP     0x11U
#define CMDQU_MOTOR_CMD_GET_STATUS 0x21U
#define CMDQU_MOTOR_RSP_ACK      0x80U
#define CMDQU_MOTOR_RSP_NACK     0x81U
#define CMDQU_MOTOR_RSP_STATUS   0x91U
/* 自检步骤数：CONFIG / SET_SPEED(0,0) / SET_SPEED(1,0) / STOP(0) / STOP(1) */
#define CMDQU_UART1_PROBE_STEPS  5U
#define CMDQU_UART1_PROBE_GAP_MS 300U
	/* 周期控制的节拍与安全阈值 */
#define CMDQU_UART1_TICK_MS      5U   /* 任务基础节拍 = 1 个 tick（200 Hz） */
#define CMDQU_UART1_WD_MS        500U /* 大核心跳超时：超过它就自动停车 */
#define CMDQU_UART1_CTRL_DEF_MS  100U /* 默认控制周期 */
#define CMDQU_UART1_STATUS_EVERY 5U   /* 每 N 个控制周期读一次状态/转速 */
#define CMDQU_UART1_PWM_LIMIT    150  /* 目标 PWM 限幅（与 AKA-00 的 speed_scale 一致） */

/* UART1 周期控制状态，作为 kind=8 的环消息载荷（mode>=3 时每秒一条）。
 * 48 字节正好占满环消息载荷。 */
struct cmdqu_uart_ctrl {
	unsigned int seq;
	unsigned int mode;
	unsigned int period_ms;     /* 当前控制周期 */
	unsigned int target_lr;     /* 目标：低 16 = 左轮 PWM(i16)，高 16 = 右轮 PWM(i16) */
	unsigned int ctrl_count;    /* 已完成的控制周期数 */
	unsigned int ack_total;     /* 该链路收到的 ACK 总数 */
	unsigned int wd_stops;      /* 心跳超时自动停车的次数 */
	unsigned int dt_max_ticks;  /* 本秒内控制周期实测最大值（tick，正常恒等于周期） */
	unsigned int ch_state;      /* 底盘板回包里的状态机：0 未初始化/1 空闲/2 就绪/3 运行/4 错误 */
	int rpm_l;                  /* 底盘板回包的左轮转速 */
	int rpm_r;                  /* 底盘板回包的右轮转速 */
	unsigned int wd_ms;         /* 当前心跳看门狗窗口（毫秒），由 0x64 命令设置 */
};

/* UART1 控制周期分布，作为 kind=9 的环消息载荷（mode>=3 时每秒上报累计值）。
 * 控制周期按 5 ms 基础节拍量化，所以"抖动"体现在 dt 与期望 tick 数之差上：
 * 正常应当全部落在 `hist_eq`。 */
struct cmdqu_uart_ctrl_hist {
	unsigned int seq;
	unsigned int period_ticks; /* 期望周期（tick） */
	unsigned int count;        /* 累计统计的周期数 */
	unsigned int min_ticks;
	unsigned int max_ticks;
	unsigned int sum_ticks;    /* 用于算均值 */
	unsigned int hist_lt;      /* dt < 期望（不该出现） */
	unsigned int hist_eq;      /* dt == 期望（正常） */
	unsigned int hist_p1;      /* 期望 +1 tick */
	unsigned int hist_p2;      /* 期望 +2 tick */
	unsigned int hist_ge;      /* ≥ 期望 +3 tick（严重超时） */
	unsigned int wd_stops;
};
#define CMDQU_UART1_RBR_THR      0x00U
#define CMDQU_UART1_IER_DLM      0x04U
#define CMDQU_UART1_FCR_IIR      0x08U
#define CMDQU_UART1_LCR          0x0CU
#define CMDQU_UART1_LSR          0x14U
#define CMDQU_LSR_DATA_READY     0x01U
#define CMDQU_LSR_THR_EMPTY      0x20U
#define CMDQU_FMUX_PBASE         0x03001000UL
#define CMDQU_FMUX_UART1_TX_OFF  0x64U /* JTAG_CPU_TMS 焊盘 → UART1_TX */
#define CMDQU_FMUX_UART1_RX_OFF  0x68U /* JTAG_CPU_TCK 焊盘 → UART1_RX */
#define CMDQU_FMUX_UART1_FSEL    6U

/* UART1 链路握手的统计，作为 kind=6 的环消息载荷上报给大核 */
struct cmdqu_uart_link {
	unsigned int seq;
	unsigned int mode;              /* 0=关闭 1=握手模式 */
	unsigned int init_sent;         /* 已发出多少帧 CMD_INIT */
	unsigned int ack_rx;            /* 收到多少帧 ACK(0x80) */
	unsigned int nack_rx;           /* 收到多少帧 NACK(0x81) */
	unsigned int rx_bytes;          /* 该链路累计收到的字节数 */
	unsigned int frame_err;         /* 校验失败 / 认不出的帧数 */
	unsigned int rsp_other;         /* 合法但不是 ACK/NACK 的回包（状态/转速查询结果） */
	unsigned long long last_ack_us; /* 最近一次 ACK 的小核时刻（微秒） */
};

/* UART1 命令自检状态，作为 kind=7 的环消息载荷（mode>=2 时每秒一条）。
 * 与 kind=6 分开是为了不动它的既有布局：kind=6 只有握手统计，这里是命令级细节。 */
struct cmdqu_uart_probe {
	unsigned int seq;
	unsigned int mode;
	unsigned int stage;       /* 已发出的自检步数：0..CMDQU_UART1_PROBE_STEPS */
	unsigned int err;         /* 超时、回显命令号失配等异常次数 */
	unsigned int ack_total;   /* ACK 总数（含 INIT 与自检各命令；分类计数见下面三项） */
	unsigned int cfg_ack;     /* CMD_CONFIG 的 ACK 数 */
	unsigned int speed_ack;   /* CMD_SET_SPEED 的 ACK 数 */
	unsigned int stop_ack;    /* CMD_STOP 的 ACK 数 */
	unsigned int last_sent;   /* 最后发出的命令号 */
	unsigned int last_reply;  /* 最后收到的合法帧命令号（0x80/0x81） */
	unsigned int last_echo;   /* 该帧回显的命令号 */
	unsigned int reserved;
};

/* UART1 链路测试状态（由 mailbox 命令 0x61 开关） */
static volatile unsigned int g_cmdqu_uart1_mode = 0U;
static volatile unsigned int g_cmdqu_uart1_sent = 0U;
static volatile unsigned int g_cmdqu_uart1_ack = 0U;
static volatile unsigned int g_cmdqu_uart1_nack = 0U;
static volatile unsigned int g_cmdqu_uart1_rx_bytes = 0U;
static volatile unsigned int g_cmdqu_uart1_errs = 0U;
static volatile unsigned int g_cmdqu_uart1_rsp_other = 0U; /* 合法但不是 ACK/NACK 的回包 */
static volatile unsigned long long g_cmdqu_uart1_last_ack_us = 0ULL;
static volatile unsigned int g_cmdqu_uart1_link_seq = 0U;
/* 命令自检（mode>=2）：状态机、每类命令的 ACK 计数、最近一次收发命令号 */
static volatile unsigned int g_cmdqu_uart1_probe_seq = 0U;
static volatile unsigned int g_cmdqu_uart1_probe_stage = 0U; /* 已发出的自检步数 */
static volatile unsigned int g_cmdqu_uart1_probe_err = 0U;   /* 超时/失配次数 */
static volatile unsigned int g_cmdqu_uart1_cfg_ack = 0U;
static volatile unsigned int g_cmdqu_uart1_speed_ack = 0U;
static volatile unsigned int g_cmdqu_uart1_stop_ack = 0U;
static volatile unsigned int g_cmdqu_uart1_last_sent = 0U;
static volatile unsigned int g_cmdqu_uart1_last_reply = 0U; /* 合法帧的命令号 */
static volatile unsigned int g_cmdqu_uart1_last_echo = 0U;  /* ACK/NACK 回显的命令号 */
static volatile unsigned int g_cmdqu_uart1_replies = 0U;    /* 收到的 ACK+NACK 总数 */
/* 周期控制（mode>=3）：目标、心跳、控制周期与实测节拍 */
static volatile unsigned int g_cmdqu_uart1_ctrl_seq = 0U;
static volatile unsigned int g_cmdqu_uart1_ctrl_period_ms = CMDQU_UART1_CTRL_DEF_MS;
static volatile unsigned int g_cmdqu_uart1_target_lr = 0U; /* 低 16 = 左 i16，高 16 = 右 i16 */
static volatile unsigned int g_cmdqu_uart1_target_set = 0U;
static volatile unsigned int g_cmdqu_uart1_ctrl_count = 0U;
static volatile unsigned int g_cmdqu_uart1_wd_stops = 0U;
/* 心跳看门狗的共享状态：由 0x62 命令刷新截止时刻并清除"已停车"标志，
 * 由 UART1 任务在控制周期里检查。**不能放在任务局部**——之前就是因为
 * 截止时刻是任务局部且从未赋值（恒为 0），看门狗在第一个周期就误触发，
 * 小核此后一个 SET_SPEED 都不发（实板现象：底盘"运行中"但 RPM=0）。 */
static volatile unsigned long long g_cmdqu_uart1_wd_deadline_ms = 0ULL;
static volatile unsigned int g_cmdqu_uart1_wd_tripped = 0U;
static volatile unsigned int g_cmdqu_uart1_wd_ms = CMDQU_UART1_WD_MS;
static volatile unsigned int g_cmdqu_uart1_dt_max = 0U;    /* 本秒内控制周期最大值（tick） */
/* 控制周期分布（累计，直到关闭链路）：用来给出抖动分布而不是只看某一秒的最大值 */
static volatile unsigned int g_cmdqu_uart1_hist_seq = 0U;
static volatile unsigned int g_cmdqu_uart1_dt_min = 0xFFFFFFFFU;
static volatile unsigned int g_cmdqu_uart1_dt_sum = 0U;
static volatile unsigned int g_cmdqu_uart1_dt_count = 0U;
static volatile unsigned int g_cmdqu_uart1_hist_lt = 0U;
static volatile unsigned int g_cmdqu_uart1_hist_eq = 0U;
static volatile unsigned int g_cmdqu_uart1_hist_p1 = 0U;
static volatile unsigned int g_cmdqu_uart1_hist_p2 = 0U;
static volatile unsigned int g_cmdqu_uart1_hist_ge = 0U;
static volatile unsigned int g_cmdqu_uart1_ch_state = 0xFFU; /* 底盘回包里的状态机 */
static volatile int g_cmdqu_uart1_rpm_l = 0;
static volatile int g_cmdqu_uart1_rpm_r = 0;
/* 最近一条非 ACK 帧的载荷（GET_STATUS 的 5 字节用得上） */
static volatile unsigned int g_cmdqu_uart1_reply_cmd = 0U;
static volatile unsigned int g_cmdqu_uart1_reply_len = 0U;
static volatile unsigned char g_cmdqu_uart1_reply_payload[8];

/* 上报周期（毫秒）与自增序号，仅本文件使用 */
static volatile unsigned int g_cmdqu_report_period_ms = 1000U;

//#define __DEBUG__

#ifdef __DEBUG__
#define debug_printf printf
#else
#define debug_printf(...)
#endif

extern struct transfer_config_t transfer_config;
struct trace_snapshot_t snapshot;

typedef struct _TASK_CTX_S {
	char        name[32];
	u16         stack_size;
	UBaseType_t priority;
	void (*runTask)(void *pvParameters);
	u8            queLength;
	QueueHandle_t queHandle;
} TASK_CTX_S;

/****************************************************************************
 * Function prototypes
 ****************************************************************************/
void prvQueueISR(void);
void prvCmdQuRunTask(void *pvParameters);
void prvCmdquReportTask(void *pvParameters);
void prvCmdquUart1Task(void *pvParameters);

/****************************************************************************
 * Global parameters
 ****************************************************************************/
TASK_CTX_S gTaskCtx[E_QUEUE_MAX] = {
	{
		.name = "ISP",
		.stack_size = configMINIMAL_STACK_SIZE * 8,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = NULL,
		.queLength = 1,
		.queHandle = NULL,
	},
	{
		.name = "VCODEC",
		.stack_size = configMINIMAL_STACK_SIZE,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = NULL,
		.queLength = 1,
		.queHandle = NULL,
	},
	{
		.name = "VI",
		.stack_size = configMINIMAL_STACK_SIZE,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = NULL,
		.queLength = 1,
		.queHandle = NULL,
	},
	{
		.name = "CAMERA",
		.stack_size = configMINIMAL_STACK_SIZE,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = NULL,
		.queLength = 1,
		.queHandle = NULL,
	},
	{
		.name = "RGN",
		.stack_size = configMINIMAL_STACK_SIZE,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = prvRGNRunTask,
		.queLength = 10,
		.queHandle = NULL,
	},
	{
		.name = "CMDQU",
		.stack_size = configMINIMAL_STACK_SIZE,
		.priority = tskIDLE_PRIORITY + 5,
		.runTask = prvCmdQuRunTask,
		.queLength = 30,
		.queHandle = NULL,
	},
	{
		.name = "AUDIO",
		.stack_size = configMINIMAL_STACK_SIZE*15,
		.priority = tskIDLE_PRIORITY + 3,
		.runTask = prvAudioRunTask,
		.queLength = 10,
		.queHandle = NULL,
	},
};

volatile struct mailbox_set_register *mbox_reg;
volatile struct mailbox_done_register *mbox_done_reg;
volatile unsigned long *mailbox_context; // mailbox buffer context is 64 Bytess

/****************************************************************************
 * Function definitions
 ****************************************************************************/
QueueHandle_t main_GetMODHandle(QUEUE_HANDLE_E handle_idx)
{
	if (handle_idx >= E_QUEUE_MAX)
		return NULL;

	return gTaskCtx[handle_idx].queHandle;
}

void main_create_tasks(void)
{
	u8 i = 0;

#define TASK_INIT(_idx) \
do { \
	gTaskCtx[_idx].queHandle = xQueueCreate(gTaskCtx[_idx].queLength, sizeof(cmdqu_t)); \
	if (gTaskCtx[_idx].queHandle != NULL && gTaskCtx[_idx].runTask != NULL) { \
		xTaskCreate(gTaskCtx[_idx].runTask, gTaskCtx[_idx].name, gTaskCtx[_idx].stack_size, \
			    NULL, gTaskCtx[_idx].priority, NULL); \
	} \
} while(0)

	for (; i < ARRAY_SIZE(gTaskCtx); i++) {
		TASK_INIT(i);
	}
}

DEFINE_CVI_SPINLOCK(mailbox_lock, SPIN_MBOX);

/* 开机时清掉邮箱里"上一轮残留"的状态。
 *
 * **为什么必须做**：邮箱寄存器块在 AON 域，跨复位保留。上一轮若有一条命令写了槽位、
 * 拉了门铃却没人取走（最典型的就是大核侧超时退出，例如 `uart 0` 超时），这个待处理位
 * 会留到下一次开机；等小核在 `request_irq()` 内部 `plic_unmask_irq()` 的瞬间，PLIC
 * 立刻把中断打上来，而此时队列还没建好，`prvQueueISR` 会拿着空的队列句柄进
 * `xQueueSendFromISR` 而卡死。
 *
 * **实板现象（2026-10-03）**：小核 PC 停在 `request_irq` 收尾 / `do_irq` 入口之间来回
 * 跳（不是正常到处跑），Ring B 一条快照都不发，槽位里躺着上一轮的命令且 `rtos_valid=0`，
 * 大核这边所有命令都超时——正是这条路径。
 */
static void mailbox_clear_stale_state(void)
{
	struct mailbox_set_register *reg =
		(struct mailbox_set_register *)MAILBOX_REG_BASE;
	volatile unsigned long *ctx = (volatile unsigned long *)MAILBOX_REG_BUFF;
	int i;

	/* 清掉本核的待处理位与使能位，再把 8 个槽位清零 */
	reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_clr.mbox_int_clr = 0xFF;
	reg->cpu_mbox_en[RECEIVE_CPU].mbox_info = 0;
	for (i = 0; i < MAILBOX_MAX_NUM; i++) {
		ctx[i * 2] = 0;
		ctx[i * 2 + 1] = 0;
	}
}

void main_cvirtos(void)
{
	printf("create cvi task\n");

	/* 顺序很重要：队列必须先就绪，再使能中断——邮箱里若有残留待处理位，
	 * request_irq() 里的 plic_unmask 会立刻触发 ISR，那时 ISR 要用队列句柄。 */
	main_create_tasks();
	mailbox_clear_stale_state();

	request_irq(MBOX_INT_C906_2ND, prvQueueISR, 0, "mailbox", (void *)0);

#ifdef FAST_IMAGE_ENABLE
	start_camera(0);
#endif

	/* 跨核状态上报任务：按大核设定的周期往共享环写状态快照。
	 * 优先级取 tskIDLE_PRIORITY+4：高于 RGN/AUDIO（+3，都是周期/事件型任务），
	 * 低于 CMDQU（+5，邮箱命令应当先被处理）。实测在 +2 时会被它们推迟若干个
	 * tick，而确定性采样正是这条链路的价值所在。 */
	xTaskCreate(prvCmdquReportTask, "CMDQU-RPT", configMINIMAL_STACK_SIZE * 4, NULL,
		    tskIDLE_PRIORITY + 4, NULL);

	/* UART1 链路任务：默认什么都不做，只有大核用 0x61 打开时才去碰底盘链路。
	 * 优先级 +3：与 RGN/AUDIO 同级、低于上报任务——它是 10 ms 轮询的辅助任务。 */
	xTaskCreate(prvCmdquUart1Task, "CMDQU-UART1", configMINIMAL_STACK_SIZE * 2, NULL,
		    tskIDLE_PRIORITY + 3, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was either insufficient FreeRTOS heap memory available for the idle
    and/or timer tasks to be created, or vTaskStartScheduler() was called from
    User mode.  See the memory management section on the FreeRTOS web site for
    more details on the FreeRTOS heap http://www.freertos.org/a00111.html.  The
    mode from which main() is called is set in the C start up code and must be
    a privileged mode (not user mode). */
    for (;;)
        ;
}

void prvCmdQuRunTask(void *pvParameters)
{
	/* Remove compiler warning about unused parameter. */
	(void)pvParameters;

	cmdqu_t rtos_cmdq;
	cmdqu_t *cmdq;
	cmdqu_t *rtos_cmdqu_t;
	static int stop_ip = 0;
	int ret = 0;
	int flags;
	int valid;
	int send_to_cpu = SEND_TO_CPU1;

	unsigned int reg_base = MAILBOX_REG_BASE;

	/* set mcu_status to type1 running*/
	transfer_config.mcu_status = MCU_STATUS_RTOS_T1_RUNNING;

	if (transfer_config.conf_magic == C906_MAGIC_HEADER)
		send_to_cpu = SEND_TO_CPU1;
	else if (transfer_config.conf_magic == CA53_MAGIC_HEADER)
		send_to_cpu = SEND_TO_CPU0;
	/* to compatible code with linux side */
	cmdq = &rtos_cmdq;
	mbox_reg = (struct mailbox_set_register *) reg_base;
	mbox_done_reg = (struct mailbox_done_register *) (reg_base + 2);
	mailbox_context = (unsigned long *) (MAILBOX_REG_BUFF);

	cvi_spinlock_init();
	printf("prvCmdQuRunTask run\n");

	for (;;) {
		xQueueReceive(gTaskCtx[E_QUEUE_CMDQU].queHandle, &rtos_cmdq, portMAX_DELAY);

		switch (rtos_cmdq.cmd_id) {
#if ( configUSE_TRACE_FACILITY == 1 )
			case SYS_CMD_INFO_TRACE_SNAPSHOT_START:
				debug_printf("SYS_CMD_INFO_TRACE_SNAPSHOT_START\n");
				vTraceEnable(TRC_START);
				break;
			case SYS_CMD_INFO_TRACE_SNAPSHOT_STOP:
				snapshot.ptr = xTraceGetTraceBuffer();
				snapshot.size = uiTraceGetTraceBufferSize();
				snapshot.type = 0;
				rtos_cmdq.param_ptr = &snapshot;
				vTraceStop();
				flush_dcache_range(&snapshot, sizeof (struct trace_snapshot_t));
				flush_dcache_range(snapshot.ptr, snapshot.size);
				debug_printf("SYS_CMD_INFO_TRACE_SNAPSHOT_STOP PA =%lx\n", &snapshot);
				debug_printf("SYS_CMD_INFO_TRACE_SNAPSHOT_STOP ptr =%lx\n", snapshot.ptr);
				debug_printf("SYS_CMD_INFO_TRACE_SNAPSHOT_STOP size =%lx\n", snapshot.size);
				goto send_label;
				break;
#endif
			case SYS_CMD_INFO_DUMP_JPG:
				break;
			case SYS_CMD_INFO_DUMP_EN:
				dump_uart_enable();
				break;
			case SYS_CMD_INFO_DUMP_DIS:
				dump_uart_disable();
				break;
			case SYS_CMD_INFO_DUMP_MSG:
				/* SYS_CMD_INFO_DUMP_MSG is block mode*/
				rtos_cmdq.cmd_id = SYS_CMD_INFO_DUMP_MSG;
				rtos_cmdq.param_ptr = (unsigned int) dump_uart_msg();
				goto send_label;
				break;
			case SYS_CMD_INFO_LINUX_INIT_DONE:
				rtos_cmdq.cmd_id = SYS_CMD_INFO_RTOS_INIT_DONE;
				rtos_cmdq.param_ptr = &transfer_config;
				goto send_label;
				break;
			case SYS_CMD_INFO_STOP_ISR:
				stop_ip = 0;
				rtos_cmdq.cmd_id = SYS_CMD_INFO_STOP_ISR;
				rtos_cmdq.ip_id = IP_VI;
				xQueueSend(gTaskCtx[E_QUEUE_VI].queHandle, &rtos_cmdq, 0U);
				break;
			case SYS_CMD_INFO_STOP_ISR_DONE:
				// stop interrupt in order to avoid losing frame
				if (rtos_cmdq.ip_id == IP_VI) {
					stop_ip |= STOP_CMD_DONE_VI;
					rtos_cmdq.ip_id = IP_VCODEC;
					rtos_cmdq.cmd_id = SYS_CMD_INFO_STOP_ISR;
					xQueueSend(gTaskCtx[E_QUEUE_VCODEC].queHandle, &rtos_cmdq, 0U);
					break;
				}
				if (rtos_cmdq.ip_id == IP_VCODEC)
					stop_ip |= STOP_CMD_DONE_VCODE;
				if (stop_ip != STOP_CMD_DONE_ALL)
					break;
				else {
					// all isr of ip is disabled, and send msg back to linux
					rtos_cmdq.ip_id = IP_SYSTEM;
				}
			case CMDQU_CMD_SET_REPORT:
				/* 大核设置状态上报周期（毫秒），并回一条确认。
				 *
				 * 本固件的 tick 是 200 Hz（FreeRTOSConfig.h 的 configTICK_RATE_HZ），
				 * 即最小可表达周期为 5 ms；比它更小的值经 pdMS_TO_TICKS 会算成 0，
				 * 那样 vTaskDelay(0) 不延时、任务变成自由跑。这里直接兜到 5 ms，
				 * 避免用户下发 1 ms 之类"看起来合理"的值却得到完全不同的行为。 */
				g_cmdqu_report_period_ms =
					rtos_cmdq.param_ptr ? rtos_cmdq.param_ptr : 1000U;
				if (g_cmdqu_report_period_ms < CMDQU_MIN_PERIOD_MS)
					g_cmdqu_report_period_ms = CMDQU_MIN_PERIOD_MS;
				rtos_cmdq.param_ptr = g_cmdqu_report_period_ms;
				goto send_label;
			case CMDQU_CMD_UART1_MODE:
				/* 开关小核侧的 UART1（底盘链路）握手测试。
				 *
				 * 0 = 关闭：不碰 UART1 寄存器、不动 FMUX，零副作用；
				 * 1 = 握手模式：把 A18/A19 切给 UART1，每隔一段时间发一次
				 *     CMD_INIT 并统计底盘板的 ACK，统计结果作为 kind=6 的
				 *     环消息上报。
				 *
				 * 之所以要显式开关：厂商 Linux 上 AKA-00 应用正在使用
				 * /dev/ttyS1，小核若同时往同一条链路发帧会互相干扰。 */
				if (rtos_cmdq.param_ptr > 2U)
					g_cmdqu_uart1_mode = 2U; /* 只定义 0/1/2，越界按最大档处理 */
				else
				g_cmdqu_uart1_mode = rtos_cmdq.param_ptr;
				rtos_cmdq.param_ptr = g_cmdqu_uart1_mode;
				goto send_label;
			case CMDQU_CMD_SET_TARGET:
				/* 下发目标速度：param 低 16 位是左轮 PWM(i16)，高 16 位是右轮 PWM(i16)。
				 *
				 * 这个命令同时充当**心跳**：小核那边超过 CMDQU_UART1_WD_MS(500 ms)
				 * 没收到新的目标就会自动停车。也就是说"大核停止喂目标"= 车停，
				 * 而不是像原来那样保持最后的速度一直跑。
				 *
				 * 收到目标即进入 mode=3（周期控制），大核不必再单独发 0x61。 */
				{
					/* 限幅：两个 16 位分量各自钳到 ±CMDQU_UART1_PWM_LIMIT。
					 * 宁可停在边界上，也不把超量程 PWM 送给底盘板。 */
					int l = (short)(rtos_cmdq.param_ptr & 0xFFFFU);
					int r = (short)(rtos_cmdq.param_ptr >> 16);

					if (l > CMDQU_UART1_PWM_LIMIT)
						l = CMDQU_UART1_PWM_LIMIT;
					if (l < -CMDQU_UART1_PWM_LIMIT)
						l = -CMDQU_UART1_PWM_LIMIT;
					if (r > CMDQU_UART1_PWM_LIMIT)
						r = CMDQU_UART1_PWM_LIMIT;
					if (r < -CMDQU_UART1_PWM_LIMIT)
						r = -CMDQU_UART1_PWM_LIMIT;
					g_cmdqu_uart1_target_lr =
						((unsigned int)(unsigned short)l) |
						(((unsigned int)(unsigned short)r) << 16);
				}
				g_cmdqu_uart1_target_set = 1U;
				/* 刷新心跳截止时刻，并清掉"已因超时停车"的标志——
				 * 新目标一到就恢复下发，这正是心跳语义。 */
				g_cmdqu_uart1_wd_deadline_ms =
					(unsigned long long)xTaskGetTickCount() *
						(1000U / configTICK_RATE_HZ) +
					g_cmdqu_uart1_wd_ms;
				g_cmdqu_uart1_wd_tripped = 0U;
				if (g_cmdqu_uart1_mode < 3U)
					g_cmdqu_uart1_mode = 3U;
				rtos_cmdq.param_ptr = g_cmdqu_uart1_target_lr;
				goto send_label;
			case CMDQU_CMD_SET_CTRL_PERIOD:
				/* 控制周期（毫秒），按基础节拍量化：最小 5 ms、最大 1000 ms */
				g_cmdqu_uart1_ctrl_period_ms = rtos_cmdq.param_ptr;
				if (g_cmdqu_uart1_ctrl_period_ms < CMDQU_UART1_TICK_MS)
					g_cmdqu_uart1_ctrl_period_ms = CMDQU_UART1_TICK_MS;
				if (g_cmdqu_uart1_ctrl_period_ms > 1000U)
					g_cmdqu_uart1_ctrl_period_ms = 1000U;
				rtos_cmdq.param_ptr = g_cmdqu_uart1_ctrl_period_ms;
				goto send_label;
			case CMDQU_CMD_SET_WD:
				/* 心跳看门狗窗口（毫秒）：默认 500，演示时可以调大到几秒，
				 * 这样"停喂心跳 → 自动停车"能肉眼看清。范围 100~10000 ms。 */
				g_cmdqu_uart1_wd_ms = rtos_cmdq.param_ptr;
				if (g_cmdqu_uart1_wd_ms < 100U)
					g_cmdqu_uart1_wd_ms = 100U;
				if (g_cmdqu_uart1_wd_ms > 10000U)
					g_cmdqu_uart1_wd_ms = 10000U;
				rtos_cmdq.param_ptr = g_cmdqu_uart1_wd_ms;
				goto send_label;
			case SYS_CMD_INFO_LINUX:
			default:
send_label:
				/* used to send command to linux*/
				rtos_cmdqu_t = (cmdqu_t *) mailbox_context;

				debug_printf("RTOS_CMDQU_SEND\n");
				debug_printf("ip_id=%d cmd_id=%d param_ptr=%x\n", cmdq->ip_id, cmdq->cmd_id, (unsigned int)cmdq->param_ptr);
				debug_printf("mailbox_context = %x\n", mailbox_context);
				debug_printf("linux_cmdqu_t = %x\n", rtos_cmdqu_t);
				debug_printf("cmdq->ip_id = %d\n", cmdq->ip_id);
				debug_printf("cmdq->cmd_id = %d\n", cmdq->cmd_id);
				debug_printf("cmdq->block = %d\n", cmdq->block);
				debug_printf("cmdq->para_ptr = %x\n", cmdq->param_ptr);

				drv_spin_lock_irqsave(&mailbox_lock, flags);
				if (flags == MAILBOX_LOCK_FAILED) {
					printf("[%s][%d] drv_spin_lock_irqsave failed! ip_id = %d , cmd_id = %d\n" , cmdq->ip_id , cmdq->cmd_id);
					break;
				}

				for (valid = 0; valid < MAILBOX_MAX_NUM; valid++) {
					if (rtos_cmdqu_t->resv.valid.linux_valid == 0 && rtos_cmdqu_t->resv.valid.rtos_valid == 0) {
						// mailbox buffer context is 4 bytes write access
						int *ptr = (int *)rtos_cmdqu_t;

						cmdq->resv.valid.rtos_valid = 1;
						*ptr = ((cmdq->ip_id << 0) | (cmdq->cmd_id << 8) | (cmdq->block << 15) |
								(cmdq->resv.valid.linux_valid << 16) |
								(cmdq->resv.valid.rtos_valid << 24));
						rtos_cmdqu_t->param_ptr = cmdq->param_ptr;
						debug_printf("rtos_cmdqu_t->linux_valid = %d\n", rtos_cmdqu_t->resv.valid.linux_valid);
						debug_printf("rtos_cmdqu_t->rtos_valid = %d\n", rtos_cmdqu_t->resv.valid.rtos_valid);
						debug_printf("rtos_cmdqu_t->ip_id =%x %d\n", &rtos_cmdqu_t->ip_id, rtos_cmdqu_t->ip_id);
						debug_printf("rtos_cmdqu_t->cmd_id = %d\n", rtos_cmdqu_t->cmd_id);
						debug_printf("rtos_cmdqu_t->block = %d\n", rtos_cmdqu_t->block);
						debug_printf("rtos_cmdqu_t->param_ptr addr=%x %x\n", &rtos_cmdqu_t->param_ptr, rtos_cmdqu_t->param_ptr);
						debug_printf("*ptr = %x\n", *ptr);
						// clear mailbox
						mbox_reg->cpu_mbox_set[send_to_cpu].cpu_mbox_int_clr.mbox_int_clr = (1 << valid);
						// trigger mailbox valid to rtos
						mbox_reg->cpu_mbox_en[send_to_cpu].mbox_info |= (1 << valid);
						mbox_reg->mbox_set.mbox_set = (1 << valid);
						break;
					}
					rtos_cmdqu_t++;
				}
				drv_spin_unlock_irqrestore(&mailbox_lock, flags);
				if (valid >= MAILBOX_MAX_NUM) {
				    printf("No valid mailbox is available\n");
				    return -1;
				}
				break;
		}
	}
}

void prvQueueISR(void)
{
	//printf("prvQueueISR\n");

	unsigned char set_val;
//	unsigned char done_val;
	unsigned char valid_val;
	int i;
	cmdqu_t *cmdq;
	BaseType_t YieldRequired = pdFALSE;

	set_val = mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_int.mbox_int;
	/* Now, we do not implement info back feature */
	// done_val = mbox_done_reg->cpu_mbox_done[RECEIVE_CPU].cpu_mbox_int_int.mbox_int;

	if (set_val) {
		for(i = 0; i < MAILBOX_MAX_NUM; i++) {
			valid_val = set_val  & (1 << i);

			if (valid_val) {
				cmdqu_t rtos_cmdq;
				cmdq = (cmdqu_t *)(mailbox_context) + i;

				debug_printf("mailbox_context =%x\n", mailbox_context);
				debug_printf("sizeof mailbox_context =%x\n", sizeof(cmdqu_t));
				/* mailbox buffer context is send from linux, clear mailbox interrupt */
				mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_clr.mbox_int_clr = valid_val;
				// need to disable enable bit
				mbox_reg->cpu_mbox_en[RECEIVE_CPU].mbox_info &= ~valid_val;

				// copy cmdq context (8 bytes) to buffer ASAP
				*((unsigned long *) &rtos_cmdq) = *((unsigned long *)cmdq);
				/* need to clear mailbox interrupt before clear mailbox buffer */
				*((unsigned long*) cmdq) = 0;

				/* mailbox buffer context is send from linux*/
				if (rtos_cmdq.resv.valid.linux_valid == 1) {
					debug_printf("cmdq=%x\n", cmdq);
					debug_printf("cmdq->ip_id =%d\n", rtos_cmdq.ip_id);
					debug_printf("cmdq->cmd_id =%d\n", rtos_cmdq.cmd_id);
					debug_printf("cmdq->param_ptr =%x\n", rtos_cmdq.param_ptr);
					debug_printf("cmdq->block =%x\n", rtos_cmdq.block);
					debug_printf("cmdq->linux_valid =%d\n", rtos_cmdq.resv.valid.linux_valid);
					debug_printf("cmdq->rtos_valid =%x\n", rtos_cmdq.resv.valid.rtos_valid);
					switch (rtos_cmdq.ip_id) {
					case IP_ISP:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_ISP].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_VCODEC:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_VCODEC].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_VI:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_VI].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_RGN:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_RGN].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_AUDIO:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_AUDIO].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_SYSTEM:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_CMDQU].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					case IP_CAMERA:
						xQueueSendFromISR(gTaskCtx[E_QUEUE_CAMERA].queHandle, &rtos_cmdq, &YieldRequired);
						break;
					default:
						printf("unknown ip_id =%d cmd_id=%d\n", rtos_cmdq.ip_id, rtos_cmdq.cmd_id);
						break;
					}
					portYIELD_FROM_ISR(YieldRequired);
				} else
					printf("rtos cmdq is not valid %d, ip=%d , cmd=%d\n",
						rtos_cmdq.resv.valid.rtos_valid, rtos_cmdq.ip_id, rtos_cmdq.cmd_id);
			}
		}
	}
}

/****************************************************************************
 * 跨核共享缓冲：状态上报
 *
 * 布局与大核侧 drivers/ipc/sg2002-cmdqu/src/shm.rs 的定义逐字段对应：
 *   0x0000 头部（192 字节 = 3 条 cache line，按"谁写"分行）
 *   0x00C0 同步块（128 字节 = 2 条 cache line：大核一行、小核一行）
 *   0x0140 Ring A（大核 → 小核）
 *   0x1140 Ring B（小核 → 大核）  ← 本任务写这个环
 *
 * 小核的 D-cache 是打开的，而大核按非缓存访问同一块内存，因此写环条目必须
 * 先写载荷并 clean，再更新 head 并 clean：顺序颠倒会让大核读到指向旧载荷的 head。
 *
 * 头部行 2（偏移 0x80）由本核独占写：发布 head 时只 clean 这一行，行 1 里
 * 大核拥有的 host_to_rtos_head/host_to_rtos_tail/rtos_to_host_tail 不会被
 * 本核的整行回写覆盖。
 ****************************************************************************/

/* 环的常量部分；可变索引按写入方单独放在头部的行 1 / 行 2。 */
struct cmdqu_ring_layout {
	unsigned int offset;
	unsigned int entries;
	unsigned int entry_size;
};

struct cmdqu_shm_header {
	/* ── 行 0（0x00..0x3F）：常量，大核初始化时写一次，之后双方只读 ── */
	unsigned int magic;
	unsigned int version;
	unsigned int size;
	unsigned int header_size;
	struct cmdqu_ring_layout host_to_rtos;
	struct cmdqu_ring_layout rtos_to_host;
	unsigned char reserved0[24];

	/* ── 行 1（0x40..0x7F）：大核写、小核只读 ── */
	unsigned int host_to_rtos_head;
	unsigned int host_to_rtos_tail;
	unsigned int rtos_to_host_tail;
	unsigned char reserved1[52];

	/* ── 行 2（0x80..0xBF）：小核写、大核只读 ── */
	unsigned int rtos_to_host_head;
	unsigned char reserved2[60];
};

/* 布局断言：任何一处对不上都会在编译期报错，避免两侧悄悄错位。 */
_Static_assert(sizeof(struct cmdqu_shm_header) == 192, "shm header must be 3 cache lines");
_Static_assert(__builtin_offsetof(struct cmdqu_shm_header, rtos_to_host_head) == 128,
	       "rtos_to_host_head must sit in its own cache line (offset 128)");
_Static_assert(__builtin_offsetof(struct cmdqu_shm_header, rtos_to_host_tail) == 72,
	       "rtos_to_host_tail is host-owned and must stay out of the rtos line");

struct cmdqu_ring_entry {
	unsigned int kind;
	unsigned int seq;
	unsigned int len;
	unsigned int crc;
	unsigned char payload[48];
};

struct cmdqu_state_snapshot {
	unsigned int seq;
	unsigned int flags;
	unsigned long long rtos_time_us;
	int wheel_counts[2];
	unsigned int photogate;
	unsigned short duty_permille[2];
};

static unsigned int g_cmdqu_report_seq = 0U;

/* ── UART1 的极简轮询驱动（不占中断，够握手用） ── */

static void cmdqu_uart1_hw_init(void)
{
	/* FMUX：把 A18/A19 焊盘切到 UART1 TX/RX。幂等，与 AKA-00 的 uart_init.sh
	 * 写的是同一组寄存器；小核自己写一遍，就不依赖大核是否执行过初始化脚本。 */
	mmio_write_32(CMDQU_FMUX_PBASE + CMDQU_FMUX_UART1_TX_OFF, CMDQU_FMUX_UART1_FSEL);
	mmio_write_32(CMDQU_FMUX_PBASE + CMDQU_FMUX_UART1_RX_OFF, CMDQU_FMUX_UART1_FSEL);

	/* 115200 8N1。UART1 的输入时钟按 26 MHz 记（Linux 设备树里
	 * clock-frequency = 0x18CBA00），DLL=14 → 26e6/(16*14) = 116071 baud，
	 * 偏差 +0.76%，在 UART 容差内；换成 25 MHz 假设会偏 -3.1%，所以用 26 MHz。 */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_LCR, 0x83U);     /* DLAB=1, 8N1 */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_RBR_THR, 14U);   /* DLL */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_IER_DLM, 0U);    /* DLM */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_LCR, 0x03U);     /* DLAB=0, 8N1 */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_FCR_IIR, 0x07U); /* FIFO 使能并清空 */
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_IER_DLM, 0U);    /* 关中断，轮询 */
}

static void cmdqu_uart1_putc(unsigned char c)
{
	unsigned int guard = 200000U;

	while (!(mmio_read_32(CMDQU_UART1_BASE + CMDQU_UART1_LSR) & CMDQU_LSR_THR_EMPTY) &&
	       guard--)
		;
	mmio_write_32(CMDQU_UART1_BASE + CMDQU_UART1_RBR_THR, (unsigned int)c);
}

/* 发一帧底盘协议报文：AA 55 cmd len payload… cksum（逐字节异或，含 cmd 与 len）。
 * 载荷最长 4 字节就够（CONFIG 的 ppr+pwm_freq、SET_SPEED 的 id+两字节 PWM）。 */
static void cmdqu_uart1_send_frame(unsigned char cmd, const unsigned char *payload,
				   unsigned int len)
{
	unsigned char buf[9];
	unsigned int i, acc;

	if (len > sizeof(buf) - 5U)
		return;
	buf[0] = 0xAAU;
	buf[1] = 0x55U;
	buf[2] = cmd;
	buf[3] = (unsigned char)len;
	acc = (unsigned int)cmd ^ len;
	for (i = 0U; i < len; i++) {
		buf[4U + i] = payload[i];
		acc ^= payload[i];
	}
	buf[4U + len] = (unsigned char)acc;
	for (i = 0U; i < 5U + len; i++)
		cmdqu_uart1_putc(buf[i]);
	g_cmdqu_uart1_last_sent = cmd;
}

static void cmdqu_uart1_send_init(void)
{
	cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_INIT, NULL, 0U);
	g_cmdqu_uart1_sent++;
}

/* 命令自检的第 stage 步（0..CMDQU_UART1_PROBE_STEPS-1）。**全部是零速度/停车指令**：
 * CONFIG 只改底盘板的闭环参数，SET_SPEED 给 0，STOP 就是停车。 */
static void cmdqu_uart1_probe_step(unsigned int stage)
{
	unsigned char payload[4];

	switch (stage) {
	case 0U: /* CMD_CONFIG：ppr=4680（1:90 减速比）、pwm_freq=20000（与 AKA-00 一致） */
		payload[0] = 0x12U;
		payload[1] = 0x48U;
		payload[2] = 0x4EU;
		payload[3] = 0x20U;
		cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_CONFIG, payload, 4U);
		break;
	case 1U: /* SET_SPEED(motor 0, 0) */
		payload[0] = 0x00U;
		payload[1] = 0x00U;
		payload[2] = 0x00U;
		cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_SET_SPEED, payload, 3U);
		break;
	case 2U: /* SET_SPEED(motor 1, 0) */
		payload[0] = 0x01U;
		payload[1] = 0x00U;
		payload[2] = 0x00U;
		cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_SET_SPEED, payload, 3U);
		break;
	case 3U: /* STOP(motor 0) */
		payload[0] = 0x00U;
		cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_STOP, payload, 1U);
		break;
	default: /* STOP(motor 1) */
		payload[0] = 0x01U;
		cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_STOP, payload, 1U);
		break;
	}
}

/* 在缓冲区开头尝试解析一帧；返回 1 表示消费了至少一个字节（调用方应继续循环） */
static unsigned int cmdqu_uart1_parse(unsigned char *buf, unsigned int *n,
				      unsigned long long now_us)
{
	unsigned int len, i, acc, consumed;

	if (*n < 2U)
		return 0U;
	if (buf[0] != 0xAAU || buf[1] != 0x55U) {
		for (i = 1U; i < *n; i++)
			buf[i - 1U] = buf[i];
		(*n)--;
		return 1U;
	}
	if (*n < 4U)
		return 0U;
	len = buf[3];
	if (*n < 5U + len)
		return 0U; /* 还没收全，等下一轮 */

	acc = (unsigned int)buf[2] ^ (unsigned int)buf[3];
	for (i = 0U; i < len; i++)
		acc ^= buf[4U + i];
	if (acc == buf[4U + len]) {
		g_cmdqu_uart1_last_reply = buf[2];
		g_cmdqu_uart1_last_echo = (len >= 1U) ? buf[4] : 0U;
		g_cmdqu_uart1_reply_cmd = buf[2];
		g_cmdqu_uart1_reply_len = (len > sizeof(g_cmdqu_uart1_reply_payload))
						 ? sizeof(g_cmdqu_uart1_reply_payload)
						 : len;
		for (i = 0U; i < g_cmdqu_uart1_reply_len; i++)
			g_cmdqu_uart1_reply_payload[i] = buf[4U + i];
		if (buf[2] == CMDQU_MOTOR_RSP_ACK) {
			g_cmdqu_uart1_ack++;
			g_cmdqu_uart1_last_ack_us = now_us;
			g_cmdqu_uart1_replies++;
		} else if (buf[2] == CMDQU_MOTOR_RSP_NACK) {
			g_cmdqu_uart1_nack++;
			g_cmdqu_uart1_replies++;
		} else {
			/* 校验通过但不是 ACK/NACK：例如 RSP_STATUS(0x91)、RSP_RPM_DATA(0x90)。
			 * 这些是**正常回包**，早前误计进"帧错误"，导致读数看起来像链路很脏
			 * （实板上"帧错误"数恰好等于 GET_STATUS 的次数，才暴露了这个标签错误）。 */
			g_cmdqu_uart1_rsp_other++;
		}
	} else {
		g_cmdqu_uart1_errs++; /* 只有校验失败才计入真正的帧错误 */
	}

	consumed = 5U + len;
	for (i = consumed; i < *n; i++)
		buf[i - consumed] = buf[i];
	*n -= consumed;
	return 1U;
}

/* CRC32（IEEE 802.3 多项式 0xEDB88320），与大核侧实现一致 */
static unsigned int cmdqu_crc32(const unsigned char *data, unsigned int len)
{
	unsigned int crc = 0xFFFFFFFFU;
	unsigned int i;
	int bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
	}
	return ~crc;
}

/* UART1 握手任务：大核用命令 0x61 打开后才运行，默认完全不碰这条链路。
 *
 * 轮询周期 10 ms：115200 下一个字节约 87 µs，10 ms 足够（FIFO 有 16 级），
 * 而且不做中断，避免与 UART0 的调试输出抢中断资源。
 * 发送节奏：还没握手成功就每 500 ms 补一帧 CMD_INIT；握手成功后放慢到 5 s
 * 保活一次——这样即使底盘板中途复位（回到 UNINIT），也会被重新拉起来。 */
void prvCmdquUart1Task(void *pvParameters)
{
	static unsigned char rx[64];
	unsigned int n = 0U;
	unsigned int enabled = 0U;
	unsigned long long last_send_ms = 0ULL;
	unsigned long long now_ms;
	unsigned int prev_replies = 0U;
	unsigned long long probe_wait_ms = 0ULL;
	unsigned int probe_answered = 0U;
	unsigned int ctrl_div = 0U;       /* 基础节拍累加器，凑够一个控制周期 */
	unsigned int dt_prev_tick = 0U;   /* 上一次控制帧所在 tick，用来量实测周期 */
	unsigned int ctrl_cfg_sent = 0U;  /* 周期控制模式下是否已经发过 CMD_CONFIG */
	TickType_t last_wake = xTaskGetTickCount();

	(void)pvParameters;

	for (;;) {
		/* 基础节拍 1 个 tick：既够频繁地抽干 RX FIFO（一个周期最多 ~20 字节），
		 * 又让控制周期只能是 5 ms 的整数倍——**量化是固定的相位偏移，不是抖动**。 */
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CMDQU_UART1_TICK_MS));
		now_ms = (unsigned long long)xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);

		if (g_cmdqu_uart1_mode == 0U) {
			if (enabled) {
				/* 关掉时清空统计：下次打开从干净状态开始，计数才有意义 */
				enabled = 0U;
				n = 0U;
				g_cmdqu_uart1_sent = 0U;
				g_cmdqu_uart1_ack = 0U;
				g_cmdqu_uart1_nack = 0U;
				g_cmdqu_uart1_rx_bytes = 0U;
				g_cmdqu_uart1_errs = 0U;
				g_cmdqu_uart1_rsp_other = 0U;
				g_cmdqu_uart1_last_ack_us = 0ULL;
				g_cmdqu_uart1_probe_stage = 0U;
				g_cmdqu_uart1_probe_err = 0U;
				g_cmdqu_uart1_cfg_ack = 0U;
				g_cmdqu_uart1_speed_ack = 0U;
				g_cmdqu_uart1_stop_ack = 0U;
				g_cmdqu_uart1_last_sent = 0U;
				g_cmdqu_uart1_last_reply = 0U;
				g_cmdqu_uart1_last_echo = 0U;
				g_cmdqu_uart1_replies = 0U;
				prev_replies = 0U;
				g_cmdqu_uart1_target_set = 0U;
				g_cmdqu_uart1_target_lr = 0U;
				g_cmdqu_uart1_ctrl_count = 0U;
				g_cmdqu_uart1_wd_stops = 0U;
				g_cmdqu_uart1_dt_max = 0U;
				g_cmdqu_uart1_dt_min = 0xFFFFFFFFU;
				g_cmdqu_uart1_dt_sum = 0U;
				g_cmdqu_uart1_dt_count = 0U;
				g_cmdqu_uart1_hist_lt = 0U;
				g_cmdqu_uart1_hist_eq = 0U;
				g_cmdqu_uart1_hist_p1 = 0U;
				g_cmdqu_uart1_hist_p2 = 0U;
				g_cmdqu_uart1_hist_ge = 0U;
				g_cmdqu_uart1_ch_state = 0xFFU;
				g_cmdqu_uart1_rpm_l = 0;
				g_cmdqu_uart1_rpm_r = 0;
				ctrl_div = 0U;
				dt_prev_tick = 0U;
				ctrl_cfg_sent = 0U;
			}
			continue;
		}
		if (!enabled) {
			cmdqu_uart1_hw_init();
			enabled = 1U;
			last_send_ms = now_ms - 60000ULL; /* 让下面立刻发第一帧 */
			probe_wait_ms = now_ms;
		}

		/* 收：把 FIFO 抽干，交给解析器逐帧处理 */
		while (mmio_read_32(CMDQU_UART1_BASE + CMDQU_UART1_LSR) & CMDQU_LSR_DATA_READY) {
			unsigned char c = (unsigned char)mmio_read_32(
				CMDQU_UART1_BASE + CMDQU_UART1_RBR_THR);
			unsigned int i;

			g_cmdqu_uart1_rx_bytes++;
			if (n < sizeof(rx)) {
				rx[n++] = c;
			} else {
				/* 溢出：丢最旧的字节，保证不会卡死 */
				for (i = 1U; i < sizeof(rx); i++)
					rx[i - 1U] = rx[i];
				rx[sizeof(rx) - 1U] = c;
			}
		}
		while (cmdqu_uart1_parse(rx, &n, now_ms * 1000ULL))
			;

		/* 新回包：按"最后发出的命令号"归属，并推进自检状态机 */
		if (g_cmdqu_uart1_replies != prev_replies) {
			prev_replies = g_cmdqu_uart1_replies;
			probe_wait_ms = now_ms;
			probe_answered = 1U;
			/* 底盘板的 ACK/NACK 会回显被确认的命令号，对不上就记一次错 */
			if (g_cmdqu_uart1_last_echo != g_cmdqu_uart1_last_sent)
				g_cmdqu_uart1_probe_err++;
			switch (g_cmdqu_uart1_last_sent) {
			case CMDQU_MOTOR_CMD_CONFIG:
				g_cmdqu_uart1_cfg_ack++;
				break;
			case CMDQU_MOTOR_CMD_SET_SPEED:
				g_cmdqu_uart1_speed_ack++;
				break;
			case CMDQU_MOTOR_CMD_STOP:
				g_cmdqu_uart1_stop_ack++;
				break;
			default: /* INIT 已在 g_cmdqu_uart1_ack 里统计 */
				break;
			}
		}

		/* mode>=2：握手成功后按 300 ms 间隔跑一遍命令自检；
		 * 某一步没等到回包就重发并记一次 err，避免卡死。 */
		/* 只在本步专属的 mode=2 里跑：mode=3（周期控制）下若也跑自检，
		 * 会往控制流里插 CONFIG/STOP，把底盘搅乱（实板已经踩到过）。 */
		if (g_cmdqu_uart1_mode == 2U && g_cmdqu_uart1_ack != 0U &&
		    g_cmdqu_uart1_probe_stage < CMDQU_UART1_PROBE_STEPS) {
			if (now_ms - probe_wait_ms >= CMDQU_UART1_PROBE_GAP_MS) {
				/* 上一步没等到回包就记一次错，再继续下一步——不卡死 */
				if (g_cmdqu_uart1_probe_stage > 0U && probe_answered == 0U)
					g_cmdqu_uart1_probe_err++;
				cmdqu_uart1_probe_step(g_cmdqu_uart1_probe_stage);
				g_cmdqu_uart1_probe_stage++;
				probe_wait_ms = now_ms;
				probe_answered = 0U;
			}
		}

		/* 发：未握手 500 ms 重试；已握手 5 s 保活 */
		if (g_cmdqu_uart1_ack == 0U) {
			if (now_ms - last_send_ms >= 500ULL) {
				cmdqu_uart1_send_init();
				last_send_ms = now_ms;
			}
		} else if (g_cmdqu_uart1_mode < 3U && now_ms - last_send_ms >= 5000ULL) {
			/* 只握手/自检时才发保活 INIT：底盘板收到 INIT 会回到 IDLE，
			 * 周期控制模式下这会打断正在跑的控制，所以那一档不发。 */
			cmdqu_uart1_send_init();
			last_send_ms = now_ms;
		}

		/* mode>=3：周期控制。每个控制周期下发一次目标速度；大核心跳超时则自动停车。 */
		if (g_cmdqu_uart1_mode >= 3U) {
			unsigned int div = g_cmdqu_uart1_ctrl_period_ms / CMDQU_UART1_TICK_MS;

			if (div == 0U)
				div = 1U;
			if (++ctrl_div >= div) {
				unsigned int tick_now = (unsigned int)xTaskGetTickCount();
				unsigned int sent_this_cycle = 0U;

				ctrl_div = 0U;
				/* 实测控制周期（tick）：固定量化不算抖动，这个值应当恒等于 div */
				if (dt_prev_tick != 0U) {
					unsigned int dt = tick_now - dt_prev_tick;

					if (dt > g_cmdqu_uart1_dt_max)
						g_cmdqu_uart1_dt_max = dt;
					if (dt < g_cmdqu_uart1_dt_min)
						g_cmdqu_uart1_dt_min = dt;
					g_cmdqu_uart1_dt_sum += dt;
					g_cmdqu_uart1_dt_count++;
					if (dt < div)
						g_cmdqu_uart1_hist_lt++;
					else if (dt == div)
						g_cmdqu_uart1_hist_eq++;
					else if (dt == div + 1U)
						g_cmdqu_uart1_hist_p1++;
					else if (dt == div + 2U)
						g_cmdqu_uart1_hist_p2++;
					else
						g_cmdqu_uart1_hist_ge++;
				}
				dt_prev_tick = tick_now;

				/* 握手成功后先配一次闭环参数（ppr=4680、20 kHz），
				 * 与本周期后续的控制帧错开，避免底盘板来不及处理。 */
				if (g_cmdqu_uart1_ack != 0U && ctrl_cfg_sent == 0U) {
					unsigned char cfg[4] = {0x12U, 0x48U, 0x4EU, 0x20U};

					cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_CONFIG, cfg, 4U);
					ctrl_cfg_sent = 1U;
					sent_this_cycle = 1U;
				}

				if (!sent_this_cycle) {
					if (!g_cmdqu_uart1_target_set) {
						/* 还没收到目标：不发控制帧，等大核下发 */
					} else if (g_cmdqu_uart1_wd_tripped) {
						/* 已因心跳超时停车：保持静默，等新的目标把它清掉 */
					} else if (now_ms > g_cmdqu_uart1_wd_deadline_ms) {
						/* ★ 安全兜底：大核 500 ms 没刷新目标就立即停车。
						 * 这正是"OS/应用失效后车不会一直跑"的那条保证。 */
						unsigned char id;

						id = 0U;
						cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_STOP, &id, 1U);
						id = 1U;
						cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_STOP, &id, 1U);
						g_cmdqu_uart1_target_lr = 0U;
						g_cmdqu_uart1_wd_stops++;
						g_cmdqu_uart1_wd_tripped = 1U;
					} else {
						unsigned char sp[3];
						int left = (short)(g_cmdqu_uart1_target_lr & 0xFFFFU);
						int right = (short)(g_cmdqu_uart1_target_lr >> 16);

						sp[0] = 0U;
						sp[1] = (unsigned char)((left >> 8) & 0xFF);
						sp[2] = (unsigned char)(left & 0xFF);
						cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_SET_SPEED, sp, 3U);
						sp[0] = 1U;
						sp[1] = (unsigned char)((right >> 8) & 0xFF);
						sp[2] = (unsigned char)(right & 0xFF);
						cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_SET_SPEED, sp, 3U);
					}
				}
				g_cmdqu_uart1_ctrl_count++;

				/* 每 N 个控制周期读一次底盘状态与转速（RSP_STATUS = 5 字节） */
				if ((g_cmdqu_uart1_ctrl_count % CMDQU_UART1_STATUS_EVERY) == 0U)
					cmdqu_uart1_send_frame(CMDQU_MOTOR_CMD_GET_STATUS, NULL, 0U);

				if (g_cmdqu_uart1_reply_cmd == CMDQU_MOTOR_RSP_STATUS &&
				    g_cmdqu_uart1_reply_len >= 5U) {
					unsigned char b0 = g_cmdqu_uart1_reply_payload[0];
					unsigned char b1 = g_cmdqu_uart1_reply_payload[1];
					unsigned char b2 = g_cmdqu_uart1_reply_payload[2];
					unsigned char b3 = g_cmdqu_uart1_reply_payload[3];
					unsigned char b4 = g_cmdqu_uart1_reply_payload[4];

					g_cmdqu_uart1_ch_state = b0;
					g_cmdqu_uart1_rpm_l = (int)(short)(((unsigned int)b1 << 8) | b2);
					g_cmdqu_uart1_rpm_r = (int)(short)(((unsigned int)b3 << 8) | b4);
				}
			}
		}
	}
}

/* 往状态环发布一条消息。
 *
 * 只有**一个**任务调用它（`prvCmdquReportTask`）：环的 `head` 是"读-改-写"，
 * 两个任务各自发布就会互相覆盖（小核内部没有跨任务的自旋锁），所以 UART1 任务
 * 只更新计数，由上报任务统一发。顺序仍然是"先写载荷并 clean、再发布 head 并
 * clean"，且只回写小核独占的那条 cache line。 */
static void cmdqu_shm_publish(volatile struct cmdqu_shm_header *hdr, unsigned int kind,
			      const unsigned char *payload, unsigned int len)
{
	struct cmdqu_ring_entry entry;
	volatile unsigned char *ring =
		(volatile unsigned char *)(CMDQU_SHM_BASE + CMDQU_RING_B_OFF);
	unsigned int head, idx, i;
	unsigned char *src;

	if (len > sizeof(entry.payload))
		return;
	if (hdr->magic != CMDQU_SHM_MAGIC || hdr->version != CMDQU_SHM_VERSION)
		return;
	if (hdr->rtos_to_host.entry_size != CMDQU_RING_ENTRY_SZ ||
	    hdr->rtos_to_host.entries == 0U)
		return;

	head = hdr->rtos_to_host_head;
	idx = head % hdr->rtos_to_host.entries;

	entry.kind = kind;
	entry.seq = head;
	entry.len = len;
	entry.crc = 0U;
	for (i = 0U; i < sizeof(entry.payload); i++)
		entry.payload[i] = 0U;
	for (i = 0U; i < len; i++)
		entry.payload[i] = payload[i];
	entry.crc = cmdqu_crc32(entry.payload, len);

	src = (unsigned char *)&entry;
	{
		volatile unsigned char *dst = ring + idx * CMDQU_RING_ENTRY_SZ;
		for (i = 0U; i < sizeof(entry); i++)
			dst[i] = src[i];
		flush_dcache_range((uintptr_t)dst, sizeof(entry));
	}
	hdr->rtos_to_host_head = head + 1U;
	flush_dcache_range((uintptr_t)(CMDQU_SHM_BASE + CMDQU_SHM_RTOS_LINE_OFF), 64U);
}

void prvCmdquReportTask(void *pvParameters)
{
	volatile struct cmdqu_shm_header *hdr =
		(volatile struct cmdqu_shm_header *)CMDQU_SHM_BASE;
	/* 绝对延时基准：vTaskDelay 是"相对现在"，一次被推迟之后每个周期都跟着后移；
	 * vTaskDelayUntil 则始终对齐到 tick 边界，采样节拍不会累积漂移。 */
	TickType_t last_wake = xTaskGetTickCount();
	unsigned int last_period = 0U;
	unsigned long long last_uart_pub_ms = 0ULL;

	(void)pvParameters;

	for (;;) {
		unsigned int period = g_cmdqu_report_period_ms;
		struct cmdqu_state_snapshot snap;
		unsigned long long now_ms;

		{
			TickType_t ticks = pdMS_TO_TICKS(period ? period : 1000U);
			/* 兜底：任何小于一个 tick 的周期都必须至少延时 1 tick，
			 * 否则 vTaskDelay(0) 会让任务自由跑（实测会把"1 ms"变成 ~818 µs/次）。 */
			if (ticks == 0U)
				ticks = 1U;
			/* 周期被改过时重新对齐基准，避免按旧基准一次性补偿一大截。 */
			if (period != last_period) {
				last_wake = xTaskGetTickCount();
				last_period = period;
			}
			vTaskDelayUntil(&last_wake, ticks);
		}

		/* ★ 先把头部那一行从主存重新读进来（invalidate），再读取其中的字段。
		 *
		 * 小核比大核先启动，启动后就把头部读进了 D-cache；此后若只用缓存副本，
		 * 每轮 flush 都会把这份旧副本整行写回，覆盖掉大核刚写入的头部——
		 * 实板表现就是 head 从上一个开机周期的值继续累加，而不是从 0 开始。
		 * 这里每轮先 invalidate，保证读到的是大核最新写入的头部。 */
		inv_dcache_range((uintptr_t)hdr, sizeof(*hdr));

		/* 大核还没初始化共享区就跳过本轮 */
		if (hdr->magic != CMDQU_SHM_MAGIC || hdr->version != CMDQU_SHM_VERSION)
			continue;
		if (hdr->rtos_to_host.entry_size != CMDQU_RING_ENTRY_SZ ||
		    hdr->rtos_to_host.entries == 0U)
			continue;

		/* 采样：目前用自增计数与固定值，便于验证频率与完整性；
		 * 接入真实外设后替换成轮速、光电门、占空比等实测值。 */
		g_cmdqu_report_seq++;
		snap.seq = g_cmdqu_report_seq;
		snap.flags = 0U;
		/* 注意单位：这里要算的是**微秒**，所以是 1e6 而不是 1e3。
		 * 写成 *1000/tick_rate 得到的是毫秒，实板读出来每次只 +1000，
		 * 恰好暴露了这个错误。 */
		snap.rtos_time_us = (unsigned long long)xTaskGetTickCount() * 1000000ULL /
				    (unsigned long long)configTICK_RATE_HZ;
		snap.wheel_counts[0] = (int)g_cmdqu_report_seq;
		snap.wheel_counts[1] = -(int)g_cmdqu_report_seq;
		snap.photogate = 0x1U;
		snap.duty_permille[0] = 500U;
		snap.duty_permille[1] = 0U;

		cmdqu_shm_publish(hdr, CMDQU_MSG_STATE_SNAPSHOT, (const unsigned char *)&snap,
				  sizeof(snap));

		/* UART1 链路测试开着时，每秒额外发一条 kind=6 的链路状态。
		 * 与快照同一个任务发布，因此不会与环 head 的读改写打架。 */
		now_ms = (unsigned long long)xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);
		if (g_cmdqu_uart1_mode != 0U && (now_ms - last_uart_pub_ms) >= 1000ULL) {
			struct cmdqu_uart_link link;

			link.seq = ++g_cmdqu_uart1_link_seq;
			link.mode = g_cmdqu_uart1_mode;
			link.init_sent = g_cmdqu_uart1_sent;
			link.ack_rx = g_cmdqu_uart1_ack;
			link.nack_rx = g_cmdqu_uart1_nack;
			link.rx_bytes = g_cmdqu_uart1_rx_bytes;
			link.frame_err = g_cmdqu_uart1_errs;
			link.rsp_other = g_cmdqu_uart1_rsp_other;
			link.last_ack_us = g_cmdqu_uart1_last_ack_us;
			cmdqu_shm_publish(hdr, CMDQU_MSG_UART_LINK,
					  (const unsigned char *)&link, sizeof(link));
			/* kind=7 是 mode=2（命令自检）的产物，别的档位不发，
			 * 否则会看到一份"没跑过却显示已走完"的自检状态 */
			if (g_cmdqu_uart1_mode == 2U) {
				struct cmdqu_uart_probe probe;

				probe.seq = ++g_cmdqu_uart1_probe_seq;
				probe.mode = g_cmdqu_uart1_mode;
				probe.stage = g_cmdqu_uart1_probe_stage;
				probe.err = g_cmdqu_uart1_probe_err;
				probe.ack_total = g_cmdqu_uart1_ack;
				probe.cfg_ack = g_cmdqu_uart1_cfg_ack;
				probe.speed_ack = g_cmdqu_uart1_speed_ack;
				probe.stop_ack = g_cmdqu_uart1_stop_ack;
				probe.last_sent = g_cmdqu_uart1_last_sent;
				probe.last_reply = g_cmdqu_uart1_last_reply;
				probe.last_echo = g_cmdqu_uart1_last_echo;
				probe.reserved = 0U;
				cmdqu_shm_publish(hdr, CMDQU_MSG_UART_PROBE,
						  (const unsigned char *)&probe,
						  sizeof(probe));
			}
			if (g_cmdqu_uart1_mode >= 3U) {
				struct cmdqu_uart_ctrl ctrl;

				ctrl.seq = ++g_cmdqu_uart1_ctrl_seq;
				ctrl.mode = g_cmdqu_uart1_mode;
				ctrl.period_ms = g_cmdqu_uart1_ctrl_period_ms;
				ctrl.target_lr = g_cmdqu_uart1_target_lr;
				ctrl.ctrl_count = g_cmdqu_uart1_ctrl_count;
				ctrl.ack_total = g_cmdqu_uart1_ack;
				ctrl.wd_stops = g_cmdqu_uart1_wd_stops;
				ctrl.dt_max_ticks = g_cmdqu_uart1_dt_max;
				ctrl.ch_state = g_cmdqu_uart1_ch_state;
				ctrl.rpm_l = g_cmdqu_uart1_rpm_l;
				ctrl.rpm_r = g_cmdqu_uart1_rpm_r;
				ctrl.wd_ms = g_cmdqu_uart1_wd_ms;
				cmdqu_shm_publish(hdr, CMDQU_MSG_UART_CTRL,
						  (const unsigned char *)&ctrl, sizeof(ctrl));

				{
					struct cmdqu_uart_ctrl_hist hist;
					unsigned int div = g_cmdqu_uart1_ctrl_period_ms /
							   CMDQU_UART1_TICK_MS;

					if (div == 0U)
						div = 1U;
					hist.seq = ++g_cmdqu_uart1_hist_seq;
					hist.period_ticks = div;
					hist.count = g_cmdqu_uart1_dt_count;
					hist.min_ticks = (g_cmdqu_uart1_dt_count != 0U)
								 ? g_cmdqu_uart1_dt_min
								 : 0U;
					hist.max_ticks = g_cmdqu_uart1_dt_max;
					hist.sum_ticks = g_cmdqu_uart1_dt_sum;
					hist.hist_lt = g_cmdqu_uart1_hist_lt;
					hist.hist_eq = g_cmdqu_uart1_hist_eq;
					hist.hist_p1 = g_cmdqu_uart1_hist_p1;
					hist.hist_p2 = g_cmdqu_uart1_hist_p2;
					hist.hist_ge = g_cmdqu_uart1_hist_ge;
					hist.wd_stops = g_cmdqu_uart1_wd_stops;
					cmdqu_shm_publish(hdr, CMDQU_MSG_UART_CTRL_HIST,
							  (const unsigned char *)&hist,
							  sizeof(hist));
				}
			}
			last_uart_pub_ms = now_ms;
		}
	}
}
