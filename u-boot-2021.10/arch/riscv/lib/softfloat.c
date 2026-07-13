// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal soft-float stubs for rv64imac (no hardware float in toolchain).
 *
 * The C906 CPU does have an FPU, but U-Boot is compiled with rv64imac/lp64
 * (soft-float ABI).  cvi_mipi.c uses C-level float arithmetic for MIPI DSI
 * timing calculations, which the compiler turns into calls to these soft-float
 * helpers.  Since the toolchain's libgcc was built for rv64imafdc/lp64d, its
 * implementations use a mismatched ABI.
 *
 * We implement the helpers using inline RISC-V FPU assembly so the resulting
 * .o remains lp64 (soft-float ABI) and links cleanly with the rest of u-boot.
 * The assembler is told to accept FP opcodes via:
 *   CFLAGS_softfloat.o += -Wa,-march=rv64imafdc
 * No "f" constraint is used to avoid requiring the F extension in the compiler.
 *
 * All values flow through integer registers (the lp64 convention):
 *   - Input floats arrive as raw bits in a0, a1, ...
 *   - fmv.w.x / fmv.d.x moves them into fa0, fa1
 *   - FP operation executes
 *   - fmv.x.w / fmv.x.d moves the result back to an integer register
 */

/* Move a 32-bit value from int reg to float reg */
#define MV_I2F(ireg, freg)	"fmv.w.x " freg ", " ireg "\n\t"
#define MV_F2I(freg, ireg)	"fmv.x.w " ireg ", " freg "\n\t"
/* Move a 64-bit value from int reg to double reg */
#define MV_I2D(ireg, dreg)	"fmv.d.x " dreg ", " ireg "\n\t"
#define MV_D2I(dreg, ireg)	"fmv.x.d " ireg ", " dreg "\n\t"

float __mulsf3(float a, float b)
{
	float r;
	asm volatile (
		MV_I2F("%1", "fa0")
		MV_I2F("%2", "fa1")
		"fmul.s  fa0, fa0, fa1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

float __addsf3(float a, float b)
{
	float r;
	asm volatile (
		MV_I2F("%1", "fa0")
		MV_I2F("%2", "fa1")
		"fadd.s  fa0, fa0, fa1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

float __subsf3(float a, float b)
{
	float r;
	asm volatile (
		MV_I2F("%1", "fa0")
		MV_I2F("%2", "fa1")
		"fsub.s  fa0, fa0, fa1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

float __floatsisf(int i)
{
	float r;
	asm volatile (
		"fcvt.s.w fa0, %1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(i) : "fa0"
	);
	return r;
}

float __floatunsisf(unsigned int u)
{
	float r;
	asm volatile (
		"fcvt.s.wu fa0, %1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(u) : "fa0"
	);
	return r;
}

unsigned int __fixunssfsi(float f)
{
	unsigned int r;
	asm volatile (
		MV_I2F("%1", "fa0")
		"fcvt.wu.s %0, fa0, rtz\n\t"
		: "=r"(r) : "r"(f) : "fa0"
	);
	return r;
}

double __muldf3(double a, double b)
{
	double r;
	asm volatile (
		MV_I2D("%1", "fa0")
		MV_I2D("%2", "fa1")
		"fmul.d  fa0, fa0, fa1\n\t"
		MV_D2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

double __floatsidf(int i)
{
	double r;
	asm volatile (
		"fcvt.d.w fa0, %1\n\t"
		MV_D2I("fa0", "%0")
		: "=r"(r) : "r"(i) : "fa0"
	);
	return r;
}

float __divsf3(float a, float b)
{
	float r;
	asm volatile (
		MV_I2F("%1", "fa0")
		MV_I2F("%2", "fa1")
		"fdiv.s  fa0, fa0, fa1\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

double __divdf3(double a, double b)
{
	double r;
	asm volatile (
		MV_I2D("%1", "fa0")
		MV_I2D("%2", "fa1")
		"fdiv.d  fa0, fa0, fa1\n\t"
		MV_D2I("fa0", "%0")
		: "=r"(r) : "r"(a), "r"(b) : "fa0", "fa1"
	);
	return r;
}

double __floatunsidf(unsigned int u)
{
	double r;
	asm volatile (
		"fcvt.d.wu fa0, %1\n\t"
		MV_D2I("fa0", "%0")
		: "=r"(r) : "r"(u) : "fa0"
	);
	return r;
}

float __truncdfsf2(double d)
{
	float r;
	asm volatile (
		MV_I2D("%1", "fa0")
		"fcvt.s.d fa0, fa0\n\t"
		MV_F2I("fa0", "%0")
		: "=r"(r) : "r"(d) : "fa0"
	);
	return r;
}
