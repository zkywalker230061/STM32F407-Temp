#include "bsp/fpu.h"

void bsp_FPU_Enable(void) {
#if defined(__ARM_FP) && (__ARM_FP != 0)
	/* Coprocessor Access Control Register (CPACR) */
	/* Set CP10 and CP11 Full Access (bits 20..23 = 0b11, 0b11) */
	SCB->CPACR |= ((0b11UL << 20) | (0b11UL << 22));
	__DSB();
	__ISB();
#endif
}
