#include "amber_registers.h"
#include "amber_macros.h"

start:
	mov	r1, #3
	mov 	r2, #4
	add	r3, r1, r2
	b	0x00000000