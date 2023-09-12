#ifndef __cnes_cpu_h__
#define __cnes_cpu_h__

#include <stdint.h>

#include "bus.h"


typedef enum cpu_status_flag_t {
	CPU_STATUS_FLAG_CARRY = 0,
	CPU_STATUS_FLAG_ZERO,
	CPU_STATUS_FLAG_INT_DISABLE,
	CPU_STATUS_FLAG_DECIMAL,
	CPU_STATUS_FLAG_OVERFLOW = 6,
	CPU_STATUS_FLAG_NEGATIVE,
} cpu_status_flag_t;

typedef uint8_t cpu_status_flags_t;

typedef struct cpu_registers_t {
	uint8_t x, y;
	uint8_t s;
	cpu_status_flags_t flags;
	uint16_t pc;
} cpu_registers_t;

typedef struct cpu_t {
	cpu_registers_t registers;
	bus_t *bus;
} cpu_t;

void cpu_run(cpu_t *cpu);


#endif /* __cnes_cpu_h__ */
