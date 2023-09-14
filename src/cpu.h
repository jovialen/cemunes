#ifndef __cnes_cpu_h__
#define __cnes_cpu_h__

#include <stddef.h>
#include <stdint.h>

#include "bus.h"

#define INSTRUCTION_COUNT 256


typedef enum cpu_status_flag_t {
	CPU_STATUS_FLAG_CARRY       = (1 << 0),
	CPU_STATUS_FLAG_ZERO        = (1 << 1),
	CPU_STATUS_FLAG_INT_DISABLE = (1 << 2),
	CPU_STATUS_FLAG_DECIMAL     = (1 << 3),
	CPU_STATUS_FLAG_OVERFLOW    = (1 << 6),
	CPU_STATUS_FLAG_NEGATIVE    = (1 << 7),
} cpu_status_flag_t;

typedef uint8_t cpu_status_flags_t;

typedef struct cpu_registers_t {
	uint8_t x, y;
	uint8_t s;
	uint8_t a;
	cpu_status_flags_t flags;
	uint16_t pc;
} cpu_registers_t;

typedef struct cpu_t {
	cpu_registers_t registers;
	bus_t *bus;
} cpu_t;

typedef void(*cpu_instruction_func_t)(cpu_t *);

typedef struct cpu_instruction_t {
	const char *name;
	cpu_instruction_func_t func;
} cpu_instruction_t;

extern const cpu_instruction_t INSTRUCTIONS[INSTRUCTION_COUNT];

void cpu_load_program(cpu_t *cpu, uint8_t *program, size_t size);

void cpu_reset(cpu_t *cpu);
void cpu_run(cpu_t *cpu);


#endif /* __cnes_cpu_h__ */
