#ifndef __cnes_cpu_h__
#define __cnes_cpu_h__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bus.h"
#include "cartridge.h"

#define INSTRUCTION_COUNT 256
#define STACK_START_ADDR 0x100


typedef enum cpu_addressing_mode_t {
	CPU_ADDRESSING_MODE_IMPLIED = 0,
	CPU_ADDRESSING_MODE_ACCUMULATOR,
	CPU_ADDRESSING_MODE_IMMEDIATE,
	CPU_ADDRESSING_MODE_RELATIVE,
	CPU_ADDRESSING_MODE_ABSOLUTE,
	CPU_ADDRESSING_MODE_ABSOLUTE_X,
	CPU_ADDRESSING_MODE_ABSOLUTE_Y,
	CPU_ADDRESSING_MODE_ZERO_PAGE,
	CPU_ADDRESSING_MODE_ZERO_PAGE_X,
	CPU_ADDRESSING_MODE_ZERO_PAGE_Y,
	CPU_ADDRESSING_MODE_INDIRECT,
	CPU_ADDRESSING_MODE_INDIRECT_X,
	CPU_ADDRESSING_MODE_INDIRECT_Y,
} cpu_addressing_mode_t;

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

typedef void(*cpu_instruction_func_t)(cpu_t *, cpu_addressing_mode_t);

typedef struct cpu_instruction_t {
	bool valid;
	bool unofficial;
	const char *name;
	cpu_instruction_func_t func;
	cpu_addressing_mode_t addr_mode;
} cpu_instruction_t;

extern const cpu_instruction_t INSTRUCTIONS[INSTRUCTION_COUNT];

uint16_t cpu_read_address(cpu_t *cpu, cpu_addressing_mode_t mode);

uint8_t *cpu_mem_addr(cpu_t *cpu, cpu_addressing_mode_t mode);
uint8_t cpu_mem_read_u8(cpu_t *cpu, cpu_addressing_mode_t mode);
void cpu_mem_write_u8(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value);

void cpu_stack_push_u8(cpu_t *cpu, uint8_t value);
uint8_t cpu_stack_pop_u8(cpu_t *cpu);
void cpu_stack_push_u16(cpu_t *cpu, uint16_t value);
uint16_t cpu_stack_pop_u16(cpu_t *cpu);

void cpu_load_cartridge(cpu_t *cpu, const cartridge_t *cart);

void cpu_reset(cpu_t *cpu);
void cpu_run(cpu_t *cpu);
void cpu_run_from(cpu_t *cpu, uint16_t start);
uint8_t cpu_step(cpu_t *cpu);

void cpu_trace(cpu_t *cpu);


#endif /* __cnes_cpu_h__ */
