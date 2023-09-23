#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

#include "bus.h"

static uint8_t fetch_op(cpu_t *cpu) {
  uint8_t op = cpu_mem_read(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
  printf("pc: %x, opcode: %x (%s)\n", cpu->registers.pc, op, INSTRUCTIONS[op].name);
  return op;
}

static uint16_t get_address(cpu_t *cpu, cpu_addressing_mode_t mode, uint16_t *bytes) {
  *bytes = 1;
  switch (mode) {
  case CPU_ADDRESSING_MODE_IMMEDIATE:
    return cpu->registers.pc;
  case CPU_ADDRESSING_MODE_ZERO_PAGE:
    return (uint16_t) bus_mem_read_u8(cpu->bus, cpu->registers.pc);
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X:
    return (uint16_t) bus_mem_read_u8(cpu->bus, cpu->registers.pc) + cpu->registers.x;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y:
    return (uint16_t) bus_mem_read_u8(cpu->bus, cpu->registers.pc) + cpu->registers.y;
  case CPU_ADDRESSING_MODE_ABSOLUTE:
    *bytes = 2;
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc);
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
    *bytes = 2;
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc) + cpu->registers.x;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    *bytes = 2;
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc) + cpu->registers.y;
  case CPU_ADDRESSING_MODE_INDIRECT:
    *bytes = 2;
    uint16_t addr = bus_mem_read_u16(cpu->bus, cpu->registers.pc);
    if ((addr & 0xFF) == 0xFF) {
      uint16_t low = (uint16_t) bus_mem_read_u8(cpu->bus, addr);
      uint16_t high = (uint16_t) bus_mem_read_u8(cpu->bus, addr & 0xFF00);
      return high << 8 | low;
    }
    return bus_mem_read_u16(cpu->bus, addr);
  case CPU_ADDRESSING_MODE_INDIRECT_X: {
    uint8_t addr = bus_mem_read_u16(cpu->bus, cpu->registers.pc) + cpu->registers.x;
    return bus_mem_read_u16(cpu->bus, addr);
  }
  case CPU_ADDRESSING_MODE_INDIRECT_Y: {
    uint8_t addr = bus_mem_read_u16(cpu->bus, cpu->registers.pc);
    return bus_mem_read_u16(cpu->bus, addr) + cpu->registers.y;
  }
  case CPU_ADDRESSING_MODE_IMPLIED:
    printf("error: cannot find address; address should be implied");
    *bytes = 0;
    return 0;
  default:
    printf("error: cannot find address; %d not implemented", mode);
    *bytes = 0;
    return 0;
  }
}

uint16_t cpu_read_address(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t bytes;
  uint16_t address = get_address(cpu, mode, &bytes);
  cpu->registers.pc += bytes;
  return address;
}

uint8_t *cpu_mem_addr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return &cpu->registers.a;
  }
  
  uint16_t bytes;
  uint16_t address = get_address(cpu, mode, &bytes);
  cpu->registers.pc += bytes;
  return bus_mem_addr(cpu->bus, address);
}

uint8_t cpu_mem_read(cpu_t *cpu, cpu_addressing_mode_t mode) {
  return *cpu_mem_addr(cpu, mode);
}

void cpu_mem_write(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value) {
  *cpu_mem_addr(cpu, mode) = value;
}

void cpu_stack_push_u8(cpu_t *cpu, uint8_t value) {
  if (cpu->registers.s == 0xFF) {
    printf("error: stack overflow");
    return;
  }
  
  bus_mem_write_u8(cpu->bus, STACK_START_ADDR + cpu->registers.s, value);
  cpu->registers.s++;
}

uint8_t cpu_stack_pop_u8(cpu_t *cpu) {
  if (cpu->registers.s == 0) {
    printf("error: stack underflow");
    return 0;
  }
  
  cpu->registers.s--;
  return bus_mem_read_u8(cpu->bus, STACK_START_ADDR + cpu->registers.s);
}

void cpu_stack_push_u16(cpu_t *cpu, uint16_t value) {
	uint8_t low = value & 0xFF;
	uint8_t high = (value >> 8) & 0xFF;
  cpu_stack_push_u8(cpu, high);
  cpu_stack_push_u8(cpu, low);
}

uint16_t cpu_stack_pop_u16(cpu_t *cpu) {
  uint16_t low  = (uint16_t) cpu_stack_pop_u8(cpu);
  uint16_t high = (uint16_t) cpu_stack_pop_u8(cpu);
  return (high << 8) | low;
}

void cpu_load_program(cpu_t *cpu, const uint8_t *program, size_t size) {
  bus_mem_write(cpu->bus, PROGRAM_START_ADDR, program, size);
  bus_mem_write_u16(cpu->bus, RESET_VECTOR, PROGRAM_START_ADDR);
}

void cpu_reset(cpu_t *cpu) {
  cpu->registers = (cpu_registers_t) { 0 };
  cpu->registers.pc = bus_mem_read_u16(cpu->bus, RESET_VECTOR);
}

void cpu_run(cpu_t *cpu) {
  cpu_reset(cpu);
  while (cpu_step(cpu));
}

uint8_t cpu_step(cpu_t *cpu) {
  uint8_t op = fetch_op(cpu);
  const cpu_instruction_t *instruction = &INSTRUCTIONS[op];

  if (instruction->func) {
    instruction->func(cpu, instruction->addr_mode);
  }

  return op;
}
