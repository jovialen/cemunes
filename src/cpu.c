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

void cpu_load_program(cpu_t *cpu, uint8_t *program, size_t size) {
  bus_mem_write(cpu->bus, 0x8000, program, size);
  bus_mem_write_u16(cpu->bus, RESET_VECTOR, 0x8000);
}

void cpu_reset(cpu_t *cpu) {
  cpu->registers = (cpu_registers_t) { 0 };
  cpu->registers.pc = bus_mem_read_u16(cpu->bus, RESET_VECTOR);
}

void cpu_run(cpu_t *cpu) {
  cpu_reset(cpu);
  for (uint8_t op = fetch_op(cpu); op != 0; op = fetch_op(cpu)) {
    const cpu_instruction_t *instruction = &INSTRUCTIONS[op];
    instruction->func(cpu, instruction->addr_mode);
  }
}
