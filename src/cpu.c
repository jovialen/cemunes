#include "cpu.h"

#include <stdio.h>

#include "bus.h"

static uint8_t cpu_mem_read_u8(cpu_t *cpu) {
  return bus_mem_read_u8(cpu->bus, cpu->registers.pc++);
}

static void update_negative_zero_registers(cpu_t *cpu, uint8_t a) {
  if (a == 0) {
    cpu->registers.flags |= CPU_STATUS_FLAG_ZERO;
  } else {
    cpu->registers.flags &= ~CPU_STATUS_FLAG_ZERO;
  }

  if (a & CPU_STATUS_FLAG_NEGATIVE) {
    cpu->registers.flags &= ~CPU_STATUS_FLAG_NEGATIVE;
  } else {
    cpu->registers.flags |= CPU_STATUS_FLAG_NEGATIVE;
  }
}

static void lda(cpu_t *cpu) {
  cpu->registers.a = cpu_mem_read_u8(cpu);
  update_negative_zero_registers(cpu, cpu->registers.a);
}

static void tax(cpu_t *cpu) {
  cpu->registers.x = cpu->registers.a;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

static void inx(cpu_t *cpu) {
  cpu->registers.x++;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

static uint8_t fetch_op(cpu_t *cpu) {
  uint8_t op = cpu_mem_read_u8(cpu);
  printf("pc: %x, opcode: %x (%s)\n", cpu->registers.pc - 1, op, INSTRUCTIONS[op].name);
  return op;
}

const cpu_instruction_t INSTRUCTIONS[INSTRUCTION_COUNT] = {
  [0x00] = { .name = "brk", .func = 0    },
  [0xa9] = { .name = "lda", .func = &lda },
  [0xaa] = { .name = "tax", .func = &tax },
  [0xe8] = { .name = "inx", .func = &inx },
};

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
    instruction->func(cpu);
  }
}
