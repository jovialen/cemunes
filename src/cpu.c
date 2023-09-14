#include "cpu.h"

#include <stdio.h>

#include "bus.h"

static uint8_t cpu_mem_read(cpu_t *cpu) {
  return bus_mem_read(cpu->bus, cpu->registers.pc++);
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
  cpu->registers.a = cpu_mem_read(cpu);
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
  return cpu_mem_read(cpu);
}

const cpu_instruction_t INSTRUCTIONS[0xFF] = {
  [0x00] = { .name = "brk", .func = 0    },
  [0xa9] = { .name = "lda", .func = &lda },
  [0xaa] = { .name = "tax", .func = &tax },
  [0xe8] = { .name = "inx", .func = &inx },
};

void cpu_run(cpu_t *cpu) {
  for (uint8_t op = fetch_op(cpu); op != 0; op = fetch_op(cpu)) {
    const cpu_instruction_t *instruction = &INSTRUCTIONS[op];
    printf("%u (%s): %x\n", cpu->registers.pc, instruction->name, op);
    instruction->func(cpu);
  }
  printf("%u (%s): %x\n", cpu->registers.pc, INSTRUCTIONS[0].name, 0);
}
