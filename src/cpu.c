#include "cpu.h"

#include <stdio.h>

#include "bus.h"

uint8_t fetch_op(cpu_t *cpu) {
  return bus_mem_read(cpu->bus, cpu->registers.pc++);
}

void cpu_run(cpu_t *cpu) {
  for (uint8_t op = fetch_op(cpu); op != 0; op = fetch_op(cpu)) {
    printf("%u: %x\n", cpu->registers.pc, op);
  }
}
