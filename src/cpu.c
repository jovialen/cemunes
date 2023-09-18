#include "cpu.h"

#include <stdio.h>

#include "bus.h"

#define CPU_INSTRUCTION(FUNC, MODE) { .name = #FUNC, .func = &FUNC, .addr_mode = MODE }
#define BRK_INSTRUCTION() { .name = "brk", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }
#define NOP_INSTRUCTION() { .name = "nop", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }

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

static uint8_t cpu_mem_read(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t bytes = 0;
  uint16_t address = get_address(cpu, mode, &bytes);
  cpu->registers.pc += bytes;
  return bus_mem_read_u8(cpu->bus, address);
}

static void cpu_mem_write(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value) {
  uint16_t address = get_address(cpu, mode, NULL);
  bus_mem_write_u8(cpu->bus, address, value);
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

static void lda(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu_mem_read(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define LDA_INSTRUCTION(MODE) CPU_INSTRUCTION(lda, MODE)

static void sta(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_mem_write(cpu, mode, cpu->registers.a);
}

#define STA_INSTRUCTION(MODE) CPU_INSTRUCTION(sta, MODE)

static void tax(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu->registers.a;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define TAX_INSTRUCTION() CPU_INSTRUCTION(tax, CPU_ADDRESSING_MODE_IMPLIED)

static void inx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x++;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define INX_INSTRUCTION() CPU_INSTRUCTION(inx, CPU_ADDRESSING_MODE_IMPLIED)

static uint8_t fetch_op(cpu_t *cpu) {
  uint8_t op = cpu_mem_read(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
  printf("pc: %x, opcode: %x (%s)\n", cpu->registers.pc, op, INSTRUCTIONS[op].name);
  return op;
}

const cpu_instruction_t INSTRUCTIONS[INSTRUCTION_COUNT] = {
  [0x00] = BRK_INSTRUCTION(),
  
  [0xa9] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xa5] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xb5] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xa1] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0xb1] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),
  [0xad] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xbd] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0xb9] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  
  [0x85] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x95] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x8d] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x9d] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x99] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x81] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x91] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),
  
  [0xaa] = TAX_INSTRUCTION(),
  [0xe8] = INX_INSTRUCTION(),
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
    instruction->func(cpu, instruction->addr_mode);
  }
}
