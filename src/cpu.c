#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

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

static uint8_t *cpu_mem_addr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return &cpu->registers.a;
  }
  
  uint16_t bytes;
  uint16_t address = get_address(cpu, mode, &bytes);
  cpu->registers.pc += bytes;
  return bus_mem_addr(cpu->bus, address);
}

static uint8_t cpu_mem_read(cpu_t *cpu, cpu_addressing_mode_t mode) {
  return *cpu_mem_addr(cpu, mode);
}

static void cpu_mem_write(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value) {
  *cpu_mem_addr(cpu, mode) = value;
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

static void sec(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags |= CPU_STATUS_FLAG_CARRY;
}

#define SEC_INSTRUCTION() CPU_INSTRUCTION(sec, CPU_ADDRESSING_MODE_IMPLIED)

static void clc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_CARRY;
}

#define CLC_INSTRUCTION() CPU_INSTRUCTION(clc, CPU_ADDRESSING_MODE_IMPLIED)

static void adc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t b = cpu_mem_read(cpu, mode) + (cpu->registers.flags & CPU_STATUS_FLAG_CARRY);
  uint8_t c = cpu->registers.a + b;
  
  if (cpu->registers.a > c) {
    cpu->registers.flags |= CPU_STATUS_FLAG_OVERFLOW;
  } else {
    cpu->registers.flags &= ~CPU_STATUS_FLAG_OVERFLOW;
  }

  cpu->registers.a = c;
  update_negative_zero_registers(cpu, c);
}

#define ADC_INSTRUCTION(MODE) CPU_INSTRUCTION(adc, MODE)

static void and(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t m = cpu_mem_read(cpu, mode);
  cpu->registers.a &= m;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define AND_INSTRUCTION(MODE) CPU_INSTRUCTION(and, MODE)

static void asl(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *byte = cpu_mem_addr(cpu, mode);

  if (*byte & (1 << 7)) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  *byte <<= 1;
  
  update_negative_zero_registers(cpu, *byte);
}

static void branch(cpu_t *cpu, bool condition) {
  uint8_t jump = cpu_mem_read(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
  if (condition) {
    cpu->registers.pc += jump;
  }
}

#define ASL_INSTRUCTION(MODE) CPU_INSTRUCTION(asl, MODE)

static void bcc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, !(cpu->registers.flags & CPU_STATUS_FLAG_CARRY));
}

#define BCC_INSTRUCTION() CPU_INSTRUCTION(bcc, CPU_ADDRESSING_MODE_RELATIVE)

static void bcs(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, cpu->registers.flags & CPU_STATUS_FLAG_CARRY);
}

#define BCS_INSTRUCTION() CPU_INSTRUCTION(bcs, CPU_ADDRESSING_MODE_RELATIVE)

static void beq(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, cpu->registers.flags & CPU_STATUS_FLAG_ZERO);
}

#define BEQ_INSTRUCTION() CPU_INSTRUCTION(beq, CPU_ADDRESSING_MODE_RELATIVE)

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

  [0x69] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x65] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x75] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x6d] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x7d] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x79] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x61] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x71] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x29] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x25] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x35] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x2d] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x3d] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x39] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x21] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x31] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x0a] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x06] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x16] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x0e] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x1e] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  
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
  
  [0x18] = CLC_INSTRUCTION(),
  [0x38] = SEC_INSTRUCTION(),
  [0x90] = BCC_INSTRUCTION(),
  [0xb0] = BCS_INSTRUCTION(),
  [0xf0] = BEQ_INSTRUCTION(),
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
