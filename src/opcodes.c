#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

#include "log.h"

#define CPU_INSTRUCTION(FUNC, MODE)            { .valid = true, .unofficial = false, .name = #FUNC, .func = &FUNC, .addr_mode = MODE }
#define UNOFFICIAL_CPU_INSTRUCTION(FUNC, MODE) { .valid = true, .unofficial = true,  .name = #FUNC, .func = &FUNC, .addr_mode = MODE }

#define BRK_INSTRUCTION() { .valid = true, .unofficial = false, .name = "brk", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }
#define NOP_INSTRUCTION() { .valid = true, .unofficial = false, .name = "nop", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }
#define UNOFFICIAL_NOP_INSTRUCTION() { .valid = true, .unofficial = true, .name = "nop", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }

static void update_negative_zero_registers(cpu_t *cpu, uint8_t a) {
  if (a == 0) {
    sez(cpu);
  } else {
    clz(cpu);
  }

  if (a & CPU_STATUS_FLAG_NEGATIVE) {
    sen(cpu);
  } else {
    cln(cpu);
  }
}

static void sez(cpu_t *cpu) {
  cpu->registers.flags |= CPU_STATUS_FLAG_ZERO;
  log_trace("setting zero flag");
}

static void sev(cpu_t *cpu, cpu_addressing_mode_t mode) {
    cpu->registers.flags |= CPU_STATUS_FLAG_OVERFLOW;
    log_trace("setting overflow flag");
}

static void clz(cpu_t *cpu) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_ZERO;
  log_trace("clearing zero flag");
}

static void sen(cpu_t *cpu) {
  cpu->registers.flags |= CPU_STATUS_FLAG_NEGATIVE;
  log_trace("setting negative flag");
}

static void cln(cpu_t *cpu) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_NEGATIVE;
  log_trace("clearing negative flag");
}

static void sec(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags |= CPU_STATUS_FLAG_CARRY;
  log_trace("setting carry flag");
}

#define SEC_INSTRUCTION() CPU_INSTRUCTION(sec, CPU_ADDRESSING_MODE_IMPLIED)

static void sed(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags |= CPU_STATUS_FLAG_DECIMAL;
  log_trace("setting decimal flag");
}

#define SED_INSTRUCTION() CPU_INSTRUCTION(sed, CPU_ADDRESSING_MODE_IMPLIED)

static void sei(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags |= CPU_STATUS_FLAG_INT_DISABLE;
  log_trace("setting interupt disable flag");
}

#define SEI_INSTRUCTION() CPU_INSTRUCTION(sei, CPU_ADDRESSING_MODE_IMPLIED)

static void clc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_CARRY;
  log_trace("clearing carry flag");
}

#define CLC_INSTRUCTION() CPU_INSTRUCTION(clc, CPU_ADDRESSING_MODE_IMPLIED)

static void adc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t b = cpu_mem_read_u8(cpu, mode) + (cpu->registers.flags & CPU_STATUS_FLAG_CARRY);
  uint8_t new_a = cpu->registers.a + b;
  
  if (cpu->registers.a > new_a) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  if ((b ^ cpu->registers.a) & (b ^ new_a) & 0x80) {
    sev(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clv(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  cpu->registers.a = new_a;
  update_negative_zero_registers(cpu, new_a);
}

#define ADC_INSTRUCTION(MODE) CPU_INSTRUCTION(adc, MODE)

static void sbc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t b = cpu_mem_read_u8(cpu, mode) + ~(cpu->registers.flags & CPU_STATUS_FLAG_CARRY);

  if (cpu->registers.a < b) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }
  
  uint8_t new_a = cpu->registers.a - b;

  if ((b ^ cpu->registers.a) & (b ^ new_a) & 0x80) {
    sev(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clv(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  cpu->registers.a -= new_a;
  update_negative_zero_registers(cpu, new_a);
}

#define SBC_INSTRUCTION(MODE) CPU_INSTRUCTION(sbc, MODE)

static void and(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t m = cpu_mem_read_u8(cpu, mode);
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

#define ASL_INSTRUCTION(MODE) CPU_INSTRUCTION(asl, MODE)

static void lsr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *byte = cpu_mem_addr(cpu, mode);

  if (*byte & 0b1) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  *byte >>= 1;

  update_negative_zero_registers(cpu, *byte);
}

#define LSR_INSTRUCTION(MODE) CPU_INSTRUCTION(lsr, MODE)

static void rol(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *byte = cpu_mem_addr(cpu, mode);
  uint8_t carry = cpu->registers.flags & CPU_STATUS_FLAG_CARRY ? 0b1 : 0;

  if (*byte & (0b1 << 7)) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  *byte = (*byte << 1) | carry;

  update_negative_zero_registers(cpu, *byte);
}

#define ROL_INSTRUCTION(MODE) CPU_INSTRUCTION(rol, MODE)

static void ror(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *byte = cpu_mem_addr(cpu, mode);
  uint8_t carry = cpu->registers.flags & CPU_STATUS_FLAG_CARRY ? 0b1 << 7 : 0;

  if (*byte & 0b1) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  *byte = (*byte >> 1) | carry;

  update_negative_zero_registers(cpu, *byte);
}

#define ROR_INSTRUCTION(MODE) CPU_INSTRUCTION(ror, MODE)

static void branch(cpu_t *cpu, bool condition) {
  uint8_t jump = cpu_mem_read_u8(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
  if (condition) {
    cpu->registers.pc += jump;
  }
}

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

static void bit(cpu_t *cpu, cpu_addressing_mode_t mode) {
	uint8_t m = cpu_mem_read_u8(cpu, mode);
	cpu->registers.a &= m;

	if (m & (1 << 6)) {
        sev(cpu, CPU_ADDRESSING_MODE_IMPLIED);
	} else {
        clv(cpu, CPU_ADDRESSING_MODE_IMPLIED);
	}
	
	update_negative_zero_registers(cpu, cpu->registers.a);
}

#define BIT_INSTRUCTION(MODE) CPU_INSTRUCTION(bit, MODE)

static void bmi(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, cpu->registers.flags & CPU_STATUS_FLAG_NEGATIVE);
}

#define BMI_INSTRUCTION() CPU_INSTRUCTION(bmi, CPU_ADDRESSING_MODE_RELATIVE)

static void bne(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, !(cpu->registers.flags & CPU_STATUS_FLAG_ZERO));
}

#define BNE_INSTRUCTION() CPU_INSTRUCTION(bne, CPU_ADDRESSING_MODE_RELATIVE)

static void bpl(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, !(cpu->registers.flags & CPU_STATUS_FLAG_NEGATIVE));
}

#define BPL_INSTRUCTION() CPU_INSTRUCTION(bpl, CPU_ADDRESSING_MODE_RELATIVE)

static void bvp(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, !(cpu->registers.flags & CPU_STATUS_FLAG_OVERFLOW));
}

#define BVP_INSTRUCTION() CPU_INSTRUCTION(bvp, CPU_ADDRESSING_MODE_RELATIVE)

static void bvs(cpu_t *cpu, cpu_addressing_mode_t mode) {
  branch(cpu, cpu->registers.flags & CPU_STATUS_FLAG_OVERFLOW);
}

#define BVS_INSTRUCTION() CPU_INSTRUCTION(bvs, CPU_ADDRESSING_MODE_RELATIVE)

static void jmp(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.pc = cpu_read_address(cpu, mode);
}

#define JMP_INSTRUCTION(MODE) CPU_INSTRUCTION(jmp, MODE)

static void jsr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t addr = cpu_read_address(cpu, mode);
  cpu_stack_push_u16(cpu, cpu->registers.pc - 1);
  cpu->registers.pc = addr;
}

#define JSR_INSTRUCTION() CPU_INSTRUCTION(jsr, CPU_ADDRESSING_MODE_ABSOLUTE)

static void rts(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t addr = cpu_stack_pop_u16(cpu) + 1;
  cpu->registers.pc = addr;
}

#define RTS_INSTRUCTION() CPU_INSTRUCTION(rts, CPU_ADDRESSING_MODE_IMPLIED)

static void cld(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_DECIMAL;
  log_trace("clearing decimal flag");
}

#define CLD_INSTRUCTION() CPU_INSTRUCTION(cld, CPU_ADDRESSING_MODE_IMPLIED)

static void cli(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_INT_DISABLE;
  log_trace("clearing interupt disable flag")
}

#define CLI_INSTRUCTION() CPU_INSTRUCTION(cli, CPU_ADDRESSING_MODE_IMPLIED)

static void clv(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_OVERFLOW;
  log_trace("clearing overflow flag");
}

#define CLV_INSTRUCTION() CPU_INSTRUCTION(clv, CPU_ADDRESSING_MODE_IMPLIED)

static void compare(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t a) {
  uint8_t m = cpu_mem_read_u8(cpu, mode);

  if (a >= m) {
    sec(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  } else {
    clc(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  }

  update_negative_zero_registers(cpu, a - m);
}

static void cmp(cpu_t *cpu, cpu_addressing_mode_t mode) {
  compare(cpu, mode, cpu->registers.a);
}

#define CMP_INSTRUCTION(MODE) CPU_INSTRUCTION(cmp, MODE)

static void cpx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  compare(cpu, mode, cpu->registers.x);
}

#define CPX_INSTRUCTION(MODE) CPU_INSTRUCTION(cpx, MODE)

static void cpy(cpu_t *cpu, cpu_addressing_mode_t mode) {
  compare(cpu, mode, cpu->registers.y);
}

#define CPY_INSTRUCTION(MODE) CPU_INSTRUCTION(cpy, MODE)

static void lda(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu_mem_read_u8(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define LDA_INSTRUCTION(MODE) CPU_INSTRUCTION(lda, MODE)

static void ldx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu_mem_read_u8(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define LDX_INSTRUCTION(MODE) CPU_INSTRUCTION(ldx, MODE)

static void ldy(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.y = cpu_mem_read_u8(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.y);
}

#define LDY_INSTRUCTION(MODE) CPU_INSTRUCTION(ldy, MODE)

static void sta(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_mem_write_u8(cpu, mode, cpu->registers.a);
}

#define STA_INSTRUCTION(MODE) CPU_INSTRUCTION(sta, MODE)

static void stx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_mem_write_u8(cpu, mode, cpu->registers.x);
}

#define STX_INSTRUCTION(MODE) CPU_INSTRUCTION(stx, MODE)

static void sty(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_mem_write_u8(cpu, mode, cpu->registers.y);
}

#define STY_INSTRUCTION(MODE) CPU_INSTRUCTION(sty, MODE)

static void tax(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu->registers.a;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define TAX_INSTRUCTION() CPU_INSTRUCTION(tax, CPU_ADDRESSING_MODE_IMPLIED)

static void tay(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.y = cpu->registers.a;
  update_negative_zero_registers(cpu, cpu->registers.y);
}

#define TAY_INSTRUCTION() CPU_INSTRUCTION(tay, CPU_ADDRESSING_MODE_IMPLIED)

static void txa(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu->registers.x;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define TXA_INSTRUCTION() CPU_INSTRUCTION(txa, CPU_ADDRESSING_MODE_IMPLIED)

static void tya(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu->registers.y;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define TYA_INSTRUCTION() CPU_INSTRUCTION(tya, CPU_ADDRESSING_MODE_IMPLIED)

static void tsx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu->registers.s;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define TSX_INSTRUCTION() CPU_INSTRUCTION(tsx, CPU_ADDRESSING_MODE_IMPLIED)

static void txs(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.s = cpu->registers.x;
  update_negative_zero_registers(cpu, cpu->registers.s);
}

#define TXS_INSTRUCTION() CPU_INSTRUCTION(txs, CPU_ADDRESSING_MODE_IMPLIED)

static void dec(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *addr = cpu_mem_addr(cpu, mode);
  (*addr)--;
  update_negative_zero_registers(cpu, *addr);
}

#define DEC_INSTRUCTION(MODE) CPU_INSTRUCTION(dec, MODE)

static void dex(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x--;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define DEX_INSTRUCTION() CPU_INSTRUCTION(dex, CPU_ADDRESSING_MODE_IMPLIED)

static void dey(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.y--;
  update_negative_zero_registers(cpu, cpu->registers.y);
}

#define DEY_INSTRUCTION() CPU_INSTRUCTION(dey, CPU_ADDRESSING_MODE_IMPLIED)

static void inc(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t *addr = cpu_mem_addr(cpu, mode);
  (*addr)++;
  update_negative_zero_registers(cpu, *addr);
}

#define INC_INSTRUCTION(MODE) CPU_INSTRUCTION(inc, MODE)

static void inx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x++;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define INX_INSTRUCTION() CPU_INSTRUCTION(inx, CPU_ADDRESSING_MODE_IMPLIED)

static void iny(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.y++;
  update_negative_zero_registers(cpu, cpu->registers.y);
}

#define INY_INSTRUCTION() CPU_INSTRUCTION(iny, CPU_ADDRESSING_MODE_IMPLIED)

static void eor(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t m = cpu_mem_read_u8(cpu, mode);
  cpu->registers.a ^= m;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define EOR_INSTRUCTION(MODE) CPU_INSTRUCTION(eor, MODE)

static void ora(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t m = cpu_mem_read_u8(cpu, mode);
  cpu->registers.a |= m;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define ORA_INSTRUCTION(MODE) CPU_INSTRUCTION(ora, MODE)

static void pha(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_stack_push_u8(cpu, cpu->registers.a);
}

#define PHA_INSTRUCTION() CPU_INSTRUCTION(pha, CPU_ADDRESSING_MODE_IMPLIED)

static void php(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_stack_push_u8(cpu, cpu->registers.flags);
}

#define PHP_INSTRUCTION() CPU_INSTRUCTION(php, CPU_ADDRESSING_MODE_IMPLIED)

static void pla(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu_stack_pop_u8(cpu);
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define PLA_INSTRUCTION() CPU_INSTRUCTION(pla, CPU_ADDRESSING_MODE_IMPLIED)

static void plp(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags = cpu_stack_pop_u8(cpu);
}

#define PLP_INSTRUCTION() CPU_INSTRUCTION(plp, CPU_ADDRESSING_MODE_IMPLIED)

static void rti(cpu_t *cpu, cpu_addressing_mode_t mode) {
  plp(cpu, CPU_ADDRESSING_MODE_IMPLIED);
  cpu->registers.pc = cpu_stack_pop_u16(cpu);
}

#define RTI_INSTRUCTION() CPU_INSTRUCTION(rti, CPU_ADDRESSING_MODE_IMPLIED)

const cpu_instruction_t INSTRUCTIONS[INSTRUCTION_COUNT] = {
  [0x00] = BRK_INSTRUCTION(),
  
  [0xea] = NOP_INSTRUCTION(),
  [0x80] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x82] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xc2] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xe2] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x04] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x44] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x64] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x89] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x0c] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x14] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x34] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x54] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x74] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xd4] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xf4] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x1a] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x3a] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x5a] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x7a] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xda] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xfa] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x1c] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x3c] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x5c] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0x7c] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xdc] = UNOFFICIAL_NOP_INSTRUCTION(),
  [0xfc] = UNOFFICIAL_NOP_INSTRUCTION(),

  [0x69] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x65] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x75] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x6d] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x7d] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x79] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x61] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x71] = ADC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),
  [0xe9] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xe5] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xf5] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xed] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xfd] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0xf9] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0xe1] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0xf1] = SBC_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x0a] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x06] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x16] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x0e] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x1e] = ASL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x4a] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x46] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x56] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x4e] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x5e] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x2a] = ROL_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x26] = ROL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x36] = ROL_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x2e] = ROL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x3e] = ROL_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x6a] = ROR_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x66] = ROR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x76] = ROR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x6e] = ROR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x7e] = ROR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  
  [0xa9] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xa5] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xb5] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xa1] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0xb1] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),
  [0xad] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xbd] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0xb9] = LDA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0xa2] = LDX_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xa6] = LDX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xb6] = LDX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xae] = LDX_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xbe] = LDX_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0xa0] = LDY_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xa4] = LDY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xb4] = LDY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xac] = LDY_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xbc] = LDY_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  
  [0x85] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x95] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x8d] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x9d] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x99] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x81] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x91] = STA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),
  [0x86] = STX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x96] = STX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x8e] = STX_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x84] = STY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x94] = STY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x8c] = STY_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  
  [0x24] = BIT_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x2c] = BIT_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),

  [0xc9] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xc5] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xd5] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xcd] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xdd] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0xd9] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0xc1] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0xd1] = CMP_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0xe0] = CPX_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xe4] = CPX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xec] = CPX_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  
  [0xc0] = CPY_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xc4] = CPY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xcc] = CPY_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),

  [0xc6] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xd6] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xce] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xde] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),

  [0xe6] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xf6] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xee] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xfe] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),

  [0x29] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x25] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x35] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x2d] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x3d] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x39] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x21] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x31] = AND_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x49] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x45] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x55] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x4d] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x5d] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x59] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x41] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x51] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x09] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x05] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x15] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x0d] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x1d] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x19] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x01] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x11] = ORA_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x4c] = JMP_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x6c] = JMP_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT),
  [0x20] = JSR_INSTRUCTION(),
  [0x60] = RTS_INSTRUCTION(),
  
  [0x90] = BCC_INSTRUCTION(),
  [0xb0] = BCS_INSTRUCTION(),
  [0xf0] = BEQ_INSTRUCTION(),
  [0x30] = BMI_INSTRUCTION(),
  [0xd0] = BNE_INSTRUCTION(),
  [0x10] = BPL_INSTRUCTION(),
  [0x50] = BVP_INSTRUCTION(),
  [0x70] = BVS_INSTRUCTION(),
  
  [0x18] = CLC_INSTRUCTION(),
  [0xd8] = CLD_INSTRUCTION(),
  [0x58] = CLI_INSTRUCTION(),
  [0xb8] = CLV_INSTRUCTION(),
  
  [0x38] = SEC_INSTRUCTION(),
  [0xf8] = SED_INSTRUCTION(),
  [0x78] = SEI_INSTRUCTION(),
  
  [0xca] = DEX_INSTRUCTION(),
  [0x88] = DEY_INSTRUCTION(),
  [0xe8] = INX_INSTRUCTION(),
  [0xc8] = INY_INSTRUCTION(),
  
  [0xaa] = TAX_INSTRUCTION(),
  [0xa8] = TAY_INSTRUCTION(),
  [0x8a] = TXA_INSTRUCTION(),
  [0x98] = TYA_INSTRUCTION(),
  [0xba] = TSX_INSTRUCTION(),
  [0x9a] = TXS_INSTRUCTION(),

  [0x48] = PHA_INSTRUCTION(),
  [0x08] = PHP_INSTRUCTION(),
  [0x68] = PLA_INSTRUCTION(),
  [0x28] = PLP_INSTRUCTION(),

  [0x40] = RTI_INSTRUCTION(),
};
