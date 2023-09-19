#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

#define CPU_INSTRUCTION(FUNC, MODE) { .name = #FUNC, .func = &FUNC, .addr_mode = MODE }
#define BRK_INSTRUCTION() { .name = "brk", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }
#define NOP_INSTRUCTION() { .name = "nop", .func = 0, .addr_mode = CPU_ADDRESSING_MODE_IMPLIED }

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

static void branch(cpu_t *cpu, bool condition) {
  uint8_t jump = cpu_mem_read(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
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
	uint8_t m = cpu_mem_read(cpu, mode);
	cpu->registers.a &= m;

	if (m & (1 << 6)) {
		cpu->registers.flags |= CPU_STATUS_FLAG_OVERFLOW;
	} else {
		cpu->registers.flags &= ~CPU_STATUS_FLAG_OVERFLOW;
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
  cpu->registers.pc = cpu_mem_read(cpu, mode);
}

#define JMP_INSTRUCTION(MODE) CPU_INSTRUCTION(bvs, MODE)

static void cld(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_DECIMAL;
}

#define CLD_INSTRUCTION() CPU_INSTRUCTION(cld, CPU_ADDRESSING_MODE_IMPLIED)

static void cli(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_INT_DISABLE;
}

#define CLI_INSTRUCTION() CPU_INSTRUCTION(cli, CPU_ADDRESSING_MODE_IMPLIED)

static void clv(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.flags &= ~CPU_STATUS_FLAG_OVERFLOW;
}

#define CLV_INSTRUCTION() CPU_INSTRUCTION(clv, CPU_ADDRESSING_MODE_IMPLIED)

static void compare(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t a) {
  uint8_t m = cpu_mem_read(cpu, mode);

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

static void cmx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  compare(cpu, mode, cpu->registers.x);
}

#define CMX_INSTRUCTION(MODE) CPU_INSTRUCTION(cmx, MODE)

static void cmy(cpu_t *cpu, cpu_addressing_mode_t mode) {
  compare(cpu, mode, cpu->registers.y);
}

#define CMY_INSTRUCTION(MODE) CPU_INSTRUCTION(cmy, MODE)

static void lda(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.a = cpu_mem_read(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define LDA_INSTRUCTION(MODE) CPU_INSTRUCTION(lda, MODE)

static void ldx(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu_mem_read(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define LDX_INSTRUCTION(MODE) CPU_INSTRUCTION(ldx, MODE)

static void ldy(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.y = cpu_mem_read(cpu, mode);
  update_negative_zero_registers(cpu, cpu->registers.y);
}

#define LDY_INSTRUCTION(MODE) CPU_INSTRUCTION(ldy, MODE)

static void sta(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu_mem_write(cpu, mode, cpu->registers.a);
}

#define STA_INSTRUCTION(MODE) CPU_INSTRUCTION(sta, MODE)

static void tax(cpu_t *cpu, cpu_addressing_mode_t mode) {
  cpu->registers.x = cpu->registers.a;
  update_negative_zero_registers(cpu, cpu->registers.x);
}

#define TAX_INSTRUCTION() CPU_INSTRUCTION(tax, CPU_ADDRESSING_MODE_IMPLIED)

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

#define INY_INSTRUCTION() CPU_INSTRUCTION(inx, CPU_ADDRESSING_MODE_IMPLIED)

static void eor(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint8_t m = cpu_mem_read(cpu, mode);
  cpu->registers.a ^= m;
  update_negative_zero_registers(cpu, cpu->registers.a);
}

#define EOR_INSTRUCTION(MODE) CPU_INSTRUCTION(eor, MODE)

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
  [0x4a] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ACCUMULATOR),
  [0x46] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x56] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x4e] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x5e] = LSR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  
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

  [0xe0] = CMX_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xe4] = CMX_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xec] = CMX_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  
  [0xc0] = CMY_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0xc4] = CMY_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xcc] = CMY_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),

  [0xc6] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xd6] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xce] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xde] = DEC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),

  [0xe6] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0xf6] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0xee] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0xfe] = INC_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),

  [0x49] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_IMMEDIATE),
  [0x45] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE),
  [0x55] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ZERO_PAGE_X),
  [0x4d] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x5d] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_X),
  [0x59] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE_Y),
  [0x41] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_X),
  [0x51] = EOR_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT_Y),

  [0x4c] = JMP_INSTRUCTION(CPU_ADDRESSING_MODE_ABSOLUTE),
  [0x6c] = JMP_INSTRUCTION(CPU_ADDRESSING_MODE_INDIRECT),
  
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
  
  [0xca] = DEX_INSTRUCTION(),
  [0x88] = DEY_INSTRUCTION(),
  [0xe8] = INX_INSTRUCTION(),
  [0xc8] = INY_INSTRUCTION(),
  
  [0xaa] = TAX_INSTRUCTION(),
};
