#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

#include "bus.h"
#include "log.h"

static uint8_t fetch_op(cpu_t *cpu) {
  return cpu_mem_read(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
}

static uint16_t get_address(cpu_t *cpu, cpu_addressing_mode_t mode) {
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
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc);
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc) + cpu->registers.x;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    return bus_mem_read_u16(cpu->bus, cpu->registers.pc) + cpu->registers.y;
  case CPU_ADDRESSING_MODE_INDIRECT: {
    uint16_t addr = bus_mem_read_u16(cpu->bus, cpu->registers.pc);
    if ((addr & 0xFF) == 0xFF) {
      uint16_t low = (uint16_t) bus_mem_read_u8(cpu->bus, addr);
      uint16_t high = (uint16_t) bus_mem_read_u8(cpu->bus, addr & 0xFF00);
      return high << 8 | low;
    }
    return bus_mem_read_u16(cpu->bus, addr);
  }
  case CPU_ADDRESSING_MODE_INDIRECT_X: {
    uint8_t addr = bus_mem_read_u8(cpu->bus, cpu->registers.pc) + cpu->registers.x;
    return bus_mem_read_u16(cpu->bus, addr);
  }
  case CPU_ADDRESSING_MODE_INDIRECT_Y: {
    uint8_t addr = bus_mem_read_u8(cpu->bus, cpu->registers.pc);
    return bus_mem_read_u16(cpu->bus, addr) + cpu->registers.y;
  }
  case CPU_ADDRESSING_MODE_IMPLIED:
    log_error("cannot find address; address should be implied");
    return 0;
  default:
    log_error("cannot find address; %d not implemented", mode);
    return 0;
  }
}

static uint16_t get_addr_mode_byte_length(cpu_addressing_mode_t mode) {
  switch (mode) {
  case CPU_ADDRESSING_MODE_IMPLIED:
  case CPU_ADDRESSING_MODE_ACCUMULATOR:
    return 0;
  case CPU_ADDRESSING_MODE_IMMEDIATE:
  case CPU_ADDRESSING_MODE_RELATIVE:
  case CPU_ADDRESSING_MODE_ZERO_PAGE:
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X:
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y:
  case CPU_ADDRESSING_MODE_INDIRECT:
  case CPU_ADDRESSING_MODE_INDIRECT_X:
  case CPU_ADDRESSING_MODE_INDIRECT_Y:
    return 1;
  case CPU_ADDRESSING_MODE_ABSOLUTE:
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    return 2;
  default:
    log_error("missing address mode %d", mode);
    return 0;
  }
}

uint16_t cpu_read_address(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t address = get_address(cpu, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return address;
}

uint8_t *cpu_mem_addr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return &cpu->registers.a;
  }
  
  uint16_t address = get_address(cpu, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return bus_mem_addr(cpu->bus, address);
}

uint8_t cpu_mem_read(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return cpu->registers.a;
  }
  
  uint16_t address = get_address(cpu, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return bus_mem_read_u8(cpu->bus, address);
}

void cpu_mem_write(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value) {
  uint16_t address = get_address(cpu, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  bus_mem_write_u8(cpu->bus, address, value);
}

void cpu_stack_push_u8(cpu_t *cpu, uint8_t value) {
  if (cpu->registers.s == 0xFF) {
    log_error("stack overflow");
    return;
  }
  
  bus_mem_write_u8(cpu->bus, STACK_START_ADDR + cpu->registers.s, value);
  cpu->registers.s++;
}

uint8_t cpu_stack_pop_u8(cpu_t *cpu) {
  if (cpu->registers.s == 0) {
    log_error("stack underflow");
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

void cpu_load_cartridge(cpu_t *cpu, const cartridge_t *cart) {
  bus_load_cartridge(cpu->bus, cart);
  cpu_reset(cpu);
}

void cpu_reset(cpu_t *cpu) {
  cpu->registers = (cpu_registers_t) { 0 };
  cpu->registers.pc = bus_mem_read_u16(cpu->bus, RESET_VECTOR);
}

void cpu_run(cpu_t *cpu) {
  while (cpu_step(cpu));
}

uint8_t cpu_step(cpu_t *cpu) {
  cpu_trace(cpu);
  uint8_t op = fetch_op(cpu);
  const cpu_instruction_t *instruction = &INSTRUCTIONS[op];

  if (instruction->func) {
    instruction->func(cpu, instruction->addr_mode);
  }

  return op && instruction->valid;
}

void cpu_trace(cpu_t *cpu) {
  uint8_t opcode = bus_mem_read_u8(cpu->bus, cpu->registers.pc);
  const cpu_instruction_t *instruction = &INSTRUCTIONS[opcode];

  if (!instruction->valid) {
    printf(
    #ifdef CNES_TRACE_ADDR_MODE
      "%04X  %02X             (err) instruction not implemented A:%02X X:%02X Y:%02X P:%02X SP:%02X\n",
    #else
      "%04X  %02X             instruction not implemented A:%02X X:%02X Y:%02X P:%02X SP:%02X\n",
    #endif
      cpu->registers.pc,
      opcode,
      cpu->registers.a,
      cpu->registers.x,
      cpu->registers.y,
      cpu->registers.flags,
      cpu->registers.s
    );
    return;
  }

  uint8_t b1 = bus_mem_read_u8(cpu->bus, cpu->registers.pc + 1);
  uint8_t b2 = bus_mem_read_u8(cpu->bus, cpu->registers.pc + 2);
  uint16_t bb = bus_mem_read_u16(cpu->bus, cpu->registers.pc + 1);
  
  printf("%04X  %02X  ", cpu->registers.pc, opcode);

  switch(get_addr_mode_byte_length(instruction->addr_mode)) {
    case 2:
      printf("%02X %02X ", b1, b2);
      break;
    case 1:
      printf("%02X    ", b1);
      break;
    case 0:
      printf("      ");
      break;
    default:
      printf("error: unexpected opcode byte length ");
      break;
  }

  printf("%c%s ", instruction->unofficial ? '*' : ' ', instruction->name);

  #ifdef CNES_TRACE_ADDR_MODE
  switch (instruction->addr_mode) {
  case CPU_ADDRESSING_MODE_IMPLIED:     printf("(imp) "); break;
  case CPU_ADDRESSING_MODE_ACCUMULATOR: printf("(acc) "); break;
  case CPU_ADDRESSING_MODE_IMMEDIATE:   printf("(imm) "); break;
  case CPU_ADDRESSING_MODE_RELATIVE:    printf("(rel) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE:   printf("(zp ) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X: printf("(zpx) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y: printf("(zpy) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE:    printf("(abs) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:  printf("(abx) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:  printf("(aby) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT:    printf("(ind) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT_X:  printf("(izx) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT_Y:  printf("(izy) "); break;
  default:                              printf("(err) "); break;
  }
  #endif

  switch (instruction->addr_mode) {
  case CPU_ADDRESSING_MODE_IMPLIED:
    printf("                            ");
    break;
  case CPU_ADDRESSING_MODE_ACCUMULATOR:
    printf("A                           ");
    break;
  case CPU_ADDRESSING_MODE_IMMEDIATE:
    printf("#$%02X                        ", b1);
    break;
  case CPU_ADDRESSING_MODE_RELATIVE:
    printf("$%04X                       ", cpu->registers.pc + b1);
    break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE: {
    uint8_t addr = bus_mem_read_u8(cpu->bus, b1);
    printf("$%02X = %02X                    ", b1, addr);
    break;
  }
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X: {
    uint8_t addr = bus_mem_read_u8(cpu->bus, b1 + cpu->registers.x);
    printf("$%02X,X @ %02X = %02X             ", b1, b1 + cpu->registers.x, addr);
    break;
  }
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y: {
    uint8_t addr = bus_mem_read_u8(cpu->bus, b1 + cpu->registers.y);
    printf("$%02X,Y @ %02X = %02X             ", b1, b1 + cpu->registers.y, addr);
    break;
  }
  case CPU_ADDRESSING_MODE_ABSOLUTE: {
    printf("$%04X                       ", bb);
    break;
  }
  case CPU_ADDRESSING_MODE_ABSOLUTE_X: {
    uint16_t addr = bb + cpu->registers.x;
    printf("$%04X,X @ %04X = %02X         ", bb, addr, bus_mem_read_u8(cpu->bus, addr));
    break;
  }
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y: {
    uint16_t addr = bb + cpu->registers.y;
    printf("$%04X,Y @ %04X = %02X         ", bb, addr, bus_mem_read_u8(cpu->bus, addr));
    break;
  }
  case CPU_ADDRESSING_MODE_INDIRECT: {
    uint16_t addr = bus_mem_read_u16(cpu->bus, bb);
    printf("($%04X) = %04X              ", bb, addr);
    break;
  }
  case CPU_ADDRESSING_MODE_INDIRECT_X: {
    uint16_t addr = bus_mem_read_u8(cpu->bus, b1) + cpu->registers.x;
    uint16_t ind = bus_mem_read_u16(cpu->bus, addr);
    printf("($%02X,X) @ %02X = %04X = %02X    ", b1, addr, ind, bus_mem_read_u8(cpu->bus, ind));
    break;
  }
  case CPU_ADDRESSING_MODE_INDIRECT_Y: {
    uint16_t addr = bus_mem_read_u8(cpu->bus, b1);
    uint16_t ind = bus_mem_read_u16(cpu->bus, addr) + cpu->registers.y;
    printf("($%02X),Y = %04X @ %04X = %02X  ", b1, addr, ind, bus_mem_read_u8(cpu->bus, ind));
    break;
  }
  default:
    printf("cannot format addr mode     ");
    break;
  }

  printf("A:%02X X:%02X Y:%02X P:%02X SP:%02X",
    cpu->registers.a,
    cpu->registers.x,
    cpu->registers.y,
    cpu->registers.flags,
    cpu->registers.s
  );

  printf("\n");
}
