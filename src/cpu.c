#include "cpu.h"

#include <stdio.h>
#include <stdbool.h>

#include "bus.h"
#include "log.h"

static uint8_t fetch_op(cpu_t *cpu) {
  return cpu_mem_read_u8(cpu, CPU_ADDRESSING_MODE_IMMEDIATE);
}

static uint16_t get_address(cpu_t *cpu, uint16_t base, cpu_addressing_mode_t mode) {
  switch (mode) {
  case CPU_ADDRESSING_MODE_IMMEDIATE:
    return base;
  case CPU_ADDRESSING_MODE_ZERO_PAGE:
    return (uint16_t) bus_mem_read_u8(cpu->bus, base);
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X:
    return (uint16_t) (bus_mem_read_u8(cpu->bus, base) + cpu->registers.x) % 0x100;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y:
    return (uint16_t) (bus_mem_read_u8(cpu->bus, base) + cpu->registers.y) % 0x100;
  case CPU_ADDRESSING_MODE_ABSOLUTE:
  case CPU_ADDRESSING_MODE_ABSOLUTE_ADDR:
    return bus_mem_read_u16(cpu->bus, base);
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
    return bus_mem_read_u16(cpu->bus, base) + cpu->registers.x;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    return bus_mem_read_u16(cpu->bus, base) + cpu->registers.y;
  case CPU_ADDRESSING_MODE_INDIRECT: {
    uint16_t addr = bus_mem_read_u16(cpu->bus, base);
    if ((addr & 0xFF) == 0xFF) {
      uint16_t low = (uint16_t) bus_mem_read_u8(cpu->bus, addr);
      uint16_t high = (uint16_t) bus_mem_read_u8(cpu->bus, addr & 0xFF00);
      return uint16_from_bytes(high, low);
    }
    return bus_mem_read_u16(cpu->bus, addr);
  }
  case CPU_ADDRESSING_MODE_INDIRECT_X: {
    uint8_t addr = base + cpu->registers.x;
    return bus_mem_read_u16_zero_page(cpu->bus, addr);
  }
  case CPU_ADDRESSING_MODE_INDIRECT_Y: {
    uint16_t addr = bus_mem_read_u16_zero_page(cpu->bus, base);
    return addr + cpu->registers.y;
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
  case CPU_ADDRESSING_MODE_ABSOLUTE_ADDR:
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    return 2;
  default:
    log_error("missing address mode %d", mode);
    return 0;
  }
}

uint16_t cpu_read_address(cpu_t *cpu, cpu_addressing_mode_t mode) {
  uint16_t address = get_address(cpu, cpu->registers.pc, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return address;
}

uint8_t *cpu_mem_addr(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return &cpu->registers.a;
  }

  uint16_t addr = get_address(cpu, cpu->registers.pc, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return bus_mem_addr(cpu->bus, addr);
}

uint8_t cpu_mem_read_u8(cpu_t *cpu, cpu_addressing_mode_t mode) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    return cpu->registers.a;
  }
  
  uint16_t address = get_address(cpu, cpu->registers.pc, mode);
  cpu->registers.pc += get_addr_mode_byte_length(mode);
  return bus_mem_read_u8(cpu->bus, address);
}

void cpu_mem_write_u8(cpu_t *cpu, cpu_addressing_mode_t mode, uint8_t value) {
  if (mode == CPU_ADDRESSING_MODE_ACCUMULATOR) {
    cpu->registers.a = value;
    return;
  }

  uint16_t address = get_address(cpu, cpu->registers.pc, mode);
  log_trace("writing %02X to $%04X", value, address);

  cpu->registers.pc += get_addr_mode_byte_length(mode);
  bus_mem_write_u8(cpu->bus, address, value);
}

void cpu_stack_push_u8(cpu_t *cpu, uint8_t value) {
  if (cpu->registers.s == 0x0) {
    log_error("stack overflow");
    return;
  }
  
  log_trace("pushing %02X to stack at $%02X", value, cpu->registers.s);
  bus_mem_write_u8(cpu->bus, STACK_START_ADDR + cpu->registers.s, value);
  cpu->registers.s--;
}

uint8_t cpu_stack_pop_u8(cpu_t *cpu) {
  if (cpu->registers.s == 0xFF) {
    log_error("stack underflow");
    return 0;
  }
  
  cpu->registers.s++;
  uint8_t value = bus_mem_read_u8(cpu->bus, STACK_START_ADDR + cpu->registers.s);

  log_trace("popping %02X from stack at $%02X", value, cpu->registers.s);
  return value;
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
  cpu->registers.s = 0xfd;
  cpu->registers.flags = CPU_STATUS_FLAG_INT_DISABLE | CPU_STATUS_FLAG_B2;
}

void cpu_run(cpu_t *cpu) {
  cpu_reset(cpu);
  while (cpu_step(cpu));
}

void cpu_run_from(cpu_t *cpu, uint16_t start) {
  cpu_reset(cpu);
  cpu->registers.pc = start;
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
  char buffer[124] = { 0 };
  char *ptr = &buffer[0];

  uint8_t opcode = bus_mem_read_u8(cpu->bus, cpu->registers.pc);
  const cpu_instruction_t *instruction = &INSTRUCTIONS[opcode];

  if (!instruction->valid) {
    ptr += sprintf(ptr,
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

  uint16_t addr;
  uint8_t m;
  if (instruction->addr_mode != CPU_ADDRESSING_MODE_IMPLIED &&
      instruction->addr_mode != CPU_ADDRESSING_MODE_ACCUMULATOR &&
      instruction->addr_mode != CPU_ADDRESSING_MODE_RELATIVE) {
      addr = get_address(cpu, cpu->registers.pc + 1, instruction->addr_mode);
      m = bus_mem_read_u8(cpu->bus, addr);
  }
  
  ptr += sprintf(ptr, "%04X  %02X ", cpu->registers.pc, opcode);

  switch(get_addr_mode_byte_length(instruction->addr_mode)) {
    case 2:
      ptr += sprintf(ptr, "%02X %02X ", b1, b2);
      break;
    case 1:
      ptr += sprintf(ptr, "%02X    ", b1);
      break;
    case 0:
      ptr += sprintf(ptr, "      ");
      break;
    default:
      ptr += sprintf(ptr, "error: unexpected opcode byte length ");
      break;
  }

  ptr += sprintf(ptr, "%c%s ", instruction->unofficial ? '*' : ' ', instruction->name);

  #ifdef CNES_TRACE_ADDR_MODE
  switch (instruction->addr_mode) {
  case CPU_ADDRESSING_MODE_IMPLIED:       ptr += sprintf(ptr, "(imp) "); break;
  case CPU_ADDRESSING_MODE_ACCUMULATOR:   ptr += sprintf(ptr, "(acc) "); break;
  case CPU_ADDRESSING_MODE_IMMEDIATE:     ptr += sprintf(ptr, "(imm) "); break;
  case CPU_ADDRESSING_MODE_RELATIVE:      ptr += sprintf(ptr, "(rel) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE:     ptr += sprintf(ptr, "(zp ) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X:   ptr += sprintf(ptr, "(zpx) "); break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y:   ptr += sprintf(ptr, "(zpy) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_ADDR:
  case CPU_ADDRESSING_MODE_ABSOLUTE:      ptr += sprintf(ptr, "(abs) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:    ptr += sprintf(ptr, "(abx) "); break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:    ptr += sprintf(ptr, "(aby) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT:      ptr += sprintf(ptr, "(ind) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT_X:    ptr += sprintf(ptr, "(izx) "); break;
  case CPU_ADDRESSING_MODE_INDIRECT_Y:    ptr += sprintf(ptr, "(izy) "); break;
  default:                                ptr += sprintf(ptr, "(err) "); break;
  }
  #endif

  switch (instruction->addr_mode) {
  case CPU_ADDRESSING_MODE_IMPLIED:
    ptr += sprintf(ptr, "                            ");
    break;
  case CPU_ADDRESSING_MODE_ACCUMULATOR:
    ptr += sprintf(ptr, "A                           ");
    break;
  case CPU_ADDRESSING_MODE_IMMEDIATE:
    ptr += sprintf(ptr, "#$%02X                        ", b1);
    break;
  case CPU_ADDRESSING_MODE_RELATIVE:
    ptr += sprintf(ptr, "$%04X                       ", cpu->registers.pc + b1 + 2);
    break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE:
    ptr += sprintf(ptr, "$%02X = %02X                    ", addr, m);
    break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_X:
    ptr += sprintf(ptr, "$%02X,X @ %02X = %02X             ", b1, addr, m);
    break;
  case CPU_ADDRESSING_MODE_ZERO_PAGE_Y:
    ptr += sprintf(ptr, "$%02X,Y @ %02X = %02X             ", b1, addr, m);
    break;
  case CPU_ADDRESSING_MODE_ABSOLUTE:
    ptr += sprintf(ptr, "$%04X = %02X                  ", addr, m);
    break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_ADDR:
    ptr += sprintf(ptr, "$%04X                       ", addr);
    break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_X:
    ptr += sprintf(ptr, "$%04X,X @ %04X = %02X         ", bb, addr, m);
    break;
  case CPU_ADDRESSING_MODE_ABSOLUTE_Y:
    ptr += sprintf(ptr, "$%04X,Y @ %04X = %02X         ", bb, addr, m);
    break;
  case CPU_ADDRESSING_MODE_INDIRECT:
    ptr += sprintf(ptr, "($%04X) = %04X              ", bb, addr);
    break;
  case CPU_ADDRESSING_MODE_INDIRECT_X: {
    uint8_t zp_addr = b1 + cpu->registers.x;
    ptr += sprintf(ptr, "($%02X,X) @ %02X = %04X = %02X    ", b1, zp_addr, addr, m);
    break;
  }
  case CPU_ADDRESSING_MODE_INDIRECT_Y: {
    uint16_t zp_addr = bus_mem_read_u16_zero_page(cpu->bus, b1);
    ptr += sprintf(ptr, "($%02X),Y = %04X @ %04X = %02X  ", b1, zp_addr, addr, m);
    break;
  }
  default:
    ptr += sprintf(ptr, "cannot format addr mode     ");
    break;
  }

  ptr += sprintf(ptr, "A:%02X X:%02X Y:%02X P:%02X SP:%02X",
    cpu->registers.a,
    cpu->registers.x,
    cpu->registers.y,
    cpu->registers.flags,
    cpu->registers.s
  );
  *ptr = '\0';

  log_debug("%s", buffer);
}
