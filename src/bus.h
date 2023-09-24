#ifndef __cpu_bus_h__
#define __cpu_bus_h__

#include <stddef.h>
#include <stdint.h>

#include "cartridge.h"

#define RAM_START 0x0
#define RAM_END   0x2000
#define RAM_SIZE  0x0800
#define PPU_REG_START 0x2000
#define PPU_REG_END   0x4000
#define PPU_REG_SIZE  0x8
#define APU_REG_START 0x4000
#define APU_REG_END   0x4018
#define APU_REG_SIZE  0x0018
#define WROM_START 0x4020
#define WROM_END   0x8000
#define WROM_SIZE  0x3fe0
#define CARTRIDGE_START 0x8000
#define CARTRIDGE_END   0x10000
#define CARTRIDGE_SIZE  0x8000

#define RESET_VECTOR 0xFFFC


typedef struct bus_t {
  uint8_t ram[RAM_SIZE];
  uint8_t ppu_reg[PPU_REG_SIZE];
  uint8_t apu_reg[APU_REG_SIZE];
  uint8_t wrom[WROM_SIZE];
  const cartridge_t *cartridge;
} bus_t;

bus_t *bus_new();
void bus_free(bus_t *bus);
void bus_load_cartridge(bus_t *bus, const cartridge_t *cart);

uint8_t *bus_mem_addr(bus_t *bus, uint16_t address);
void bus_mem_read(bus_t *bus, uint16_t address, uint8_t *dst, size_t size);
void bus_mem_write(bus_t *bus, uint16_t address, const uint8_t *src, size_t size);

uint8_t bus_mem_read_u8(bus_t *bus, uint16_t address);
void bus_mem_write_u8(bus_t *bus, uint16_t address, uint8_t value);

uint16_t bus_mem_read_u16(bus_t *bus, uint16_t address);
void bus_mem_write_u16(bus_t *bus, uint16_t address, uint16_t value);


#endif /* __cpu_bus_h__ */
