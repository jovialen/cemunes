#ifndef __cpu_bus_h__
#define __cpu_bus_h__

#include <stddef.h>
#include <stdint.h>

#define BUS_MEM_SIZE 0x10000
#define PROGRAM_START_ADDR 0x8000
#define RESET_VECTOR 0xFFFC


typedef struct bus_t {
  uint8_t memory[BUS_MEM_SIZE];
} bus_t;

bus_t *bus_new();
void bus_free(bus_t *bus);

uint8_t *bus_mem_addr(bus_t *bus, uint16_t address);
void bus_mem_read(bus_t *bus, uint16_t address, uint8_t *dst, size_t size);
void bus_mem_write(bus_t *bus, uint16_t address, const uint8_t *src, size_t size);

uint8_t bus_mem_read_u8(bus_t *bus, uint16_t address);
void bus_mem_write_u8(bus_t *bus, uint16_t address, uint8_t value);

uint16_t bus_mem_read_u16(bus_t *bus, uint16_t address);
void bus_mem_write_u16(bus_t *bus, uint16_t address, uint16_t value);


#endif /* __cpu_bus_h__ */
