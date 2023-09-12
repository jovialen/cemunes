#ifndef __cpu_bus_h__
#define __cpu_bus_h__

#include <stdint.h>


typedef struct bus_t {
  uint8_t cpu_ram[0x800];
} bus_t;

bus_t *bus_new();
void bus_free(bus_t *bus);

uint8_t bus_mem_read(bus_t *bus, uint16_t address);
void bus_mem_write(bus_t *bus, uint16_t address, uint8_t value);


#endif /* __cpu_bus_h__ */
