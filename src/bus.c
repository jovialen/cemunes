#include "bus.h"

#include <stdlib.h>
#include <string.h>

bus_t *bus_new() {
	bus_t *bus = malloc(sizeof(bus_t));
	memset(bus, 0, sizeof(bus_t));
	return bus;
}

void bus_free(bus_t *bus) {
	free(bus);
}

uint8_t bus_mem_read(bus_t *bus, uint16_t address) {
	return bus->cpu_ram[address % 0x800];
}

void bus_mem_write(bus_t *bus, uint16_t address, uint8_t value) {
	bus->cpu_ram[address % 0x800] = value;
}
