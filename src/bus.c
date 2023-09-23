#include "bus.h"

#include <stdlib.h>
#include <string.h>

static uint8_t *bus_address(bus_t *bus, uint16_t address) {
	return &bus->memory[(address % BUS_MEM_SIZE)];
}

bus_t *bus_new() {
	bus_t *bus = malloc(sizeof(bus_t));
	memset(bus, 0, sizeof(bus_t));
	return bus;
}

void bus_free(bus_t *bus) {
	free(bus);
}

uint8_t *bus_mem_addr(bus_t *bus, uint16_t address) {
	return &bus->memory[address];
}

void bus_mem_read(bus_t *bus, uint16_t address, uint8_t *dst, size_t size) {
	memcpy(dst, bus_address(bus, address), size);
}

void bus_mem_write(bus_t *bus, uint16_t address, const uint8_t *src, size_t size) {
	memcpy(bus_address(bus, address), src, size);
}

uint8_t bus_mem_read_u8(bus_t *bus, uint16_t address) {
	return *bus_address(bus, address);
}

void bus_mem_write_u8(bus_t *bus, uint16_t address, uint8_t value) {
	*bus_address(bus, address) = value;
}

uint16_t bus_mem_read_u16(bus_t *bus, uint16_t address) {
	uint16_t low = bus_mem_read_u8(bus, address);
	uint16_t high = bus_mem_read_u8(bus, address + 1);
	return (high << 8) | low;
}

void bus_mem_write_u16(bus_t *bus, uint16_t address, uint16_t value) {
	uint8_t low = value & 0xFF;
	uint8_t high = (value >> 8) & 0xFF;
	bus_mem_write_u8(bus, address, low);
	bus_mem_write_u8(bus, address + 1, high);
}
