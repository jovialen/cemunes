#include "bus.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "log.h"

static uint8_t *map_memory(bus_t *bus, uint16_t address) {
	if (/* address >= RAM_START && */ address < RAM_END) {
		return bus->ram;
	} else if (address >= PPU_REG_START && address < PPU_REG_END) {
		return bus->ppu_reg;
	} else if (address >= APU_REG_START && address < APU_REG_END) {
		return bus->apu_reg;
	} else if (address >= WROM_START && address < WROM_END) {
		return bus->wrom;
	} else if (address >= CARTRIDGE_START /* && address < CARTRIDGE_END */) {
		log_error("should use cartridge read instead of mapping");
		return NULL;
	} else {
		log_error("cannot map address $%04X", address);
		return NULL;
	}
}

static uint16_t map_memory_size(uint16_t address) {
	if (/* address >= RAM_START && */ address < RAM_END) {
		return RAM_SIZE;
	} else if (address >= PPU_REG_START && address < PPU_REG_END) {
		return PPU_REG_SIZE;
	} else if (address >= APU_REG_START && address < APU_REG_END) {
		return APU_REG_SIZE;
	} else if (address >= WROM_START && address < WROM_END) {
		return WROM_SIZE;
	} else if (address >= CARTRIDGE_START /* && address < CARTRIDGE_END */) {
		return CARTRIDGE_SIZE;
	} else {
		log_error("cannot map address $%04X", address);
		return 0;
	}
}

static uint16_t map_address(uint16_t address) {
	if (/* address >= RAM_START && */ address < RAM_END) {
		return (address - RAM_START) % RAM_SIZE;
	} else if (address >= PPU_REG_START && address < PPU_REG_END) {
		return (address - PPU_REG_START) % PPU_REG_SIZE;
	} else if (address >= APU_REG_START && address < APU_REG_END) {
		return (address - APU_REG_START) % APU_REG_SIZE;
	} else if (address >= WROM_START && address < WROM_END) {
		return (address - WROM_START) % WROM_SIZE;
	} else if (address >= CARTRIDGE_START /* && address < CARTRIDGE_END */) {
		return (address - CARTRIDGE_START) % CARTRIDGE_SIZE;
	} else {
		log_error("cannot map address $%04X", address);
		return address;
	}
}

bus_t *bus_new() {
	bus_t *bus = malloc(sizeof(bus_t));
	if (bus == NULL) {
		log_error("failed to allocate bus memory");
	}

	memset(bus, 0, sizeof(bus_t));
	return bus;
}

void bus_free(bus_t *bus) {
	free(bus);
}

void bus_load_cartridge(bus_t *bus, const cartridge_t *cart) {
	bus->cartridge = cart;
}

uint8_t *bus_mem_addr(bus_t *bus, uint16_t address) {
	if (address >= CARTRIDGE_START) {
		log_error("cannot get pointer to rom address");
		return NULL;
	} else {
		uint8_t *memory = map_memory(bus, address);
		address = map_address(address);
		return &memory[address];
	}
}

void bus_mem_read(bus_t *bus, uint16_t address, uint8_t *dst, size_t size) {
	if (address >= CARTRIDGE_START) {
		cartridge_read(bus->cartridge, address - CARTRIDGE_START, dst, size);
		return;
	}
	
	uint8_t *memory = map_memory(bus, address);
	uint16_t mem_size = map_memory_size(address);
	uint16_t local_address = map_address(address);

	if (local_address + size > mem_size) {
		uint16_t read = mem_size - local_address;
		memcpy(dst, memory + local_address, read);
		bus_mem_read(bus, address + read, dst + read, size - read);
	} else {
		memcpy(dst, memory + address, size);
	}
}

void bus_mem_write(bus_t *bus, uint16_t address, const uint8_t *src, size_t size) {
	if (address >= CARTRIDGE_START) {
		log_error("cannot write to rom");
		return;
	}
	
	uint8_t *memory = map_memory(bus, address);
	uint16_t mem_size = map_memory_size(address);
	uint16_t local_address = map_address(address);

	if (local_address + size > mem_size) {
		uint16_t write = mem_size - local_address;
		memcpy(memory + local_address, src, write);
		bus_mem_write(bus, address + write, src + write, size - write);
	} else {
		memcpy(memory + address, src, size);
	}
}

uint8_t bus_mem_read_u8(bus_t *bus, uint16_t address) {
	if (address >= CARTRIDGE_START) {
		return cartridge_read_u8(bus->cartridge, address);
	}
	return *bus_mem_addr(bus, address);
}

void bus_mem_write_u8(bus_t *bus, uint16_t address, uint8_t value) {
	if (address >= CARTRIDGE_START) {
		log_error("cannot write to cartridge rom");
		return;
	}
	*bus_mem_addr(bus, address) = value;
}

uint16_t bus_mem_read_u16(bus_t *bus, uint16_t address) {
	return uint16_from_bytes(bus_mem_read_u8(bus, address + 1), bus_mem_read_u8(bus, address));
}

void bus_mem_write_u16(bus_t *bus, uint16_t address, uint16_t value) {
	bus_mem_write_u8(bus, address, low_byte(value));
	bus_mem_write_u8(bus, address + 1, high_byte(value));
}

uint16_t bus_mem_read_u16_zero_page(bus_t *bus, uint8_t addr) {
	uint8_t low = bus_mem_read_u8(bus, addr);
	uint8_t high = bus_mem_read_u8(bus, (addr + 1) % 0x100);
	return uint16_from_bytes(high, low);
}
