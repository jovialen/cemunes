#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "bus.h"
#include "cartridge.h"
#include "cpu.h"

uint8_t *read_bytes(const char *path, size_t *size) {
	FILE *f = fopen(path, "rb");

	fseek(f, 0, SEEK_END);
	*size = ftell(f);
	fseek(f, 0, SEEK_SET);

	uint8_t *buffer = malloc(*size);
	fread(buffer, sizeof(char), *size, f);

	return buffer;
}

int main(int argc, char **argv) {
	bus_t *bus = bus_new();
	cpu_t cpu = {
		.bus = bus,
		.registers = { 0 },
	};

	size_t size;
	uint8_t *bytes = read_bytes("./snake.nes", &size);
	cartridge_t *cart = ines_to_cartridge(bytes, size);
	printf("%zu %zu %zu\n", size, cart->rom_size, cart->vrom_size);
	
	cpu_load_program(&cpu, cart);
	
	cpu_run(&cpu);

	bus_free(bus);
	cartridge_free(cart);
	return 0;
}
