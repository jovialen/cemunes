#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "bus.h"
#include "cartridge.h"
#include "cpu.h"

typedef struct args_t {
	bool valid;
	const char *rom_path;
	bool start_pc;
	uint16_t pc;
} args_t;

uint8_t *read_bytes(const char *path, size_t *size) {
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		printf("error: failed to open file %s\n", path);
		return NULL;
	}

	fseek(f, 0, SEEK_END);
	*size = ftell(f);
	fseek(f, 0, SEEK_SET);

	uint8_t *buffer = malloc(*size);
	if (buffer == NULL) {
		printf("error: failed to allocate buffer\n");
		return NULL;
	}
	fread(buffer, sizeof(char), *size, f);

	return buffer;
}

args_t parse_args(int argc, char **argv) {
	args_t args = { 0 };

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-C") == 0) {
			i++;
			args.start_pc = true;
			char *end = NULL;
			args.pc = strtoul(argv[i], &end, 16);
		}
		else {
			args.rom_path = argv[i];
		}
	}

	args.valid = true;
	return args;
}

int main(int argc, char **argv) {
	args_t args = parse_args(argc, argv);
	if (!args.valid) {
		printf("Usage: cnes [rom path] {options}\n");
		printf("\nOptions:\n");
		printf("-C pc : Custom starting address\n");
		return 1;
	}
	
	bus_t *bus = bus_new();
	cpu_t cpu = {
		.bus = bus,
		.registers = { 0 },
	};

	size_t size;
	uint8_t *bytes = read_bytes(args.rom_path, &size);
	if (bytes == NULL) {
		return 2;
	}

	cartridge_t *cart = ines_to_cartridge(bytes, size);
	cpu_load_cartridge(&cpu, cart);

	if (args.start_pc) {
		cpu.registers.pc = args.pc;
	}
	cpu_run(&cpu);

	bus_free(bus);
	cartridge_free(cart);
	return 0;
}
