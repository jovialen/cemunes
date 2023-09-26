#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "bus.h"
#include "cartridge.h"
#include "cpu.h"
#include "log.h"

typedef struct args_t {
	bool valid;
	const char *rom_path;
	bool start_pc;
	uint16_t pc;
} args_t;

uint8_t *read_bytes(const char *path, size_t *size) {
	log_debug("reading rom file %s", path);

	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		log_error("failed to open file %s", path);
		return NULL;
	}

	fseek(f, 0, SEEK_END);
	*size = ftell(f);
	fseek(f, 0, SEEK_SET);
	log_debug("source file size %zu", size);

	uint8_t *buffer = malloc(*size);
	if (buffer == NULL) {
		log_error("error: failed to allocate buffer");
		return NULL;
	}
	fread(buffer, sizeof(char), *size, f);

	return buffer;
}

args_t parse_args(int argc, char **argv) {
	args_t args = { 0 };

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-C") == 0) {
			if (args.start_pc) {
				log_warn("multiple custom start positons");
			}

			i++;
			args.start_pc = true;
			char *end = NULL;
			args.pc = strtoul(argv[i], &end, 16);
		}
		else {
			if (args.rom_path != NULL) {
				log_warn("multiple roms defined");
			}

			args.rom_path = argv[i];
		}
	}

	args.valid = args.rom_path != NULL;
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
	free(bytes);

	cpu_load_cartridge(&cpu, cart);

	if (args.start_pc) {
		log_debug("starting execution at %04X", args.pc);
		cpu_run_from(&cpu, args.pc);
	} else {
		log_debug("starting execution");
		cpu_run(&cpu);
	}

	bus_free(bus);
	cartridge_free(cart);
	return 0;
}
