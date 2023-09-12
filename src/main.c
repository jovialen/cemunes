#include <stdio.h>
#include <string.h>

#include "bus.h"
#include "cpu.h"

int main(int argc, char **argv) {
	bus_t *bus = bus_new();
	cpu_t cpu = {
		.bus = bus,
		.registers = { 0 },
	};
	
	uint8_t program[] = { 0xa9, 0xc0, 0xaa, 0xe8, 0x00 };
	memcpy(bus->cpu_ram, program, sizeof(program));
	
	cpu_run(&cpu);

	bus_free(bus);
	return 0;
}
