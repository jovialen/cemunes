#include <stdio.h>

#include "bus.h"

int main(int argc, char **argv) {
	bus_t *bus = bus_new();

	bus_free(bus);
	return 0;
}
