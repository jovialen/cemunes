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
