#include "cartridge.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus.h"

#define INES_HEADER_SIZE 16
#define INES_ROM_BANK_SIZE 16384 
#define INES_VROM_BANK_SIZE 8192 
#define INES_TRAINER_SIZE 512

const uint8_t INES_HEADER_START[4] = { 0x4e, 0x45, 0x53, 0x1a };
const uint8_t INES_HEADER_END[6] = { 0 };

typedef struct ines_header_t {
  bool valid;
  bool has_trainer;
  uint8_t version;
  size_t rom_size;
  size_t vrom_size;
} ines_header_t;

static ines_header_t ines_header(const uint8_t *buffer, size_t size) {
  ines_header_t header = { 0 };
  
  if (memcmp(INES_HEADER_START, buffer, 4) != 0) {
    printf("error: not ines format");
    return header;
  }

  if (size < INES_HEADER_SIZE) {
    printf("error: too little data to read header");
    return header;
  }

  header.rom_size = buffer[4] * INES_ROM_BANK_SIZE;
  header.vrom_size = buffer[5] * INES_VROM_BANK_SIZE;
  header.has_trainer = buffer[6] & 0b100;
  header.version = (buffer[7] >> 2) & 0b11;

  if (memcmp(INES_HEADER_END, buffer + 0xA, 6) != 0) {
    printf("error: ines header improper ending");
    return header;
  }

  header.valid = true;
  return header;
}

cartridge_t *ines_to_cartridge(const uint8_t *buffer, size_t size) {
  ines_header_t header = ines_header(buffer, size);
  if (!header.valid) {
    printf("error: invalid ines header");
    return NULL;
  }
  
  if (header.version == 2) {
    printf("error: ines version 2 not supported");
    return NULL;
  }

  size_t rom_start = (header.has_trainer ? INES_TRAINER_SIZE : 0) + INES_HEADER_SIZE;
  size_t vrom_start = rom_start + header.rom_size;

  if (size < vrom_start + header.vrom_size) {
    printf("error: not enough data in buffer");
    return NULL;
  }
  
  cartridge_t *cart = malloc(sizeof(cartridge_t));

  cart->rom_size = header.rom_size;
  cart->vrom_size = header.vrom_size;

  cart->rom = malloc(header.rom_size);
  cart->vrom = malloc(header.vrom_size);

  memcpy(cart->rom, buffer + rom_start, header.rom_size);
  memcpy(cart->vrom, buffer + vrom_start, header.vrom_size);
  
  return cart;
}

void cartridge_free(cartridge_t *cart) {
  free(cart->rom);
  free(cart->vrom);
  free(cart);
}

void cartridge_read(const cartridge_t *cart, uint16_t address, uint8_t *dst, uint16_t size) {
  if (size > CARTRIDGE_SIZE) {
    printf("warn: attempt to read outside of memory; clamping to bounds\n");
    size = CARTRIDGE_SIZE;
  }
  
  address %= cart->rom_size;
  memcpy(dst, cart->rom + address, size);
}

uint8_t cartridge_read_u8(const cartridge_t *cart, uint16_t address) {
  address %= cart->rom_size;
  return cart->rom[address];
}
