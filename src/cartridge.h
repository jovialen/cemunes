#ifndef __cnes_cartridge_h__
#define __cnes_cartridge_h__

#include <stddef.h>
#include <stdint.h>


typedef struct cartridge_t {
  uint8_t *rom;
  size_t rom_size;
  uint8_t *vrom;
  size_t vrom_size;
} cartridge_t;

cartridge_t *ines_to_cartridge(uint8_t *buffer, size_t size);
void cartridge_free(cartridge_t *cart);


#endif /* __cnes_cartridge_h__ */
