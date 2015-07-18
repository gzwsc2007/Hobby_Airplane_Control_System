#include <string.h>
#include <assert.h>
#include "hacs_platform.h"
#include "hacs_pstore.h"
#include "stm32f4xx_hal.h"
#include "hacs_crc.h"

static uint32_t current_bank_addr;
static uint32_t alternate_bank_addr;
static uint8_t ram_buf[ALIGN_TO_WORD(HACS_PSTORE_BANK_SIZE)];

int hacs_pstore_init()
{
  hacs_pstore_header_t *header0 = (hacs_pstore_header_t *)HACS_PSTORE_0_ADDR;
  hacs_pstore_header_t *header1 = (hacs_pstore_header_t *)HACS_PSTORE_1_ADDR;
  uint32_t l_version0 = header0->layout_version;
  uint32_t l_version1 = header1->layout_version;
  
  // Determine the current bank and the alternate bank.
  // First, check if one of the banks is erased, by looking at the layout_version field.
  // If neither is erased or both are erased, proceed to compare the bank_version count.
  if (l_version0 == 0xFFFFFFFF && l_version1 != 0xFFFFFFFF) {
    current_bank_addr = HACS_PSTORE_1_ADDR;
    alternate_bank_addr = HACS_PSTORE_0_ADDR;
  } else if (l_version1 == 0xFFFFFFFF && l_version0 != 0xFFFFFFFF) {
    current_bank_addr = HACS_PSTORE_0_ADDR;
    alternate_bank_addr = HACS_PSTORE_1_ADDR;
  } else {
    uint32_t b_version0 = header0->bank_version;
    uint32_t b_version1 = header1->bank_version;

    if (b_version0 == 0xFFFFFFFF && b_version1 == 0) {
      current_bank_addr = HACS_PSTORE_1_ADDR;
      alternate_bank_addr = HACS_PSTORE_0_ADDR;
    } else if (b_version0 = 0 && b_version1 == 0xFFFFFFFF) {
      current_bank_addr = HACS_PSTORE_0_ADDR;
      alternate_bank_addr = HACS_PSTORE_1_ADDR;
    } else if (b_version0 < b_version1) {
      current_bank_addr = HACS_PSTORE_1_ADDR;
      alternate_bank_addr = HACS_PSTORE_0_ADDR;
    } else {
      current_bank_addr = HACS_PSTORE_0_ADDR;
      alternate_bank_addr = HACS_PSTORE_1_ADDR;
    }
  }

  return HACS_NO_ERROR;
}

int hacs_pstore_get(hacs_pstore_type_t ent, uint8_t *rbuf, uint32_t byte_len)
{
  uint32_t ent_addr_abs = current_bank_addr + hacs_pstore_ent_rel_addr[ent];
  uint8_t first = 1;
  hacs_pstore_entry_t *ent_header;
  uint8_t *ent_content;
  int retval;

top:
  ent_header = (hacs_pstore_entry_t*)ent_addr_abs;

  // Check entry size. Make sure the read buffer isn't too small.
  if (byte_len < ent_header->byte_len) {
    if (first) {
      // Failed current bank. Try the other bank.
      ent_addr_abs = alternate_bank_addr + hacs_pstore_ent_rel_addr[ent];
      first = 0;
      goto top;
    } else {
      // We failed size check for both banks. Treat it as a message corruption.
      retval = -HACS_BAD_MESSAGE;
      goto done;
    }
  }

  // Validate CRC
  ent_content = (uint8_t *)(ent_addr_abs + sizeof(hacs_pstore_entry_t)); // skip the entry header
  if (ent_header->crc != hacs_crc32_calc(ent_content, ent_header->byte_len)) {
    if (first) {
      // Failed current bank. Try the other bank.
      ent_addr_abs = alternate_bank_addr + hacs_pstore_ent_rel_addr[ent];
      first = 0;
      goto top;
    } else {
      // We failed CRC for both banks. Message corrupted.
      retval = -HACS_BAD_MESSAGE;
      goto done;
    }
  }

  // Read in the entry content
  memcpy(rbuf, ent_content, ent_header->byte_len);

  retval = HACS_NO_ERROR;
done:
  return retval;
}

int hacs_pstore_set(hacs_pstore_type_t ent, uint8_t *wbuf, uint32_t byte_len)
{
  uint32_t ent_addr = hacs_pstore_ent_rel_addr[ent];
  hacs_pstore_entry_t *ent_header;
  hacs_pstore_header_t *bank_header;
  uint8_t *buf_ptr;
  uint32_t counter;
  uint32_t temp;
  uint32_t sector;
  int retval;

  // Read in the entire current bank
  memcpy(ram_buf, (uint8_t*)current_bank_addr, HACS_PSTORE_BANK_SIZE);

  // Check entry size. Make sure we are not writing something too big.
  retval = -HACS_INVALID_ARGS;
  HACS_REQUIRES(byte_len <= hacs_pstore_ent_size[ent], done);

  // Update entry content
  buf_ptr = (uint8_t*)(ram_buf + ent_addr + sizeof(hacs_pstore_entry_t));
  memcpy(buf_ptr, wbuf, byte_len);

  // Update entry header
  ent_header = (hacs_pstore_entry_t*)(ram_buf + ent_addr);
  ent_header->byte_len = byte_len;
  ent_header->crc = hacs_crc32_calc(wbuf, byte_len); // each entry has its own CRC

  // Update bank header
  bank_header = (hacs_pstore_header_t *)ram_buf;
  bank_header->bank_version++;
  bank_header->layout_version = HACS_PSTORE_LAYOUT_VERSION;

  // Write the ram buffer to the alternate bank
  HAL_FLASH_Unlock();
  sector = (alternate_bank_addr == HACS_PSTORE_0_ADDR) ? HACS_PSTORE_0_SECTOR : HACS_PSTORE_1_SECTOR;
  FLASH_Erase_Sector(sector, VOLTAGE_RANGE_3);
  // Write the ram buffer word by word.
  counter = 0;
  while(counter < sizeof(ram_buf)) {
    HAL_FLASH_Program(TYPEPROGRAM_WORD, alternate_bank_addr+counter,
                      *(uint32_t*)(ram_buf+counter)); // unaligned access supported
    counter += 4;
  }
  // Note that I exploit the fact that the ram buffer is word-aligned 
  // (its length is multiple of 4).
  assert(counter == sizeof(ram_buf));
  HAL_FLASH_Lock();

  // Switch current bank and alternate bank
  temp = alternate_bank_addr;
  alternate_bank_addr = current_bank_addr;
  current_bank_addr = temp;

  retval = HACS_NO_ERROR;
done:
  return retval;
}
