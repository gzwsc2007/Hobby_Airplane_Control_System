#ifndef _HACS_CRC_H_
#define _HACS_CRC_H_

int hacs_crc_init(void);
uint32_t hacs_crc32_calc(uint8_t *buf, uint32_t len);
uint32_t hacs_crc32_accum(uint8_t *buf, uint32_t len);

#endif
