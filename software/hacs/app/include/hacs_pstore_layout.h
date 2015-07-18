#ifndef _HACS_PSTORE_LAYOUT_H_
#define _HACS_PSTORE_LAYOUT_H_

#define HACS_PSTORE_LAYOUT_VERSION    (1)

// Header definition. The first field must always be a 32-bit version field.
typedef struct {
  uint32_t layout_version; // must not be 0xFFFF which means the entire bank is invalid
  uint32_t bank_version; // used to determine which bank is the most recent one
} hacs_pstore_header_t;

// Entry definition
typedef struct {
  uint32_t byte_len;
  uint32_t crc;
} hacs_pstore_entry_t;

// Entry allocation. Think of this as laying out the entries one by one manually.
// For each new entry added, the ent_reL_addr table and the ent_size table should
// be amended correspondingy.
typedef enum {
  HACS_PSTORE_MAG_HARD_CAL, // Magnetometer hard-iron calibration
  HACS_PSTORE_MAG_SOFT_CAL, // Magnetometer soft-iron calibration
//  HACS_PSTORE_RC_RECVR_CAL, // RC receiver channel range calibration

  HACS_PSTORE_NUM_ENTRIES,
} hacs_pstore_type_t;

// Table for the relative addresses of each PSTORE entry. Each address is relative
// to the start address of a PSTORE bank (thus including the bank header).
extern const uint32_t hacs_pstore_ent_rel_addr[HACS_PSTORE_NUM_ENTRIES];
extern const uint16_t hacs_pstore_ent_size[HACS_PSTORE_NUM_ENTRIES];

#include "hacs_platform.h"
#include "hacs_sensor_cal.h"

#define PSTORE_MAG_HARD_CAL_ADDR      (ALIGN_TO_WORD(sizeof(hacs_pstore_header_t)))
#define PSTORE_MAG_HARD_CAL_LEN       (ALIGN_TO_WORD(sizeof(float32_t) * HARD_IRON_MAT_ROW * \
                                       HARD_IRON_MAT_COL))

#define PSTORE_MAG_SOFT_CAL_ADDR      (ALIGN_TO_WORD(PSTORE_MAG_HARD_CAL_ADDR + \
                                       sizeof(hacs_pstore_entry_t) \
                                       + PSTORE_MAG_HARD_CAL_LEN))
#define PSTORE_MAG_SOFT_CAL_LEN       (ALIGN_TO_WORD(sizeof(float32_t) * SOFT_IRON_MAT_ROW * \
                                       SOFT_IRON_MAT_COL))

#endif