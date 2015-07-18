#include "hacs_platform.h"
#include "hacs_pstore_layout.h"

const uint32_t hacs_pstore_ent_rel_addr[HACS_PSTORE_NUM_ENTRIES] = {
  [HACS_PSTORE_MAG_HARD_CAL] = PSTORE_MAG_HARD_CAL_ADDR,
  [HACS_PSTORE_MAG_SOFT_CAL] = PSTORE_MAG_SOFT_CAL_ADDR,
};

const uint16_t hacs_pstore_ent_size[HACS_PSTORE_NUM_ENTRIES] = {
  [HACS_PSTORE_MAG_HARD_CAL] = PSTORE_MAG_HARD_CAL_LEN,
  [HACS_PSTORE_MAG_SOFT_CAL] = PSTORE_MAG_SOFT_CAL_LEN,
};
