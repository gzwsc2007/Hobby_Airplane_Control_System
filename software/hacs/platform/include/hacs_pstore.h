#ifndef _HACS_PSTORE_H_
#define _HACS_PSTORE_H_

#include "hacs_pstore_layout.h"

int hacs_pstore_init();
int hacs_pstore_get(hacs_pstore_type_t ent, uint8_t *rbuf, uint32_t byte_len);
int hacs_pstore_set(hacs_pstore_type_t ent, uint8_t *wbuf, uint32_t byte_len);

#endif
