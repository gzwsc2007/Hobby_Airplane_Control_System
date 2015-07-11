#include "hacs_platform.h"

static void jump_to_app(uint32_t *base_addr)
{
  // Update stack pointer
  if (__get_CONTROL() & 0x2) {
    __set_PSP(*base_addr);
  } else {
    __set_MSP(*base_addr);
  }

  base_addr++;

  __ASM("LDR R0, [%0]\n\t" :: "r"(base_addr));
  __ASM("BX R0\n\t");
}

int main(void)
{
  jump_to_app((uint32_t*)HACS_APP_BASE_ADDR);

  while(1); // should never reach here
}
