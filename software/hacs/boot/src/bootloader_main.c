#include "stm32f411xe.h"
#include "core_cm4.h"

#define HACS_APP_BASE_ADDR    ((uint32_t*)0x0800C000)

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
  jump_to_app(HACS_APP_BASE_ADDR);

  while(1); // should never reach here
}
