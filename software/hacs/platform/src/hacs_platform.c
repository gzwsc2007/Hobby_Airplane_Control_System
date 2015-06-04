#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_debug_uart.h"
#include "hacs_spi_master.h"
#include "hacs_i2c_master.h"
#include "hacs_uart.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "nrf24l01.h"
#include "hmc5883.h"
#include "gps_serial.h"
#include "mpu6050_serial.h"
#include "bmp085.h"

#include "hacs_telemetry.h"

/* Platform static data */
uint8_t hacs_critical_ref_count = 0; // Critical section reference count

/* Function prototypes */
static void system_clock_config(void);
static void error_handler(void);

void hacs_platform_init(void)
{
  /* Turn on all GPIO clocks since we don't care about power */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /* Turn on all DMA clocks since we don't care about power */
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

	/* STM32F4xx HAL library initialization:
   - Configure the Flash prefetch, Flash preread and Buffer caches
   - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
   - Low Level Initialization
  */
  HAL_Init();
  
  /* Configure the System clock to 100 MHz */
  system_clock_config();

	/* Init debug UART */
  debug_uart_init(115200);

	/* Init SPI master */
  spi_master_init(HACS_SPI_NRF24, 2000000, HACS_SPI_CPOL_0, HACS_SPI_CPHA_0);

	/* Init I2C master */
  if (i2c_master_init(HACS_I2C, 100000) != 0) {
    printf("Error in i2c_master_init!\r\n");
  }

	/* Init USART */
  if (hacs_uart_init(HACS_UART_GPS, 115200, 
                     HACS_UART_NOT_USE_TX_DMA, HACS_UART_USE_RX_DMA) != 0) {
    printf("Error in GPS uart_init!\r\n");
  }
  if (hacs_uart_init(HACS_UART_MPU6050, 115200, 
                     HACS_UART_NOT_USE_TX_DMA, HACS_UART_USE_RX_DMA) != 0) {
    printf("Error in MPU6050 uart_init!\r\n");
  }

	// TODO: Move sensor init into the sensor_manager thread
  if (hmc5883_init() != 0) {
    printf("Error in hmc5883_init!\r\n");
  }

  /* Early (pre-scheduler) init for devices */
  nrf24_early_init();
  gps_early_init();
  mpu6050_early_init();
  bmp085_early_init();

  hacs_telemetry_early_init();
}

// Redirect putc to UART send
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// Retargets the C library printf function to the USART.
PUTCHAR_PROTOTYPE
{
  debug_uart_putchar(ch); 
  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void system_clock_config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSI Oscillator and activate PLL with HSI as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 0x10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		error_handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		error_handler();
	}
}

static void error_handler(void)
{
    while(1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

// Dummy functions to stop the HAL from crashing us
HAL_StatusTypeDef HAL_InitTick(uint32_t dummy)
{
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return xTaskGetTickCount();
}
