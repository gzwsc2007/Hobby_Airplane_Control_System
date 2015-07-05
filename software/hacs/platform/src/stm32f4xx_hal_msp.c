/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2014
  * @brief   HAL MSP module.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by MicroXplorer and eventually modified 
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "hacs_platform.h"
#include "hacs_platform_resources.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  if (huart->Instance == USART2) {
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USART2_TX_GPIO_CLK_ENABLE();
    USART2_RX_GPIO_CLK_ENABLE();

    /* Enable USART2 clock */
    __USART2_CLK_ENABLE();
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USART2_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USART2_TX_AF;
    HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);
      
    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USART2_RX_PIN;
    GPIO_InitStruct.Alternate = USART2_RX_AF;
    HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);

    /* Enable RXNE interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    NVIC_SetPriority(USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3);
    NVIC_EnableIRQ(USART2_IRQn);
    
  }  else if (huart->Instance==USART6) {
    GPIO_InitTypeDef  GPIO_InitStruct;
      
    /* Peripheral clock enable */
    __USART6_CLK_ENABLE();
    USART6_TX_GPIO_CLK_ENABLE();
    USART6_RX_GPIO_CLK_ENABLE();
  
    /* USART6 GPIO Configuration */
    GPIO_InitStruct.Pin = USART6_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = USART6_TX_AF;
    HAL_GPIO_Init(USART6_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART6_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = USART6_RX_AF;
    HAL_GPIO_Init(USART6_RX_PORT, &GPIO_InitStruct);

  } else if (huart->Instance==USART1) {
    GPIO_InitTypeDef  GPIO_InitStruct;
      
    /* Peripheral clock enable */
    __USART1_CLK_ENABLE();
    USART1_TX_GPIO_CLK_ENABLE();
    USART1_RX_GPIO_CLK_ENABLE();
  
    /* USART1 GPIO Configuration */
    GPIO_InitStruct.Pin = USART1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = USART1_TX_AF;
    HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = USART1_RX_AF;
    HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
  }
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    /*##-1- Reset peripherals ##################################################*/
    __USART2_FORCE_RESET();
    __USART2_RELEASE_RESET();

    __USART2_CLK_DISABLE();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure UART Tx as alternate function  */
    HAL_GPIO_DeInit(USART2_TX_PORT, USART2_TX_PIN);
    /* Configure UART Rx as alternate function  */
    HAL_GPIO_DeInit(USART2_RX_PORT, USART2_RX_PIN);  
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI3) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    SPI3_MOSI_GPIO_CLK_ENABLE();
    SPI3_MISO_GPIO_CLK_ENABLE();
    SPI3_SCK_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin       = SPI3_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = SPI3_MOSI_AF;
    HAL_GPIO_Init(SPI3_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = SPI3_SCK_PIN;
    GPIO_InitStruct.Alternate = SPI3_SCK_AF;
    HAL_GPIO_Init(SPI3_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = SPI3_MISO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = SPI3_MISO_AF;
    HAL_GPIO_Init(SPI3_MISO_PORT, &GPIO_InitStruct);

    // TODO: initialize DMA and IT

    __SPI3_CLK_ENABLE();
  } else if (hspi->Instance == SPI2) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    SPI2_MOSI_GPIO_CLK_ENABLE();
    SPI2_MISO_GPIO_CLK_ENABLE();
    SPI2_SCK_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin       = SPI2_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = SPI2_MOSI_AF;
    HAL_GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = SPI2_SCK_PIN;
    GPIO_InitStruct.Alternate = SPI2_SCK_AF;
    HAL_GPIO_Init(SPI2_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = SPI2_MISO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = SPI2_MISO_AF;
    HAL_GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStruct);

    // TODO: initialize DMA and IT

    __SPI2_CLK_ENABLE();
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI3) {
    __SPI3_FORCE_RESET();
    __SPI3_RELEASE_RESET();

    __SPI3_CLK_DISABLE();

    HAL_GPIO_DeInit(SPI3_MOSI_PORT, SPI3_MOSI_PIN);
    HAL_GPIO_DeInit(SPI3_MISO_PORT, SPI3_MISO_PIN);
    HAL_GPIO_DeInit(SPI3_SCK_PORT, SPI3_SCK_PIN);
  } else if (hspi->Instance == SPI2) {
    __SPI2_FORCE_RESET();
    __SPI2_RELEASE_RESET();

    __SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
    HAL_GPIO_DeInit(SPI2_MISO_PORT, SPI2_MISO_PIN);
    HAL_GPIO_DeInit(SPI2_SCK_PORT, SPI2_SCK_PIN);
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock enable */
    I2C1_SDA_GPIO_CLK_ENABLE();
    I2C1_SCL_GPIO_CLK_ENABLE();
    __I2C1_CLK_ENABLE();

    /**I2C1 GPIO Configuration    
    */
    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2C1_SDA_AF;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2C1_SCL_AF;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance == TIM1)
  {
    // I am too lazy to enable GPIO clocks one by one. Just assume that
    // all the GPIO clocks are already enabled.
    __TIM1_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

    GPIO_InitStruct.Pin = TIM1_CHAN1_PIN;
    GPIO_InitStruct.Alternate = TIM1_CHAN1_AF;
    HAL_GPIO_Init(TIM1_CHAN1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM1_CHAN2_PIN;
    GPIO_InitStruct.Alternate = TIM1_CHAN2_AF;
    HAL_GPIO_Init(TIM1_CHAN2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM1_CHAN3_PIN;
    GPIO_InitStruct.Alternate = TIM1_CHAN3_AF;
    HAL_GPIO_Init(TIM1_CHAN3_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM1_CHAN4_PIN;
    GPIO_InitStruct.Alternate = TIM1_CHAN4_AF;
    HAL_GPIO_Init(TIM1_CHAN4_PORT, &GPIO_InitStruct);
  }
  else if (htim_base->Instance == TIM3)
  {
    // I am too lazy to enable GPIO clocks one by one. Just assume that
    // all the GPIO clocks are already enabled.
    __TIM3_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

    GPIO_InitStruct.Pin = TIM3_CHAN1_PIN;
    GPIO_InitStruct.Alternate = TIM3_CHAN1_AF;
    HAL_GPIO_Init(TIM3_CHAN1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM3_CHAN2_PIN;
    GPIO_InitStruct.Alternate = TIM3_CHAN2_AF;
    HAL_GPIO_Init(TIM3_CHAN2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM3_CHAN3_PIN;
    GPIO_InitStruct.Alternate = TIM3_CHAN3_AF;
    HAL_GPIO_Init(TIM3_CHAN3_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TIM3_CHAN4_PIN;
    GPIO_InitStruct.Alternate = TIM3_CHAN4_AF;
    HAL_GPIO_Init(TIM3_CHAN4_PORT, &GPIO_InitStruct);
  }
  else if (htim_base->Instance == TIM5)
  {
    __TIM5_CLK_ENABLE();
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
