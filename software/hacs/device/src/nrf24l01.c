#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "task.h"
#include "semphr.h"
#include "hacs_spi_master.h"
#include "hacs_gpio.h"
#include "nrf24l01.h"

#define NRF24_DRIVER_STACK_SIZE   (128)
#define NRF24_DRIVER_PRIORITY     (6)

#define NRF24_CSN_LOW() spi_master_assert_cs(HACS_SPI_NRF24)
#define NRF24_CSN_HIGH() spi_master_deassert_cs(HACS_SPI_NRF24)
#define NRF24_CE_LOW() gpio_write_low(NRF24_CE_PORT, NRF24_CE_PIN)
#define NRF24_CE_HIGH() gpio_write_high(NRF24_CE_PORT, NRF24_CE_PIN)

/*** Low-level helpers ***/
static int nrf24_write_reg(uint8_t reg, uint8_t val);
static int nrf24_read_reg(uint8_t reg, uint8_t *val);
static int nrf24_burst_write(uint8_t cmd, uint8_t *data, uint8_t len);
static int nrf24_burst_write_reg(uint8_t reg, uint8_t *data, uint8_t len);
static int nrf24_burst_read(uint8_t reg, uint8_t *rbuf, uint8_t rlen);
static int nrf24_send_cmd(uint8_t cmd);
/** Helpers 
  * Partly ported from Mike McCauley's (mikem@open.com.au) 
  * NRF24 library for Arduino.
**/
static int nrf24_soft_reset(void);
static int nrf24_clear_irq(uint8_t bits_to_clear);
static int nrf24_set_channel(uint8_t chan);
static int nrf24_set_this_addr(uint8_t *addr, uint8_t len);
static int nrf24_set_transmit_addr(uint8_t *addr, uint8_t len);
static int nrf24_set_payload_size(uint8_t size);
static int nrf24_set_rf(NRF24DataRate dr, NRF24TransmitPower pwr);

static int nrf24_init(void);
static int nrf24_radio_config(void);
static void nrf24_irq_handler(void);
static int nrf24_recv(uint8_t *rbuf, uint8_t *plen);

static uint8_t gnd_addr[] = {0x68,0x86,0x66,0x88,0x28};
static xSemaphoreHandle send_sema4;
static xSemaphoreHandle irq_sema4;
static xSemaphoreHandle send_lock;
static xQueueHandle msg_queue;
static volatile uint8_t nrf24_status;

xQueueHandle nrf24_get_msg_queue(void) {
  return msg_queue;
}

void nrf24_driver_task(void *param) {
  nrf24_msg_t rx;

  if (nrf24_init() != HACS_NO_ERROR) {
    printf("Error in nrf24l01_init!\r\n");
  } else {
    printf("NRF24 Init Complete!\r\n");
  }

  while(1) {
    // Wait for the Radio IRQ to fire
    xSemaphoreTake(irq_sema4, portMAX_DELAY);

    /* Handle the Radio IRQ */

    // Read status register
    nrf24_read_reg(NRF24_REG_07_STATUS, (uint8_t *)&nrf24_status);
    
    // Clear NRF24 IRQ
    delay_us(NRF24_CSN_INACTIVE_HOLD_US);
    nrf24_clear_irq(nrf24_status & (NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT));

    if (nrf24_status & NRF24_RX_DR) {
      rx.len = 0;
      nrf24_recv(rx.buf, &rx.len);

      xQueueSend(msg_queue, &rx, MS_TO_TICKS(100));
    }

    if (nrf24_status & NRF24_MAX_RT) {
      nrf24_send_cmd(NRF24_COMMAND_FLUSH_TX);
      xSemaphoreGive(send_sema4);
    } else if (nrf24_status & NRF24_TX_DS) {
      xSemaphoreGive(send_sema4);
    }
  }
}

static void nrf24_irq_handler(void) {
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Wake up the driver thread to handle this IRQ */
  // TODO: Consider using Direct Task Notification in the future
  xSemaphoreGiveFromISR(irq_sema4,&xHigherPriorityTaskWoken);

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static uint8_t nrf24_get_rx_payload_len(void) {
  uint8_t len = 0;
  nrf24_burst_read(NRF24_COMMAND_R_RX_PL_WID, &len, sizeof(len));
  return len;
}

static int nrf24_recv(uint8_t *rbuf, uint8_t *plen) {
  if (nrf24_status & NRF24_RX_DR) {
    *plen = nrf24_get_rx_payload_len();
    if (*plen <= NRF24_MAX_MESSAGE_LEN) {
      nrf24_burst_read(NRF24_COMMAND_R_RX_PAYLOAD, rbuf, *plen);
    } else {
      return HACS_NRF24_RX_INVALID_LENGTH;
    }
  } else {
    return HACS_NRF24_RX_NOT_READY;
  }

  return HACS_NO_ERROR;
}

int nrf24_early_init(void) {
  // Create a binary semaphore for send synchronization
  send_sema4 = xSemaphoreCreateBinary();

  // Create a binary semaphore for irq handling
  irq_sema4 = xSemaphoreCreateBinary();

  // Create a queue for receiving messages over the air
  msg_queue = xQueueCreate(NRF24_MSG_QUEUE_LENGTH, sizeof(nrf24_msg_t));

  // Create a lock for the send function
  send_lock = xSemaphoreCreateMutex();

  // Create the driver task
  xTaskCreate(nrf24_driver_task, "nrf24_driver", NRF24_DRIVER_STACK_SIZE,
              NULL, NRF24_DRIVER_PRIORITY, NULL);

  return 0;
}

static int nrf24_init(void) {
  // GPIO init
  gpio_init_pin(NRF24_CE_PORT, NRF24_CE_PIN, HACS_GPIO_MODE_OUTPUT_PP,
                HACS_GPIO_NO_PULL);

	// Initialize the RADIO_IRQ pin and enable IRQ
	gpio_init_pin(NRF24_IRQ_PORT, NRF24_IRQ_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
	gpio_exti_init(NRF24_IRQ_PORT, NRF24_IRQ_PIN, nrf24_irq_handler);
	gpio_exti_enable(NRF24_IRQ_PORT, NRF24_IRQ_PIN, 0, 1);

	return nrf24_radio_config();
}

static int nrf24_radio_config(void) {
  int retval;

  retval = nrf24_soft_reset();
  if (retval != HAL_OK) {
    return retval;
  }

  retval = nrf24_clear_irq(NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
  if (retval != HAL_OK) {
    return retval;
  }

  retval = nrf24_send_cmd(NRF24_COMMAND_FLUSH_TX);
  if (retval != HAL_OK) {
    return retval;
  }

  retval = nrf24_send_cmd(NRF24_COMMAND_FLUSH_RX);
  if (retval != HAL_OK) {
    return retval;
  }

  retval = nrf24_set_channel(40);
  if (retval != HAL_OK) {
    return retval;
  }

  // Set remote address
  retval = nrf24_set_transmit_addr(gnd_addr, sizeof(gnd_addr));
  if (retval != HAL_OK) {
    return retval;
  }

  // Configure Auto-retransmission delay and retry numbers
  retval = nrf24_write_reg(NRF24_REG_04_SETUP_RETR, 0xF3); // Important depending on ACK payload length
  if (retval != HAL_OK) {
    return retval;
  }

/* Not needed because we use ACK with payload
  // Set local address
  retval = nrf24_set_this_addr(this_addr, sizeof(this_addr));
  if (retval != HAL_OK) {
    return retval;
  }
*/

/* Not needed because we use DPL
  // Set static payload length
  retval = nrf24_set_payload_size(32);
  if (retval != HAL_OK) {
    return retval;
  }
*/

  // Enable Dynamic Payload Length in the feature register
  retval = nrf24_write_reg(NRF24_REG_1D_FEATURE, NRF24_EN_DPL | NRF24_EN_ACK_PAY);
  if (retval != HAL_OK) {
    return retval;
  }

  // Enable DPL on pipe 0 (which is the pipe we use to receive ACK)
  retval = nrf24_write_reg(NRF24_REG_1C_DYNPD, NRF24_DPL_P0);
  if (retval != HAL_OK) {
    return retval;
  }

  // Configure RF channel and power
  retval = nrf24_set_rf(NRF24DataRate250kbps, NRF24TransmitPower0dBm);

  // Enter Standby-II mode as we don't care about power consumption
  NRF24_CE_HIGH();

  return retval;
}

int nrf24_send(uint8_t *data, uint8_t len, uint8_t ack_cmd) {
  // Protect the send process. Only one instance of nrf24_send should be
  // invoked at one time
  xSemaphoreTake(send_lock, portMAX_DELAY);

  // Always clear the MAX_RT bit in status register
  nrf24_clear_irq(NRF24_MAX_RT);

  // Write the payload
  nrf24_burst_write(ack_cmd, data, len);
  
  // Grab the send semaphore to wait for send-complete (500ms timeout)
  xSemaphoreTake(send_sema4, MS_TO_TICKS(500));

  xSemaphoreGive(send_lock);

  return nrf24_status;
/*
  if (nrf24_status & NRF24_TX_DS) {
    return HACS_NO_ERROR;
  } else {
    return HACS_NRF24_TX_FAILED;
  }
  */
}

/*
 * Restore default configuration:
 *   IRQ reflects RX_DR, TX_DS and MAX_RT
 *   CRC enabled
 *   CRC 1 byte
 *   POWER DOWN mode
 *   PTX selected
 *
 * Return HAL_OK if no error.
 */
static int nrf24_soft_reset(void)
{
  NRF24_CE_LOW();
  delay_us(NRF24_CSN_INACTIVE_HOLD_US);
  return nrf24_write_reg(NRF24_REG_00_CONFIG, NRF24_BASE_CONFIGURATION);
}

/*
 * Clear any pending interrupt in the status register
 *
 * Return HAL_OK if no error.
 */
static int nrf24_clear_irq(uint8_t bits_to_clear)
{
  return nrf24_write_reg(NRF24_REG_07_STATUS, bits_to_clear);
}

/*
 * Set the RF channel 0-50
 * Freqeuncy == (2400 + chan) MHz
 *
 * Return HAL_OK if no error
 */
static int nrf24_set_channel(uint8_t chan)
{
  return nrf24_write_reg(NRF24_REG_05_RF_CH, chan & NRF24_RF_CH);
}

/*
 * Set the address of THIS radio
 *
 * Return HAL_OK if no error
 */
static int nrf24_set_this_addr(uint8_t *addr, uint8_t len)
{
  return nrf24_burst_write_reg(NRF24_REG_0B_RX_ADDR_P1, addr, len);
}

/*
 * Set transmit address (addr of the intended receiver)
 *
 * Return HAL_OK if no error
 */
static int nrf24_set_transmit_addr(uint8_t *addr, uint8_t len)
{
  int retval;

  // Set RX_ADDR_P0 to equal TX_ADDR for auto-ack with enhanced ShockBurst
  retval = nrf24_burst_write_reg(NRF24_REG_0A_RX_ADDR_P0, addr, len);
  if (retval != HACS_NO_ERROR) {
    return retval;
  }

  // Set TX_ADDR
  return nrf24_burst_write_reg(NRF24_REG_10_TX_ADDR, addr, len);
}

/*
 * Set the payload size 0 - 32 bytes
 *
 * Return HAL_OK if no error
 */
static int nrf24_set_payload_size(uint8_t size)
{
  int retval;

  retval = nrf24_write_reg(NRF24_REG_11_RX_PW_P0, size);
  if (retval != HACS_NO_ERROR) {
    return retval;
  }

  return nrf24_write_reg(NRF24_REG_12_RX_PW_P1, size);
}

/*
 * Set data rate and transmit power for the radio
 *
 * Return HAL_OK if no error
 */
static int nrf24_set_rf(NRF24DataRate dr, NRF24TransmitPower pwr)
{
  uint8_t val = ((uint8_t)pwr << 1) & NRF24_PWR;

  if (dr == NRF24DataRate250kbps) {
    val |= NRF24_RF_DR_LOW;
  } else if (dr == NRF24DataRate2Mbps) {
    val |= NRF24_RF_DR_HIGH;
  } else {
    return (-1); // invalid parameter
  }

  return nrf24_write_reg(NRF24_REG_06_RF_SETUP, val);
}

int nrf24_dump_registers(void) {
	const uint8_t registers[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                                0x1C, 0x1D };
  int i;
  int retval = 0;
  uint8_t val;

  for (i = 0; i < sizeof(registers); i++) {
    retval = nrf24_read_reg(registers[i], &val);
    if (retval != HAL_OK) {
      return retval;
    }

    printf("NRF24 REG %x: %x\n\r", registers[i], val);
  }

  return retval;
}

/*** Low level helper functions ***/

static int nrf24_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t cmd[] = { (reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER, 
                     val };
  int retval;

  NRF24_CSN_LOW();

  retval = spi_master_write(HACS_SPI_NRF24, cmd, sizeof(cmd));
  
  NRF24_CSN_HIGH();
  delay_us(NRF24_CSN_INACTIVE_HOLD_US);

  return retval;
}

static int nrf24_read_reg(uint8_t reg, uint8_t *val)
{
  return nrf24_burst_read((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_R_REGISTER, val, 1);
}

static int nrf24_burst_write(uint8_t cmd, uint8_t *data, uint8_t len)
{
  int retval;

  NRF24_CSN_LOW();

  retval = spi_master_write(HACS_SPI_NRF24, &cmd, sizeof(cmd));
  if (retval == HACS_NO_ERROR) {
    retval = spi_master_write(HACS_SPI_NRF24, data, len);
  }

  NRF24_CSN_HIGH();
  delay_us(NRF24_CSN_INACTIVE_HOLD_US);

  return retval;
}

static int nrf24_burst_write_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  return nrf24_burst_write((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER,
                           data, len);
}

static int nrf24_burst_read(uint8_t cmd, uint8_t *rbuf, uint8_t rlen)
{
  int retval;

  NRF24_CSN_LOW();

  retval = spi_master_transfer(HACS_SPI_NRF24, &cmd, sizeof(cmd), rbuf, rlen);

  NRF24_CSN_HIGH();
  delay_us(NRF24_CSN_INACTIVE_HOLD_US);

  return retval;
}

static int nrf24_send_cmd(uint8_t cmd)
{
  int retval;

  NRF24_CSN_LOW();

  retval = spi_master_write(HACS_SPI_NRF24, &cmd, sizeof(cmd));

  NRF24_CSN_HIGH();
  delay_us(NRF24_CSN_INACTIVE_HOLD_US);

  return retval;
}
