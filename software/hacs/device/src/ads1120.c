#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "hacs_spi_master.h"
#include "hacs_gpio.h"
#include "ads1120.h"

#define FULL_SCALE                    ((uint16_t)0x7FFF)

#define INTERNAL_REFERENCE_VOLTS      (2.048f)
#define EXTERNAL_REFERENCE_VOLTS      (4.089f)

#define COMMAND_RESET                 (0x06)
#define COMMAND_START                 (0x08)
#define COMMAND_POWERDOWN             (0x02)
#define COMMAND_RDATA                 (0x10)
#define COMMAND_RREG                  (0x20)
#define COMMAND_WREG                  (0x40)

#define REG_0                         (0x00)
#define REG_1                         (0x04)
#define REG_2                         (0x08)
#define REG_3                         (0x0C)

/*********** Configuration Register 0 ***********/
/* PGA bypass */
#define REG_0_PGA_BYPASS_MASK         (0x01)
#define REG_0_PGA_ENABLED             (0x00)
#define REG_0_PGA_DISABLED_AND_BYPASS (0x01)

/* PGA gain */
#define REG_0_GAIN_MASK               (0x0E)
#define REG_0_GAIN_1                  (0x00)
#define REG_0_GAIN_2                  (0x02)
#define REG_0_GAIN_4                  (0x04)
#define REG_0_GAIN_8                  (0x06)
#define REG_0_GAIN_16                 (0x08)
#define REG_0_GAIN_32                 (0x0A)
#define REG_0_GAIN_64                 (0x0C)
#define REG_0_GAIN_128                (0x0E)

/* Input channel selection */
#define REG_0_MUX_MASK                (0xF0)
#define REG_0_MUX_SINGLE_ENDED_0      (0x80)
#define REG_0_MUX_SINGLE_ENDED_1      (0x90)
#define REG_0_MUX_SINGLE_ENDED_2      (0xA0)
#define REG_0_MUX_SINGLE_ENDED_3      (0xB0)

/*********** Configuration Register 1 ***********/
/* Burn-out current sources */
#define REG_1_BCS_MASK                (0x01)
#define REG_1_BCS_OFF                 (0x00)
#define REG_1_BCS_ON                  (0x01)

/* Temperature sensor mode */
#define REG_1_TS_MASK                 (0x02)
#define REG_1_TS_DISABLED             (0x00)
#define REG_1_TS_ENABLED              (0x01)

/* Conversion mode */
#define REG_1_CM_MASK                 (0x04)
#define REG_1_CM_SINGLE_SHOT          (0x00)
#define REG_1_CM_CONTINUOUS           (0x04)

/* Operating mode */
#define REG_1_MODE_MASK               (0x18)
#define REG_1_MODE_NORMAL             (0x00)
#define REG_1_MODE_DUTY_CYCLE         (0x08)
#define REG_1_MODE_TURBO              (0x10)

/* Data rate */
#define REG_1_DR_MASK                 (0xE0) // Below are for Normal mode only
#define REG_1_DR_0                    (0x00) // 20 SPS
#define REG_1_DR_1                    (0x20) // 45 SPS
#define REG_1_DR_2                    (0x40) // 90 SPS
#define REG_1_DR_3                    (0x60) // 175 SPS
#define REG_1_DR_4                    (0x80) // 330 SPS
#define REG_1_DR_5                    (0xA0) // 600 SPS
#define REG_1_DR_6                    (0xC0) // 1000 SPS

/*********** Configuration Register 2 ***********/
/* Voltage reference selection (this is the only one I care about) */
#define REG_2_VREF_MASK               (0xC0)
#define REG_2_VREF_INTERNAL_2V048     (0x00)
#define REG_2_VREF_EXTERNAL_0         (0x40)
#define REG_2_VREF_EXTERNAL_1         (0x80)
#define REG_2_VREF_VDD                (0xC0)

/*********** Configuration Register 2 ***********/
/* Behavior of DOUT/DRDY_L pin */
#define REG_3_DRDYM_MASK              (0x02)
#define REG_3_DRDYM_NORMAL            (0x00)
#define REG_3_DRDYM_DUAL              (0x02)

static xSemaphoreHandle dev_lock;
static xSemaphoreHandle drdy_sem;

static void drdy_handler(void);
static int send_cmd(uint8_t cmd);
static int read_reg(uint8_t reg, uint8_t *pval);
static int write_reg(uint8_t reg, uint8_t val);
static int read_data(uint16_t *pdata);

int ads1120_early_init()
{
  int retval;

  // Device lock
  dev_lock = xSemaphoreCreateMutex();

  // DRDY semaphore
  drdy_sem = xSemaphoreCreateBinary();

  // Init DRDY pin
  gpio_init_pin(ADS1120_DRDY_PORT, ADS1120_DRDY_PIN, HACS_GPIO_MODE_INPUT, HACS_GPIO_NO_PULL);
  gpio_exti_init(ADS1120_DRDY_PORT, ADS1120_DRDY_PIN, drdy_handler);

  // Reset the device
  retval = send_cmd(COMMAND_RESET);
  HACS_REQUIRES(retval >= 0, done);
  delay_us(1000);

  // Configure Reg 0: Single-ended chan 0, Gain == 1, PGA disabled
  retval = write_reg(REG_0, REG_0_PGA_DISABLED_AND_BYPASS | REG_0_GAIN_1 | 
                            REG_0_MUX_SINGLE_ENDED_0);
  HACS_REQUIRES(retval >= 0, done);

  // Configure Reg 1: Single-shot, normal mode, 90 SPS
  retval = write_reg(REG_1, REG_1_BCS_OFF | REG_1_TS_DISABLED | REG_1_CM_SINGLE_SHOT | 
                            REG_1_MODE_NORMAL | REG_1_DR_2);
  HACS_REQUIRES(retval >= 0, done);

  // Configure Reg 2: External voltage reference on REF0
  retval = write_reg(REG_2, REG_2_VREF_EXTERNAL_0);
  HACS_REQUIRES(retval >= 0, done);

done:
  return retval;
}

/*
 * Read a single-ended channel, using internal/external/Vdd as reference.
 * If Vdd (Ratiometric) is selected, the range of the result is 0.0 ~ 1.0.
 * Otherwise, the result is in Volts.
 */
int ads1120_read_single_ended(ads1120_chan_t chan, ads1120_vref_t ref, float *p_result)
{
  uint8_t val;
  uint8_t mux;
  uint8_t vref;
  float vref_v;
  uint16_t raw;
  int retval;

  xSemaphoreTake(dev_lock, portMAX_DELAY);

  switch (chan) {
    case ADS1120_SINGLE_ENDED_CHAN_0: mux = REG_0_MUX_SINGLE_ENDED_0; break;
    case ADS1120_SINGLE_ENDED_CHAN_1: mux = REG_0_MUX_SINGLE_ENDED_1; break;
    case ADS1120_SINGLE_ENDED_CHAN_2: mux = REG_0_MUX_SINGLE_ENDED_2; break;
    case ADS1120_SINGLE_ENDED_CHAN_3: mux = REG_0_MUX_SINGLE_ENDED_3; break;
    default: break;
  }

  // Read Reg 0, and then set the appropriate channel
  retval = read_reg(REG_0, &val);
  HACS_REQUIRES(retval >= 0, done);
  retval = write_reg(REG_0, (val & ~REG_0_MUX_MASK) | mux);
  HACS_REQUIRES(retval >= 0, done);

  switch (ref) {
    case ADS1120_REF_INTERNAL:
      vref = REG_2_VREF_INTERNAL_2V048;
      vref_v = INTERNAL_REFERENCE_VOLTS;
      break;
    case ADS1120_REF_EXTERNAL:
      vref = REG_2_VREF_EXTERNAL_0;
      vref_v = EXTERNAL_REFERENCE_VOLTS;
      break;
    case ADS1120_REF_RATIOMETRIC:
      vref = REG_2_VREF_VDD;
      vref_v = 1.0; // Ratiometric - range 0.0 ~ 1.0
      break;
  }

  // Read Reg 2, and then set the appropriate voltage reference
  retval = read_reg(REG_2, &val);
  HACS_REQUIRES(retval >= 0, done);
  retval = write_reg(REG_2, (val & ~REG_2_VREF_MASK) | vref);
  HACS_REQUIRES(retval >= 0, done);

  // Enable external interrupt. To be triggered by falling edge
  gpio_exti_enable(ADS1120_DRDY_PORT, ADS1120_DRDY_PIN, 0, 1);

  // Send the START command
  retval = send_cmd(COMMAND_START);
  HACS_REQUIRES(retval >= 0, done);

  // Wait for conversion to finish. TODO: add timeout
  xSemaphoreTake(drdy_sem, portMAX_DELAY);

  // Read the 16-bit result
  retval = read_data(&raw);
  HACS_REQUIRES(retval >= 0, done);

  *p_result = (float)raw / (float)FULL_SCALE * vref_v;

done:
  xSemaphoreGive(dev_lock);
  return retval;
}

static void drdy_handler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(drdy_sem, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static int send_cmd(uint8_t cmd)
{
  int retval;

  spi_master_assert_cs(HACS_SPI_ADS1120);
  retval = spi_master_write(HACS_SPI_ADS1120, &cmd, sizeof(cmd));
  spi_master_deassert_cs(HACS_SPI_ADS1120);

  return retval;
}

static int read_reg(uint8_t reg, uint8_t *pval)
{
  int retval;

  spi_master_assert_cs(HACS_SPI_ADS1120);
  reg |= COMMAND_RREG;
  retval = spi_master_transfer(HACS_SPI_ADS1120, &reg, sizeof(reg), pval, 1);
  spi_master_deassert_cs(HACS_SPI_ADS1120);

  return retval;
}

static int write_reg(uint8_t reg, uint8_t val)
{
  int retval;
  uint8_t wbuf[2];

  spi_master_assert_cs(HACS_SPI_ADS1120);
  wbuf[0] = COMMAND_WREG | reg;
  wbuf[1] = val;
  retval = spi_master_write(HACS_SPI_ADS1120, wbuf, sizeof(wbuf));
  spi_master_deassert_cs(HACS_SPI_ADS1120);

  return retval;
}

static int read_data(uint16_t *pdata)
{
  int retval;
  uint8_t rbuf[2];

  spi_master_assert_cs(HACS_SPI_ADS1120);
  retval = spi_master_read(HACS_SPI_ADS1120, rbuf, sizeof(rbuf));
  spi_master_deassert_cs(HACS_SPI_ADS1120);

  *pdata = ((uint16_t)rbuf[0] << 8) | (uint16_t)rbuf[1];

  return retval;
}
