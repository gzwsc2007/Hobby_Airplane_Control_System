#ifndef _ADS1120_H_
#define _ADS1120_H_

typedef enum {
  ADS1120_SINGLE_ENDED_CHAN_0,
  ADS1120_SINGLE_ENDED_CHAN_1,
  ADS1120_SINGLE_ENDED_CHAN_2,
  ADS1120_SINGLE_ENDED_CHAN_3,
} ads1120_chan_t;

typedef enum {
  ADS1120_REF_INTERNAL,
  ADS1120_REF_EXTERNAL,
  ADS1120_REF_RATIOMETRIC,
} ads1120_vref_t;

int ads1120_early_init();

/*
 * Read a single-ended channel, using internal/external/Vdd as reference.
 * If Vdd (Ratiometric) is selected, the range of the result is 0.0 ~ 1.0.
 * Otherwise, the result is in Volts.
 */
int ads1120_read_single_ended(ads1120_chan_t chan, ads1120_vref_t ref, float *p_result);

#endif
