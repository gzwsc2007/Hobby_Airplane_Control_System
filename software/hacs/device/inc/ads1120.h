#ifndef _ADS1120_H_
#define _ADS1120_H_

typedef enum {
  ADS1120_SINGLE_ENDED_CHAN_0,
  ADS1120_SINGLE_ENDED_CHAN_1,
  ADS1120_SINGLE_ENDED_CHAN_2,
  ADS1120_SINGLE_ENDED_CHAN_3,
} ads1120_chan_t;

int ads1120_early_init();
int ads1120_read_single_ended(ads1120_chan_t chan, uint16_t *p_result);

#endif
