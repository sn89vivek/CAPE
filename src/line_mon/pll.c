/*
 * pll.c
 *
 *  Created on: Apr 11, 2016
 *      Author: Vivek
 */

#include "pll.h"

pll_t pll_s;

void pll_init(pll_t *pll_s)
{
  pll_s->cap_counts_now = 0;
  pll_s->cap_counts_prev = 0;
  pll_s->capture_detected = 0;
  pll_s->freq_in_cap_counts = 0;
  pll_s->phase_shift_index = 0;
}

void phase_locked_loop(pll_t *pll)
{
  int32_t systick_present, systick_next, systick_final, change;
  int32_t new_capture;
  uint16_t shift_index;

  /* Both next_tprd and present_tprd start with the present Timer register value */
  systick_next = systick_present = ROM_SysTickPeriodGet();
  shift_index = pll->phase_shift_index;

  if(pll->capture_detected == true)
  {
    /* Clear flag to allow next interrupts to set it  */
    pll->capture_detected = false;

    new_capture = pll->freq_in_cap_counts;
    systick_final = new_capture/SINE_SAMPLE_SIZE;

    /**
     * FREQUENCY SYNC
     * The change in tbprd allowed is restricted to MAX_INCREMENTAL_CHANGE_IN_COUNTS to
     * ensure a smooth transition to new frequency
     */
    change = _IQsat((systick_final - systick_present),MAX_INCREMENTAL_CHANGE, -MAX_INCREMENTAL_CHANGE);
    systick_next = systick_present + change;

    /** PHASE SYNCING **/
    if(PHASE_LEAD(shift_index))
      systick_next++;
    else if(PHASE_LAG(shift_index))
      systick_next--;
  }

  if(systick_next < 3606) // 65 Hz
    systick_next = 3606;
  else if(systick_next > 4261) // 55 Hz
    systick_next = 4261;
    ROM_SysTickPeriodSet(systick_next);
}



