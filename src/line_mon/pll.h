/*
 * pll.h
 *
 *  Created on: Apr 11, 2016
 *      Author: Vivek
 */

#ifndef SRC_LINE_MON_PLL_H_
#define SRC_LINE_MON_PLL_H_

#include "device.h"

/** Constants ************************************************************************************/
#define MAX_INCREMENTAL_CHANGE   50
#define SINE_SAMPLE_SIZE         512

/** function macros ******************************************************************************/
/*
 * PHASE CHECK
 * 1. Phase lead: Inverter in positive half cycle at capture interrupt
 * 2. Phase lag: Inverter in negative half cycle at capture interrupt
 */
#define PHASE_LEAD(A)           ((A>0) && (A<=SINE_SAMPLE_SIZE/2))
#define PHASE_LAG(A)            ((A>SINE_SAMPLE_SIZE/2) && (A < (SINE_SAMPLE_SIZE)))

typedef struct
{
    uint32_t cap_counts_prev;
    uint32_t cap_counts_now;
    uint32_t freq_in_cap_counts;
    uint16_t phase_shift_index;
    uint16_t sine_index;
    uint8_t capture_detected;
}pll_t;

extern pll_t pll_s;

extern void pll_init(pll_t *pll_s);
extern void phase_locked_loop(pll_t *pll);

#endif /* SRC_LINE_MON_PLL_H_ */
