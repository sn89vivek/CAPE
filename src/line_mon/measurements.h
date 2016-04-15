/*
 * measurements.h
 *
 *  Created on: Apr 11, 2016
 *      Author: Vivek
 */

#ifndef SRC_LINE_MON_MEASUREMENTS_H_
#define SRC_LINE_MON_MEASUREMENTS_H_

#include "device.h"

typedef struct
{
    _iq norm_acc; /**< RMS accumulator */
    _iq norm_rms; /**< normalised rms value */
    _iq norm_offset; /**< Offset for rms calibration */
    _iq gain; /**< Gain for rms calibration */
} rms_struct_t;

typedef struct
{
    rms_struct_t Iac;
    rms_struct_t Vac;
    _iq P_apparent;
    _iq P_active;
    _iq P_inst_acc;
    _iq P_reactive;
} ac_metrics_t;

extern ac_metrics_t ac_metrics;
extern uint32_t ac_raw_adc_counts[2];

#define ADC_LEVEL_SHIFT   (0.5)
#define _Q12toIQ24(A)     ((int32_t)A<<12)

extern void init_adc();

#endif /* SRC_LINE_MON_MEASUREMENTS_H_ */
