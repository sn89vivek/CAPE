/*
 * fft_test.h
 *
 *  Created on: Apr 15, 2016
 *      Author: Vivek
 */

#ifndef SRC_FFT_FFT_TEST_H_
#define SRC_FFT_FFT_TEST_H_

#include "device.h"
#include "arm_math.h"

#define FFT_LENGTH    256

extern float32_t sine_test_input[FFT_LENGTH];

extern uint8_t collect_fft_samples;
extern uint8_t fft_counter;
//extern _iq norm_Iinst_IQ_samples[FFT_LENGTH];
extern _iq norm_Vinst_IQ_samples[FFT_LENGTH];
extern float32_t fft_output_array_voltage[FFT_LENGTH];
//extern float32_t fft_output_array_current[FFT_LENGTH];

extern void fft_compute();
extern void generate_input();
//extern void fft_test();


#endif /* SRC_FFT_FFT_TEST_H_ */
