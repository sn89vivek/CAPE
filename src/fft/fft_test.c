/*
 * fft_test.c
 *
 *  Created on: Apr 15, 2016
 *      Author: Vivek
 */


#include "fft_test.h"
#include <math.h>
#include "arm_const_structs.h"

float32_t sine_test_input[FFT_LENGTH];

uint8_t collect_fft_samples = 1;
uint8_t fft_counter = 0;
_iq norm_Iinst_IQ_samples[FFT_LENGTH];
_iq norm_Vinst_IQ_samples[FFT_LENGTH];
float32_t fft_output_array_voltage[FFT_LENGTH];
float32_t fft_output_array_current[FFT_LENGTH];

/* vars for this file */
//static float32_t testOutput[FFT_LENGTH];

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

void generate_input()
{
  uint16_t i;

  /* generate radians */
  for(i = 0; i < FFT_LENGTH; i++)
    sine_test_input[i] = i*2*PI/FFT_LENGTH;
  /* generate sine */
  for(i = 0; i < FFT_LENGTH; i++)
    sine_test_input[i] = 0.5*sin(sine_test_input[i])/*+sin(2*sine_test_input[i])*/;
}

void fft_test()
{
  //arm_status status;
  //float32_t maxValue;
  uint16_t i = 0;

  for(i = 0; i < FFT_LENGTH; i++)
  {
    //testOutput[i] = 0;
  }

  //status = ARM_MATH_SUCCESS;

//  /* Process the data through the CFFT/CIFFT module */
//  arm_rfft_fast_f32(&rfft_fast_len256, sine_test_input, testOutput, 0);
//
//  /* Process the data through the Complex Magnitude Module for
//  calculating the magnitude at each bin */
//  arm_cmplx_mag_f32(testOutput, testOutput, FFT_LENGTH/2);
}

void fft(arm_rfft_fast_instance_f32 *S, float32_t input_arr[], float32_t output_arr[])
{

  /* Process the data through the CFFT/CIFFT module */
  arm_rfft_fast_f32(S, input_arr, output_arr, 0);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(output_arr, output_arr, FFT_LENGTH/2);
}

void fft_compute()
{
  float32_t fft_input_array[FFT_LENGTH];
  uint16_t i;
  arm_rfft_fast_instance_f32 rfft_fast_len256;

  /* initialisation */
  arm_rfft_fast_init_f32(&rfft_fast_len256,FFT_LENGTH);


  /* voltage array */
  for(i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = norm_Vinst_IQ_samples[i]/16777216.0;
  fft(&rfft_fast_len256, fft_input_array,fft_output_array_voltage);

  /* current array */
  for(i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = _IQtoF(norm_Iinst_IQ_samples[i]);
  fft(&rfft_fast_len256, fft_input_array,fft_output_array_current);
}
