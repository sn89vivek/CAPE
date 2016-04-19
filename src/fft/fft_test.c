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
float32_t fft_input_array[FFT_LENGTH];
arm_rfft_fast_instance_f32 rfft_fast_len256;

volatile double buff1[256];

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
    sine_test_input[i] = (sin(sine_test_input[i]) + sin(2*sine_test_input[i]))*0.5 + 1.0;
}

//void fft_test()
//{
//  //arm_status status;
//  //float32_t maxValue;
//  uint16_t i = 0;
//
//  for(i = 0; i < FFT_LENGTH; i++)
//  {
//    //testOutput[i] = 0;
//  }
//
//  //status = ARM_MATH_SUCCESS;
//
////  /* Process the data through the CFFT/CIFFT module */
////  arm_rfft_fast_f32(&rfft_fast_len256, sine_test_input, testOutput, 0);
////
////  /* Process the data through the Complex Magnitude Module for
////  calculating the magnitude at each bin */
////  arm_cmplx_mag_f32(testOutput, testOutput, FFT_LENGTH/2);
//}

void fft(arm_rfft_fast_instance_f32 *S, float32_t *input_arr, float32_t *output_arr)
{

  /* Process the data through the CFFT/CIFFT module */
  arm_rfft_fast_f32(S, input_arr, output_arr, 0);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(output_arr, output_arr, FFT_LENGTH/2);
  collect_fft_samples = 0;
}

void fft_compute()
{
  uint16_t i;

  /* initialisation */
  arm_rfft_fast_init_f32(&rfft_fast_len256,FFT_LENGTH);

  /* voltage array */
  for(i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = norm_Vinst_IQ_samples[i]/16777216.0;
  fft(&rfft_fast_len256, fft_input_array, fft_output_array_voltage);

  /* current array */
  for(i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = norm_Iinst_IQ_samples[i]/16777216.0;
  fft(&rfft_fast_len256, fft_input_array, fft_output_array_current);

//    buff1[i] = norm_Vinst_IQ_samples[i]/16777216.0;

//  fft1(256,8);
//  for(i = 0; i < FFT_LENGTH; i++)
//    fft_output_array_voltage[i] = buff1[i];

//  /* current array */
//  for(i = 0; i < FFT_LENGTH; i++)
//    fft_input_array[i] = _IQtoF(norm_Iinst_IQ_samples[i]);
//  fft(&rfft_fast_len256, fft_input_array,fft_output_array_current);
}

//void fft1(int n,int m)
//{
//
//  int i,j,k,n1,n2;
//  double c,s,e,a,t1,t2;
//  double y[256]={0.0};
//
//
//  j = 0; /* bit-reverse */
//  n2 = n/2;
//  for (i=1; i < n - 1; i++)
//  {
//      n1 = n2;
//      while ( j >= n1 )
//      {
//        j = j - n1;
//        n1 = n1/2;
//      }
//      j = j + n1;
//
//      if (i < j)
//      {
//        t1 = buff1[i];
//        buff1[i] = buff1[j];
//        buff1[j] = t1;
//        t1 = y[i];
//        y[i] = y[j];
//        y[j] = t1;
//      }
//  }
//
//
//  n1 = 0; /* FFT */
//  n2 = 1;
//
//  for (i=0; i < m; i++)
//  {
//      n1 = n2;
//      n2 = n2 + n2;
//      e = -6.283185307179586/n2;
//      a = 0.0;
//
//      for (j=0; j < n1; j++)
//      {
//        c = cos(a);
//        s = sin(a);
//        a = a + e;
//
//        for (k=j; k < n; k=k+n2)
//        {
//            t1 = c*buff1[k+n1] - s*y[k+n1];
//            t2 = s*buff1[k+n1] + c*y[k+n1];
//            buff1[k+n1] = buff1[k] - t1;
//            y[k+n1] = y[k] - t2;
//            buff1[k] = buff1[k] + t1;
//            y[k] = y[k] + t2;
//
//        }
//      }
//  }
//  for(k=0;k<256;k++)
//  {
//
//    buff1[k]=sqrt(buff1[k]*buff1[k]+y[k]*y[k]);
//  }
//
//}

