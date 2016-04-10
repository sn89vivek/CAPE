//*****************************************************************************
//
// main.c Main project file
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.2.111 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include "device.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

uint32_t sine_index; /* 0 - 511 */
int main(void)
{
  //
  // Set the clocking to run directly from the crystal at 120MHz.
  //
  ROM_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
  SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
  SYSCTL_CFG_VCO_480), 120000000);

  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

  //
  // Check if the peripheral access is enabled.
  //
  while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
  {
  }

  //
  // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

  /* Systick period set to 1KHz */
  ROM_SysTickPeriodSet(3906);

  /* Systick interrupt */
  ROM_SysTickIntEnable();

  /* Systick Enable */
  ROM_SysTickEnable();

  /* PWM initialisation */
  /* PWM0->PF0: 20KHz */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

  /* M0PWM0 --> PF0. Enable Port F */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* Configure the GPIO pin muxing to select PWM00 functions for these pins.
   * This step selects which alternate function is available for these pins.
   */
  ROM_GPIOPinConfigure(GPIO_PF0_M0PWM0);

  /* Configure the PWM functionality for the pin */
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);

  /* Configure PWM peripheral */
  /* PWM0 in count down mode */
  ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

  //
  // Set the PWM period to 20000Hz.  To calculate the appropriate parameter
  // use the following equation: N = (1 / f) * SysClk.  Where N is the
  // function parameter, f is the desired frequency, and SysClk is the
  // system clock frequency.
  // In this case you get: (1 / 20kHz) * 120MHz = 6000 cycles.  Note that
  // the maximum period you can set is 2^16.
  //
  ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 6000);

  //
  // Set duty to 20% = .2 * 6000 =
  //
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);

  //
  // Enable the PWM0 output signal (PD0).
  //
  ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

  //
  // Enables the PWM generator block.
  //
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  /* Initialise globals */
  sine_index = 0;

  /* enable interrupt processing at CPU level */
  ROM_IntMasterEnable();

  while (1)
  {
  }
}

volatile int32_t radians;
volatile int32_t sin;
volatile uint16_t pwm_counts;

#define SINE_SAMPLE_SIZE      512
#define PI                    3.14159265359
void sys_tick_handler()
{
  /* toggle port here */
  HWREG(GPIO_PORTN_BASE + (1 << 2)) ^= 1;

  /* Calculate radians */
  radians = _IQmpy(_IQ(PI)<<1,_IQ21toIQ(_IQ21div(_IQ21(sine_index),_IQ21(SINE_SAMPLE_SIZE))));
  sine_index = (sine_index+1)&(SINE_SAMPLE_SIZE-1);

  /* Calculate sin */
  sin = _IQsin(radians);

  /* Level shift sin */
  sin += _IQ(1);

  /* Sin is now from 0 to 2. Scale to 0 to 1 */
  sin >>= 1;

  /* Sin is scaled from 0 to 1 such that:
   * sin (0) = 0.5
   * sin (pi/2) = 1
   * sine (pi) = 0.5
   */
  /* drive duty */
  pwm_counts = _IQmpy(sin,6000);
  if(pwm_counts == 0)
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
  else
  {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm_counts);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
  }
}
