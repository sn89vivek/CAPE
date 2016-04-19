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
#include "pll.h"
#include "measurements.h"
#include "fft_test.h"

#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/pushbutton.h"
#include "drivers/Kentec320x240x16_ssd2119_SPI.h"
#include "drivers/touch.h"

volatile uint8_t scheduler_flag;

//TODO: Frequency
//TODO: THD voltage, current, power
//TODO: power spectrum
//TODO: pin mapping excel
//TODO: A3 printouts

// Libraries
// 1. IQmath libraries
// 2. driverlib
// 3. dsplib-cm4f
// 4. grlib

extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui8Right24x23[];
extern const uint8_t g_pui8RightSmall15x14[];
extern const uint8_t g_pui8Left24x23[];
extern const uint8_t g_pui8LeftSmall15x14[];
uint32_t g_ui32SysClock;

extern tCanvasWidget g_sBackground;
extern tPushButtonWidget g_sPushBtn;
void OnNext(tWidget *pWidget);
void OnPrevious(tWidget *pWidget);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 296, 0, 23,
    23, PB_STYLE_IMG, ClrGoldenrod, ClrGoldenrod, 0, 0, 0, 0, g_pui8Right24x23,
    g_pui8RightSmall15x14, 0, 0, OnNext);

RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 0, 23,
    23, PB_STYLE_IMG, ClrGoldenrod, ClrGoldenrod, 0, 0, 0, 0, g_pui8Left24x23,
    g_pui8LeftSmall15x14, 0, 0, OnPrevious);

bool g_Led1On = false;
bool g_Led2On = false;

void OnNext(tWidget *pWidget)
{
  g_Led1On = !g_Led1On;
  if (g_Led1On)
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0xFF);
  }
  else
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
  }
}

void OnPrevious(tWidget *pWidget)
{
  g_Led2On = !g_Led2On;
  if (g_Led2On)
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0xFF);
  }
  else
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
  }
}

int main(void)
{
  tContext sContext;
  tRectangle sRect;

  //
  // Set the clocking to run directly from the crystal at 120MHz.
  //
  g_ui32SysClock = ROM_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
  SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
  SYSCTL_CFG_VCO_480), 120000000);

  FPUEnable();
  FPULazyStackingEnable();

  /* FFT test */
  generate_input();

  //
  // Initialize the display driver.
  //
  Kentec320x240x16_SSD2119Init();

  //
  // Initialize the graphics context.
  //
  GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

  /* Code for Image display */
//  ClrScreen();
  GrImageDraw(&sContext, g_pui8Image, 0, 0);
  GrFlush(&sContext);

//  GrPage1_inital_setup();
  //
  // Fill the top 24 rows of the screen with blue to create the banner.
  //
  sRect.i16XMin = 0;
  sRect.i16YMin = 0;
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  sRect.i16YMax = 23;
  GrContextForegroundSet(&sContext, ClrGoldenrod);
  GrRectFill(&sContext, &sRect);

  //
  // Put a white box around the banner.
  //
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectDraw(&sContext, &sRect);

  //
  // Put the application name in the middle of the banner.
  //
  GrContextFontSet(&sContext, &g_sFontCm20);
  GrStringDrawCentered(&sContext, "Voltage Spectrum", -1,
      GrContextDpyWidthGet(&sContext) / 2, 8, 0);

  TouchScreenInit(g_ui32SysClock);
  TouchScreenCallbackSet(WidgetPointerMessage);
  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sNext);
  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sPrevious);
  WidgetPaint(WIDGET_ROOT);

  /* Init all global stuff */
  pll_init(&pll_s);
  scheduler_flag = false;
  init_adc();

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
  // Enable the GPIO pin for the LED (PN0, PN1).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

  /* Systick period set to 30720Hz = (60Hz*512samples) */
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

  /* Configure the GPIO pin muxing to select PWM1 functions for these pins.
   * This step selects which alternate function is available for these pins.
   */
  ROM_GPIOPinConfigure(GPIO_PF1_M0PWM1);

  /* Configure the PWM functionality for the pin */
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

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
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);

  //
  // Enable the PWM0 output signal (PF1).
  //
  ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

  //
  // Enables the PWM generator block.
  //
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  /* T0CCP0 Initialisation (PL4-65 breadboard) */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  ROM_GPIOPinConfigure(GPIO_PL4_T0CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);

  /* Timer 0 Capture Initialisation */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TIMER0_CC_R = 0x00; /* Clocked with SYSCLK (120MHz) */
  TIMER0_CTL_R = 0; /* Timer disabled */
  TIMER0_CFG_R = TIMER_CFG_16_BIT; /* 16 bit configuration */
//  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER; /* 32 bit configuration */
  /* Up count, Capture mode, Edge Time, Capture mode, Capture mode interrupt enable */
  TIMER0_TAMR_R =
      TIMER_TAMR_TACDIR + TIMER_TAMR_TACMR + TIMER_TAMR_TAMR_CAP/* + TIMER_TAMR_TAPWMIE*/;
  TIMER0_CTL_R = TIMER_CTL_TAEVENT_NEG; /* Capture on falling edge */
  TIMER0_IMR_R = TIMER_IMR_CAEIM; /* Capture mode event interrupt enable */
  TIMER0_ICR_R = 0xFFFFFFFF; /* Clear interrupt status */
  TIMER0_TAILR_R = 0xFFFF; /* 2^16 is the upper bound */
  TIMER0_TAPR_R = 0xFF; /* 2^8 is upper bound */
  TIMER0_CTL_R |= TIMER_CTL_TAEN; /* Enable Timer */

  /* Enable Interrupts at NVIC level */
  ROM_IntEnable(INT_TIMER0A);

  /* enable interrupt processing at CPU level */
//  ROM_IntEnable(INT_ADC0SS3);
  ROM_IntMasterEnable();

  ROM_ADCProcessorTrigger(ADC0_BASE, 3);

  while (1)
  {
    /* Wait for conversion to be completed */
    if (ROM_ADCIntStatus(ADC0_BASE, 3, false))
    {
      ROM_ADCIntClear(ADC0_BASE, 3);
      ROM_ADCProcessorTrigger(ADC0_BASE, 3);
      TouchScreenIntHandler();
    }

    WidgetMessageQueueProcess();
    /* Execute scheduled events */
    if (scheduler_flag == true)
    {
      /* reset scheduler flag in beginning */
      scheduler_flag = false;

      /* PLL event */
      phase_locked_loop(&pll_s);

      /* Once very 5 cycles 80ms approx */
      if (fft_counter == 5)
      {
        fft_counter = 0;
        collect_fft_samples = false;
//        ROM_IntMasterDisable();
        fft_compute();
//        ROM_IntMasterEnable();
        collect_fft_samples = true;
      }
      else
      {
        fft_counter++;
      }
    }
  }
}

void sys_tick_handler()
{
  volatile int32_t radians;
  volatile int32_t sin;
  volatile uint16_t pwm_counts;

//  HWREG(GPIO_PORTN_BASE + (1 << 2)) = 1;

  /* Calculate radians */
  radians = _IQmpy(_IQ(PI)<<1,
      _IQ21toIQ(_IQ21div(_IQ21(pll_s.sine_index),_IQ21(SINE_SAMPLE_SIZE))));
  pll_s.sine_index = (pll_s.sine_index + 1) & (SINE_SAMPLE_SIZE - 1);

  /* trigger adc on even sine index */
  if ((pll_s.sine_index & 0x01) == 0)
  {
    ROM_ADCProcessorTrigger(ADC1_BASE, 0);
  }
  else
  {
    /* read result in the odd cycle */
    ROM_ADCSequenceDataGet(ADC1_BASE, 0, &ac_raw_adc_counts[0]);
    ROM_ADCIntClear(ADC1_BASE, 0);

    if (collect_fft_samples == true)
    {
      /* Build the fft samples array for current and voltage */
      norm_Vinst_IQ_samples[pll_s.sine_index / 2] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[0]) - _IQ(ADC_LEVEL_SHIFT);
      norm_Iinst_IQ_samples[pll_s.sine_index / 2] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[1]) - _IQ(ADC_LEVEL_SHIFT);
    }

    /* Square and accumulate adc channels (after removing offset) */
    ac_metrics.Vac.norm_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[0]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[0])-_IQ(ADC_LEVEL_SHIFT)));

    ac_metrics.Iac.norm_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[1]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT)));

    /* power calculation */
    /* accumulate instantaneous power  */
    ac_metrics.P_inst_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[0]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT)));
  }

  /* For PLL debug */
//  if (pll_s.sine_index == 0)
//  {
//    /* toggle port here */
//    HWREG(GPIO_PORTN_BASE + (1 << 2)) ^= 1;
//  }
  /* Compute Rms parameters once every cycle */
  if (pll_s.sine_index == SINE_SAMPLE_SIZE - 1)
  {
    /* voltage rms */
    ac_metrics.Vac.norm_rms = _IQsqrt(ac_metrics.Vac.norm_acc >> 8);
    ac_metrics.Vac.norm_acc = 0;

    /* current rms */
    ac_metrics.Iac.norm_rms = _IQsqrt(ac_metrics.Iac.norm_acc >> 8);
    ac_metrics.Iac.norm_acc = 0;

    /* Active power */
    ac_metrics.P_active = ac_metrics.P_inst_acc >> 8;
    ac_metrics.P_inst_acc = 0;

    /* TODO: Move out of ISR */
    /* Apparaent power */
    ac_metrics.P_apparent = _IQmpy(ac_metrics.Vac.norm_rms,
        ac_metrics.Iac.norm_rms);

    /*Calculate reactive power */
    ac_metrics.P_reactive =
        _IQsqrt(
            _IQmpy(ac_metrics.P_apparent,ac_metrics.P_apparent) -_IQmpy(ac_metrics.P_active,ac_metrics.P_active));

    /*Calculate Power Factor */
    ac_metrics.P_PowerFactor = _IQacos(
        _IQdiv(ac_metrics.P_active,ac_metrics.P_apparent));

    /* Scheduler Flag is set to true once a cycle  */
    scheduler_flag = true;
  }

  /* Calculate sin */
//  sin = (_IQsin(radians) + (_IQsin(_IQmpy(radians,_IQ(3.0)))>>1) + (_IQsin(_IQmpy(radians,_IQ(5.0)))>>2))>>1;
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
  pwm_counts = _IQmpy(sin, 6000);
  if (pwm_counts == 0)
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
  else
  {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm_counts);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
  }

//  HWREG(GPIO_PORTN_BASE + (1 << 2)) = 0;
}

void timer0_isr_handler()
{
  pll_s.capture_detected = true;
  pll_s.cap_counts_now = (TIMER0_TAR_R & 0xFFFFFF);
  pll_s.freq_in_cap_counts = (pll_s.cap_counts_now - pll_s.cap_counts_prev)
      & 0xFFFFFF;
  pll_s.cap_counts_prev = pll_s.cap_counts_now;

  /* Phase difference is the value of sine_index at zero crossing interrupt */
  pll_s.phase_shift_index = pll_s.sine_index;

  TIMER0_ICR_R = 4;
}

