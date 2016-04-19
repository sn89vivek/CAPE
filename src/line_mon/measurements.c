/*
 * measurements.c
 *
 *  Created on: Apr 11, 2016
 *      Author: Vivek
 */

#include "measurements.h"

ac_metrics_t ac_metrics;
uint32_t ac_raw_adc_counts[2];

void init_adc()
{
  /* SS0 triggers the following channels
   * PB4 (5) - AIN10 - V_ac_sense
   * PB5 (7) - AIN11 - I_ac_sense
   */

  /* Analog pins initialisation */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
  ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);

  /* Initialise ADC */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  /* Sequencer 0, Software trigger, highest priority */
  ROM_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  /* Step 0 of Sequencer 0, CH10 */
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);
  /* Step 1 of Sequencer 0, CH11, End_channel, Generate interrupt */
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 1,
      ADC_CTL_CH11 | ADC_CTL_IE | ADC_CTL_END);
  /* Enable sequencer */
  ROM_ADCSequenceEnable(ADC0_BASE, 0);

  /* ac_metrics initialisation */
  ac_metrics.Iac.gain = _IQ(1.0);
  ac_metrics.Iac.norm_offset = _IQ(0);
  ac_metrics.Iac.norm_acc = 0;
  ac_metrics.Iac.norm_rms = 0;
  ac_metrics.Vac.gain = _IQ(1.0);
  ac_metrics.Vac.norm_offset = _IQ(0);
  ac_metrics.Vac.norm_acc = 0;
  ac_metrics.Vac.norm_rms = 0;
  ac_metrics.P_active = 0;
  ac_metrics.P_inst_acc = 0;
  ac_metrics.P_apparent = 0;
  ac_metrics.P_reactive = 0;
  ac_metrics.P_PowerFactor = 0;
}

