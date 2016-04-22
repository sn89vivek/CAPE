/*
 * display_operations.h
 *
 *  Created on: Apr 21, 2016
 *      Author: Vivek
 */

#ifndef SRC_DISPLAY_DISPLAY_OPERATIONS_H_
#define SRC_DISPLAY_DISPLAY_OPERATIONS_H_

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

#define VOLTAGE_SPECTRUM  0
#define CURRENT_SPECTRUM  1

extern uint8_t g_uiSpectrum;

extern int8_t g_iPage;

/* gui pages */
#define PAGE_METRICS            0
#define PAGE_VOLTAGE_SPECTRUM   1
#define PAGE_CURRENT_SPECTRUM   2
#define PAGE_TIME_DOMAIN_SIGNAL 3

#define LAST_PAGE     3

/* Widgets */
extern tPushButtonWidget g_sPushBtn;
extern tCanvasWidget g_sACMetricsWidget;
extern tCanvasWidget g_sVrms, g_sVrmsValue, g_sIrms, g_sIrmsValue, g_sFreq,
    g_sFreqValue, g_sPF, g_sPFValue, g_sP_apparent, g_sP_apparent_val,
    g_sP_active, g_sP_active_val, g_sTHDv, g_sTHDv_val, g_sTHDi, g_sTHDi_val,
    g_sPhase_val, g_sPhase;

/* Images */
extern const uint8_t g_pui8Image[];
extern const uint8_t g_pui8Right24x23[];
extern const uint8_t g_pui8RightSmall15x14[];
extern const uint8_t g_pui8Left24x23[];
extern const uint8_t g_pui8LeftSmall15x14[];

/* gui context */
extern tContext sContext;
extern tRectangle sRect;

/* Global functions */
extern void Display_FreqSpectrum(uint8_t param);
extern void display_setup_page(int8_t page_no);
extern void gui_init();
extern void Display_TimeDomain(void);

#endif /* SRC_DISPLAY_DISPLAY_OPERATIONS_H_ */
