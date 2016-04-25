/*
 * enet_datalogger.h
 *
 *  Created on: Apr 23, 2016
 *      Author: Vivek
 */

#ifndef SRC_ENET_ENET_DATALOGGER_H_
#define SRC_ENET_ENET_DATALOGGER_H_

#include "utils/lwiplib.h"

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

extern uint8_t nw_update_timer;

extern void enet_init();
extern void server_connect();
extern void enet_metrics_log();

#endif /* SRC_ENET_ENET_DATALOGGER_H_ */
