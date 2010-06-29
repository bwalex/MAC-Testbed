/*									tab:4
 * Copyright (c) 2002 the University of Southern California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN CALIFORNIA BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 * ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE
 * UNIVERSITY OF SOUTHERN CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * THE UNIVERSITY OF SOUTHERN CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF SOUTHERN CALIFORNIA HAS NO
 * OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
 * MODIFICATIONS.
 *
 * Authors:	Wei Ye
 *
 * This file defines debugging functions through UART
 */
 
#ifndef SMAC_UART_DEBUG_INCLUDED
#define SMAC_UART_DEUBG_INCLUDED

#include "smacEvents.h"

#if defined SMAC_UART_DEBUG_STATE_EVENT
// UART debug with predefined S-MAC states and events

#include "uartDebug.h"

static inline void smacUartDebug_init()
{
   uartDebug_init();
}

static inline void smacUartDebug_state(char byte)
{
   uartDebug_txByte(byte);
}

static inline void smacUartDebug_event(char byte)
{
   uartDebug_txByte(byte);
}

static inline void smacUartDebug_byte(char byte)
{
}

#elif defined SMAC_UART_DEBUG_BYTE
// Debug by sending a byte through UART

#include "uartDebug.h"

static inline void smacUartDebug_init()
{
   uartDebug_init();
}

static inline void smacUartDebug_state(char byte)
{
}

static inline void smacUartDebug_event(char byte)
{
}

static inline void smacUartDebug_byte(char byte)
{
   uartDebug_txByte(byte);
}

#else
// UART debug is not enabled

static inline void smacUartDebug_init()
{
}

static inline void smacUartDebug_state(char byte)
{
}

static inline void smacUartDebug_event(char byte)
{
}

static inline void smacUartDebug_byte(char byte)
{
}

#endif  // SMAC_UART_DEBUG

#endif  // SMAC_UART_DEBUG_INCLUDED
