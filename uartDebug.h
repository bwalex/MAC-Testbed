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
 * UART debugging: this component is for sending debugging bytes thru UART
 *   Note: can't be used with any application that uses the UART, e.g. motenic
 *
 */

#ifndef UART_DEBUG
#define UART_DEBUG

#define DBG_BUF_LEN 50
#define ADVANCE(x) x = (((x+1) >= DBG_BUF_LEN) ? 0 : x+1)  // from TXMAN.c
#define UART_IDLE 0
#define UART_BUSY 1

// variables for UART debugging
char UARTState;
char dbgBuf[DBG_BUF_LEN];
uint8_t dbgHead;
uint8_t dbgTail;
uint8_t dbgBufCount;


void uartDebug_init()
{
   // there is a known problem that initializing UART may cause a node fail
   // to start, especially when it's not connected with a serial board/cable
   // the reason needs to be checked further
   UARTState = UART_IDLE;
   dbgBufCount = 0;
   dbgHead = 0;
   dbgTail = 0;
   // initialize UART
    // Set 57.6 KBps
    outp(0,UBRR0H); 
    outp(15, UBRR0L);

    // Set UART double speed
    outp((1<<U2X),UCSR0A);

    // Set frame format: 8 data-bits, 1 stop-bit
    outp(((1 << UCSZ1) | (1 << UCSZ0)) , UCSR0C);

    // Enable reciever and transmitter and their interrupts
    outp(((1 << RXCIE) | (1 << TXCIE) | (1 << RXEN) | (1 << TXEN)) ,UCSR0B);
   // suppose global interrupt is enabled
}


void uartDebug_txByte(char byte)
{
   char prev = inp(SREG) & 0x80;
   cli();
   if (UARTState == UART_IDLE) { // send byte if UART is idle 
      UARTState = UART_BUSY;
      if(prev) sei();
      // send byte to UART
      outp(byte, UDR0); 
      sbi(UCSR0A, TXC);
   } else {  // UART is busy, put byte into buffer
      // if buffer is full, the byte will be dropped silently
      if (dbgBufCount < DBG_BUF_LEN) {
         dbgBuf[dbgTail] = byte;
         ADVANCE(dbgTail);
         dbgBufCount++;
      }
      if(prev) sei();
   }
}


TOSH_INTERRUPT(SIG_UART0_TRANS)
{
   // UART is able to send next byte
   // This interrupt handler is using the INTERRUPT macro, in which 
   // the global interrupt is enabled, so the interrupt handler can
   // be interruptted too.
   char byte;
   char prev = inp(SREG) & 0x80;
   cli();
   if(dbgBufCount > 0) {
      byte = dbgBuf[dbgHead];
      ADVANCE(dbgHead);
      dbgBufCount--;
      if(prev) sei();
      // send next byte to UART
      outp(byte, UDR0); 
      sbi(UCSR0A, TXC);
   } else {
      UARTState = UART_IDLE;
      if(prev) sei();
   }
}


TOSH_INTERRUPT(SIG_UART0_RECV)
{
}

#endif  // UART_DEBUG

