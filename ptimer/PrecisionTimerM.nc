/*									tab:4
 *
 *
 * "Copyright (c) 2000-2002 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */
/*									tab:4
 *  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.  By
 *  downloading, copying, installing or using the software you agree to
 *  this license.  If you do not agree to this license, do not download,
 *  install, copy or use the software.
 *
 *  Intel Open Source License 
 *
 *  Copyright (c) 2002 Intel Corporation 
 *  All rights reserved. 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 * 
 *	Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *	Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *      Neither the name of the Intel Corporation nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *  PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE INTEL OR ITS
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 */
/* @author Lama Nachman, Jonathan Huang
*/
/* @author Alex Hornung <ahornung@gmail.com>
 */

includes trace;

module PrecisionTimerM {
  provides interface StdControl;
  provides interface PrecisionTimer[uint8_t id];
  uses interface PXA27XInterrupt as OSTIrq1;
  uses interface PXA27XInterrupt as OSTIrq2;
  uses interface PXA27XInterrupt as OSTIrq3;
}


implementation
{
   command result_t StdControl.init() {
	  call OSTIrq1.allocate();
     	  call OSTIrq2.allocate();
	  call OSTIrq3.allocate();
	  OSCR0 = 0x1;
      return SUCCESS;
   }

   command result_t StdControl.start() {
      return SUCCESS;
   }

   command result_t StdControl.stop() {
      return SUCCESS;
   }

   async command uint32_t PrecisionTimer.getTime32[uint8_t id]() {
  
      uint32_t time;
      atomic {
         time = OSCR0;
      }
      return time;
   }

   async command result_t PrecisionTimer.clearAlarm[uint8_t id]()
   {
		if ((id < 1) || (id > 3))
			return FAIL;

		atomic {
			switch (id) {
			case 1:
				OIER &= ~(OIER_E1);
				call OSTIrq1.disable();
				break;
			case 2:
				OIER &= ~(OIER_E2);
				call OSTIrq2.disable();
				break;
			case 3:
				OIER &= ~(OIER_E3);
				call OSTIrq3.disable();
				break;
			default:
				/* NOTREACHED */
			}
		}
		return SUCCESS;	
   }

   async command result_t PrecisionTimer.setAlarm[uint8_t id](uint32_t val)
   {
		if ((id < 1) || (id > 3))
			return FAIL;

		atomic {
			switch (id) {
			case 1:
				OSMR1 = val;
				OIER |= OIER_E1;
				call OSTIrq1.enable();
				break;
			case 2:
				OSMR2 = val;
				OIER |= OIER_E2;
				call OSTIrq2.enable();
				break;
			case 3:
				OSMR3 = val;
				OIER |= OIER_E3;
				call OSTIrq3.enable();
				break;
			default:
				/* NOTREACHED */
			}
		}
		return SUCCESS;
   }

   async event void OSTIrq1.fired() {
   		uint32_t val;
		if (OSSR & OIER_E1) {
			OSSR |= OIER_E1;
			OIER &= ~(OIER_E1);
			atomic { val = OSMR1; }
			signal PrecisionTimer.alarmFired[1](val);
			call OSTIrq1.disable();
		}
   }

   async event void OSTIrq2.fired() {
   		uint32_t val;
		if (OSSR & OIER_E2) {
			OSSR |= OIER_E2;
			OIER &= ~(OIER_E2);
			atomic { val = OSMR2; }
			signal PrecisionTimer.alarmFired[2](val);
			call OSTIrq2.disable();
		}
	}

   async event void OSTIrq3.fired() {
   		uint32_t val;
		if (OSSR & OIER_E3) {
			OSSR |= OIER_E3;
			OIER &= ~(OIER_E3);
			atomic { val = OSMR3; }
			signal PrecisionTimer.alarmFired[3](val);
			call OSTIrq3.disable();
		}
	}

   default async event result_t PrecisionTimer.alarmFired[uint8_t id](uint32_t val) {return SUCCESS;}
}
