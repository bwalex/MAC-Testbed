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
 * This file defines the packet format for SMACWrapper
 */

#ifndef SMAC_WRAPPER_MSG
#define SMAC_WRAPPER_MSG

// include TOS_Msg defination
#include "AM.h"

#ifdef USE_SMACSCEIVER
// use this if you want stand alone motes to talk to smacsceiver (moteNIC)

// include smacsceiver header defination
#include "AppPkt.h"

typedef AppHeader WrapHeader;

#else   // normal config

// define PHY_MAX_PKT_LEN before include SMACMsg.h. Otherwise default 
// value (100) will be used when SMACMsg.h includes PhyRadioMsg.h.
#define MAC_HEADER_LEN 9
#define PHY_MAX_PKT_LEN (MAC_HEADER_LEN + sizeof(TOS_Msg) + 2)

// include S-MAC header defination
#include "SMACMsg.h"

typedef MACHeader WrapHeader;

#endif

// msg to be sent on radio
typedef struct {
   WrapHeader wrapHdr;
   TOS_Msg tosMsg;
   int16_t crc;
} __attribute__ ((packed)) WrapMsg;

#endif
