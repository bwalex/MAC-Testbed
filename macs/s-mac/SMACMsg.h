/* Copyright (c) 2002 the University of Southern California.
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
 */
/* Defination of parameters and packet format for S-MAC
 * To be included by smac.c
 * If upper layer uses S-MAC it needs to include S-MAC header as the first
 * element in its own packet declaration.
 *
 * Author: Wei Ye
 *
 */

#ifndef SMAC_MSG
#define SMAC_MSG

// In Berkeley's stack, it is defined in AM.h
// Since we are not useing it, define it here.

// When using emtos, AM is included, so there's an enum-define collision
#ifndef PLATFORM_EMSTAR
#ifndef TOS_BCAST_DEFINED 
#ifndef AM_H_INCLUDED
#define TOS_BCAST_ADDR 0xffff
#endif
#endif
#endif

#ifndef PHY_MSG
#include "PhyRadioMsg.h"
#endif

// # of bytes going out
#define AIRED_DATA_SIZE (sizeof(PhyHeader) + PHY_MAX_PAYLOAD +2)



// MAC header to be included by upper layer headers -- nested headers
// type: higher 4 bits are type; lower 4 are sub-type
// seqFragNo: for broadcast packets, it's just sequence no; for unicast,
//            higher 5 bits are sequence no, lower 3 are fragment no.
typedef struct {
   PhyHeader phyHdr;
   char type;  // type is the first byte following phyHdr
   uint16_t toAddr;
   uint16_t fromAddr;
   uint16_t duration;
   uint8_t seqFragNo;
#ifdef SMAC_TX_TIME_STAMP
   uint32_t txTimeStamp;
#endif
} __attribute__((packed)) MACHeader;


/************************************************************** 
This is an example showing how an application that used S-MAC to
to define its packet structures.

App-layer header should include MAC_Header as its first field, e.g.,

typedef struct {
	MACHeader hdr;
	// now add app-layer header fields
	char appField1;
	int16_t appField2;
} AppHeader;

This is an nested header structure, as MACHeader includes PhyHeader
as its first field.

You can get the maximum payload length by the following macro.

#define MAX_APP_PAYLOAD (PHY_MAX_PKT_LEN - sizeof(AppHeader) - 2)

The app packet with maximum allowed length is then

typedef struct {
	AppHeader hdr;
	char data[MAX_APP_PAYLOAD];
	int16_t crc;  // must be last two bytes, required by PhyRadio.
} AppPkt;

******************************************************************/

// control packet -- RTS, CTS, ACK
typedef struct {
	PhyHeader phyHdr;  // include before my own stuff
	char type;  // type is the first byte following phyHdr
	uint16_t toAddr;
	uint16_t fromAddr;
	uint16_t duration;
	int16_t crc;  // must be last two bytes, required by PhyRadio
} __attribute__((packed)) MACCtrlPkt;

// sync packet
typedef struct {
	PhyHeader phyHdr;  // include before my own stuff
	char type;  // type is the first byte following phyHdr
	uint16_t fromAddr;
	char state;
	uint8_t seqNo;
#ifdef GLOBAL_SCHEDULE
        uint16_t syncNode;  // initializer of the schedule which I am following
#endif
	uint16_t sleepTime;  // my next sleep time from now
#ifdef SMAC_SNOOPER_DEBUG
    //uint8_t numSched;
    //uint8_t numNeighb;
    uint8_t numSchedNeighb;
    uint8_t txDataReset;
    uint8_t txSyncReset;
#endif
	int16_t crc;  // must be last two bytes, required by PhyRadio
} __attribute__((packed)) MACSyncPkt;

// for performance measurement
typedef struct {
	uint32_t sleepTime;
	uint32_t idleTime;
	uint32_t rxTime;
	uint32_t txTime;
} RadioTime;

#endif //SMAC_MSG
