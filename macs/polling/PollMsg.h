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
 * Authors:	Alex Hornung <ahornung@gmail.com>
 * Date created: 30/06/2010
 */

#ifndef POLL_MSG
#define POLL_MSG

#define POLL_BROADCAST_ID	255
#define POLL_BEACON	1
#define POLL_REQ		2
#define POLL_DATA		3
#define POLL_SCAN		4
#define POLL_ACK		5
#define POLL_PRESENT	6
#define POLL_SAMPLE 7

#include "PhyRadioMsg.h"

typedef struct {
   PhyHeader phyHdr;
   char type;
   uint16_t src_id;
   uint16_t dest_id;
} __attribute__((packed)) MACHeader;

typedef struct {
  MACHeader hdr;
  uint32_t sleep_jf;
  int16_t crc;
} __attribute__((packed)) MACPkt;

typedef struct {
	MACHeader hdr;   // include lower-layer header first
} AppHeader;

//#define APP_PAYLOAD_LEN (100 - sizeof(AppHeader) - 2)
#define APP_PAYLOAD_LEN   44

typedef struct {
	AppHeader hdr;
	char data[APP_PAYLOAD_LEN];
	int16_t crc;   // crc must be the last field -- required by PhyRadio
} AppPkt;

#endif //POLL_MSG
