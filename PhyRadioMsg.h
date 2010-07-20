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
/* Authors:	Wei Ye
 *
 * This file defines the header fields of phy_radio that will be
 * added before the payload of each packet.
 * The upper layer (MAC) that use phy_radio should include this header
 * as its first field and CRC as its last field in each packet it 
 * declares (see smac_msg.h for example).
 * 
 */

#ifndef PHY_MSG
#define PHY_MSG

// Maximum packet length -- including headers of all layers
// Each application can override the default max length in Makefile
// Maximum allowable value is 125
#ifndef PHY_MAX_PKT_LEN
#define PHY_MAX_PKT_LEN 125
#endif

// Physical-layer header to be put before data payload
typedef struct {
	uint8_t length; // length of entire packet
} __attribute__((packed)) PhyHeader;


// packet information to be recorded by physical layer
typedef struct {
	uint16_t strength;
	uint32_t timestamp;   // can be used w/ external counter of fine resolution
    uint32_t timeCoarse;  // S-MAC system time w/ resolution of 1 ms
} __attribute__((packed)) PhyPktInfo;

// Physical layer packet buffer (for receiving packets)
// Sending buffer should be provided by the top-level application
//
// MIN_PKT_LEN is the header size + the final 2 FCS bytes, which are
// filled by the CC2420 automagically
#define PHY_MIN_PKT_LEN (sizeof(PhyHeader) + 2)
#define PHY_MAX_PAYLOAD (PHY_MAX_PKT_LEN - PHY_MIN_PKT_LEN)

typedef struct {
	PhyHeader hdr;
	char data[PHY_MAX_PAYLOAD];
	int16_t crc;        // last field of a packet (this actually is FCS, not crc)
	PhyPktInfo info;  // not part of a packet
} __attribute__((packed)) PhyPktBuf;

#endif  // PHY_MSG
