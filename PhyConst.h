/*									tab:4
 * Copyright (c) 2003 the University of Southern California.
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
 * Date created: 5/2003
 *
/* Physical layer parameters for S-MAC
 *-------------------------------------
 * Based on the parameters from PHY_RADIO and RADIO_CONTROL
 * BANDWIDTH: bandwidth (bit rate) in kbps. Not directly used.
 * LISTEN_RATE: sampling rate of radio in listen mode (carrier sense)
 * PRE_PKT_BYTES: number of extra bytes transmitted before each pkt. It equals
 *   preamble + start symbol + sync bytes.
 * ENCODE_RATIO: output/input ratio of the number of bytes of the encoding
 *  scheme. In Manchester encoding, 1-byte input generates 2-byte output.
 * PROC_DELAY: processing delay of each packet in physical and MAC layer, in ms
 */

#ifndef PHY_CONST
#define PHY_CONST

#define BANDWIDTH 38
#define LISTEN_RATE 19
#define PRE_PKT_BYTES 5
#define ENCODE_RATIO 1
#define PROC_DELAY 2
#define TX_TRANSITION_TIME 1

#endif
