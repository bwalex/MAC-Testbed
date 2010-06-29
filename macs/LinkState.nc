/*                                                                      tab:4
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
 * Authors:     Wei Ye
 * Date created: 11/12/2003
 *
 * This interface is for measuring link quality (state)
 *
 */


interface LinkState
{
   // signal if a new neighbor joins
   event void nodeJoin(uint16_t nodeAddr);
   
   // signal if a neighbor is gone/dead
   event void nodeGone(uint16_t nodeAddr);
   
   // signal the Rx of a SYNC packet with sequence number
   event void rxSyncPkt(uint16_t fromAddr, uint8_t seqNo);
   
   // signal the Rx of a broadcast data packet with sequence number
   event void rxBcastPkt(uint16_t fromAddr, uint8_t seqNo);
   
   // signal the Rx of a unicast packet with sequence number,
   // number of transmissions and number of copies received
   event void rxUcastPkt(uint16_t fromAddr, uint8_t seqNo, 
                        uint8_t numTx, uint8_t numRx);
}
