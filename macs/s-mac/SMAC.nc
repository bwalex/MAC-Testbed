/*
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
 * Date created: 1/21/2003
 * 
 * This module implements Sensor-MAC (S-MAC)
 * http://www.isi.edu/scadds/papers/smac_infocom.pdf
 *
 * It has the following functions.
 *  1) Low-duty-cycle operation on radio -- periodic listen and sleep
 *     Option to disable sleep cycles -- define SMAC_NO_SLEEP_CYCLE in Makefile
 *  2) Broadcast only uses CSMA
 *  3) Many features for unicast
 *     - RTS/CTS for hidden terminal problem
 *     - fragmentation support for a long message
 *       A long message is divided (by upper layer) into multiple fragments.
 *       The RTS/CTS reserves the medium for the entire message.
 *       ACK is used for each fragment for immediate error recovery.
 *     - Node goes to sleep when its neighbors are talking to other nodes.
 *
 * This configuration sets S-MAC to put time stamp on each received pkt
 *   using internal clock (1ms resolution). If fine-grained time stamp is 
 *   needed, provide an external implementation of TimeStamp interface to
 *   suply the time. To do it, change the wiring of ExtTimeStamp to your 
 *   own TimpStamp implementation. Please give the new configuration a new
 *   name.
 */

configuration SMAC
{
   provides {
      interface SplitControl;
      interface MACComm;
      interface LinkState;
      interface MACTest;
      interface MACPerformance;
      interface MACReport;
   }
  // uses {
  //    interface TimeStamp;
  // }
}

implementation
{
   components SMACM, PhyRadio, RandomLFSR, SingleTimer, HPLPowerManagementM;
   
   SplitControl = SMACM;
   MACComm = SMACM;
   LinkState = SMACM;
   MACTest = SMACM;
   MACPerformance = SMACM;
   MACReport = SMACM;
   //TimeStamp = SMACM;
   
   // wiring to lower layers
   
   SMACM.PhyControl -> PhyRadio;
   SMACM.PhyState -> PhyRadio;
   SMACM.CarrierSense -> PhyRadio;
   SMACM.PhyComm -> PhyRadio;
   SMACM.Random -> RandomLFSR;
   //SMACM.Clock -> ClockSMACM;
   SMACM.Timer -> SingleTimer.Timer;
   SMACM.PowerManagement -> HPLPowerManagementM;
}
