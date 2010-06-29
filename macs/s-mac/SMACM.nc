#/* ex: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

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
 * Authors:	Wei Ye, Honghui Chen
 *
 * This module implements Sensor-MAC (S-MAC)
 * http://www.isi.edu/scadds/papers/smac_infocom.pdf
 *
 * It has the following functions.
 *  1) Low-duty-cycle operation on radio -- periodic listen and sleep
 *     Option to disable sleep cycles
 *  2) Broadcast only uses CSMA
 *  3) Many features for unicast
 *     - RTS/CTS for hidden terminal problem
 *     - fragmentation support for a long message
 *       A long message is divided (by upper layer) into multiple fragments.
 *       The RTS/CTS reserves the medium for the entire message.
 *       ACK is used for each fragment for immediate error recovery.
 *     - Node goes to sleep when its neighbors are talking to other nodes.
 */

//includes uartDebug;

module SMACM
{
   provides {
      interface SplitControl as MACControl;
      interface MACComm;
      interface LinkState;
      interface MACTest;
      interface MACPerformance;
      interface MACReport;
   }
   uses {
      interface SplitControl as PhyControl;
      interface PhyState;
      interface CarrierSense;
      interface PhyComm;
      interface Random;
      //interface ClockSMAC as Clock;
      interface Timer;
      //interface TimeStamp;
      interface PowerManagement;
   }
}

implementation
{

#include "SMACMsg.h"
#include "PhyConst.h"
#include "SMACConst.h"

#include "smacUartDebug.h"

#ifdef SMAC_ERRCODES_ON
#include "errcodes.h"
#endif
//#include "smacEvents.h"

//#define SMAC_SNOOPER_DEBUG
//#define SMAC_PERFORMANCE


/* Internal S-MAC parameters
 * Do NOT change them unless for tuning S-MAC
 * User adjustable parameters are in SMACConst.h
 *--------------------------
 * SLOTTIME: time of each slot in contention window, in ms. It should be large
 *   enough to receive the whole start symbol.
 * DIFS: DCF interframe space (from 802.11), in ms. It is used at the beginning
 *   of each contention window. It's the minmum time to wait to start a new 
 *   transmission.
 * SIFS: short interframe space (from 802.11), in ms. It is used before sending
 *   an CTS or ACK packet. It takes care of the processing delay of each pkt.
 * SYNC_CW: number of slots in the sync contention window, must be 2^n - 1 
 * DATA_CW: number of slots in the data contention window, must be 2^n - 1
 * GUARDTIME: guard time at the end of each listen interval, in ms.
 * SYNC_PERIOD: period to send a sync pkt, in ms.
 * UPDATE_NEIGHB_PERIOD: period to update neighbor list, is n times of
 *    SYNC_PERIOD. It is used in low duty cycle mode. If there is no SYNC pkts
 *    from a node within this period, it will be removed from neighbor list.
 * 
 * DATA_ACTIVE_PERIOD (ms): This is only used in fully active mode to update 
 *    neighbor list. Since there is no SYNC pkts, data pkts are used to measure 
 *    if a neighbor is active recently.
 * TX_PKT_DONE_TIME: max time to wait for Tx done signal from PHY, in ms.
 */
#define DIFS 10
#define SIFS 5
#define EIFS 50
#define SLOTTIME 1
#define SYNC_CW 15
#define DATA_CW 31
#define GUARDTIME 8
#define SYNC_PERIOD 12000
#define UPDATE_NEIGHB_PERIOD 12
#define MAX_TX_SYNC_TIME 3
#define TX_PKT_DONE_TIME 2000
#define DATA_ACTIVE_PERIOD 300000


// hardware clock resolution in ms
#define CLOCK_RES 1

   /* MAC states
   *-------------
   * SLEEP: radio is turned off, can't Tx or Rx
   * IDLE: radio in idle mode, will go to Rx if start symbol is deteded.
   *   Can start Tx in this state
   * CARR_SENSE: carrier sense. Do it before initiate a Tx
   * TX_PKT: transmitting packet
   * BACKOFF - medium is busy, and cannot Tx
   * WAIT_CTS - just sent RTS, and is waiting for CTS
   * WAIT_DATA - just sent CTS, and is waiting for DATA
   * WAIT_ACK - just sent DATA, and is waiting for ACK
   * TX_NEXT_FRAG - just send one fragment, waiting to tx next fragment
   */
   enum {
      SLEEP,
      IDLE,
      CARR_SENSE,
      TX_PKT,
      BACKOFF,
      WAIT_CTS,
      WAIT_DATA,
      WAIT_ACK,
      TX_NEXT_FRAG,
      DATA_SENSE1,
      DATA_SENSE2
   };

   // radio states
   enum { RADIO_SLEEP, RADIO_IDLE, RADIO_RX, RADIO_TX };

   // how to send a pkt: broadcast or unicast
   enum { BCAST_DATA, SEND_SYNC, SEND_RTS, SEND_CTS, SEND_DATA, SEND_ACK };

   // MAC packet types
   enum { DATA_PKT, RTS_PKT, CTS_PKT, ACK_PKT, SYNC_PKT };

   // how to enter sleep mode
   enum { TRY, FORCE };
    uint8_t phy_reset;
    uint8_t phy_idle_call;
   // data type definitions
   // note: pkt formats are defined in tos/include/smac_msg.h
   typedef struct {
#ifdef GLOBAL_SCHEDULE
      uint16_t syncNode;  // the node who initialized this schedule
      uint32_t age; // schedule age
      uint32_t lastAgeUpdate; // last local time when updating the schedule age
#endif
      uint8_t numNodes;  // number of nodes on this schedule
      char txSync;   // flag indicating need to send sync
      char txData;   // flag indicating need to send data
      char chkSched; // flag indicating need to check numNodes
      uint16_t counter;  // tick counter
   } SchedTable;

   typedef struct {
      char state;
      uint16_t nodeId;
      uint8_t schedId;
      uint8_t active; //flag indicating the node is active recently
      uint8_t txSeqNo; // Tx sequence no of unicast packets to a node
      uint8_t rxSeqNo; // Rx sequence no of unicast packets from a node
   } NeighbList;

#ifdef FAST_PATH_SCHEDULE
   typedef struct {
      uint16_t pos; // indicating where to place the fast path schedule (relative to regular listen time)  
      uint16_t neighbPos; // indicating where to place the fast path schedule for the next hop node
      uint16_t counter;  
      uint16_t neighbCounter;    
      uint32_t timeToExpire; // indicating the time left for a fast path schedule to expire 
      uint8_t active;  // indicating if it is an active fast path schedule  
   } FastPathSchedTable;
#endif

   // state variables
   char state;			// MAC state
   char srchNeighb;    // flag to keep listening for finding possible neighbors
   char schedListen;   // flag indicating the time of scheduled listen
   char updateNeighbList;  // flag indicating need to update neighbor list
   uint8_t numNeighb;  // number of known neighbors
   NeighbList neighbList[SMAC_MAX_NUM_NEIGHB]; // neighbor list
   uint8_t numSched;  // number of different schedules
   SchedTable schedTab[SMAC_MAX_NUM_SCHED];   // schedule table
   char radioState;  // radio state
   char schedState;  // schedule state: first, second schedule...
   uint8_t resetPhyFlag; // flag to reset PHY in case radio is not working well
#ifdef FAST_PATH_SCHEDULE
   uint8_t fastPathListen;  // indicating the time of fast path schedules
   uint8_t numFastPathSched;  // number of different fast path schedules 
   FastPathSchedTable fastPathSchedTab[SMAC_MAX_NUM_FASTPATH_SCHED]; // fast path schedule table 
#endif   
   // timing variables
   uint32_t clockTime;   // clock time in mili-second
   uint16_t syncTime;    // time of listen/contention interval for sync pkt
   uint16_t dataTime;    // time of listen/contention interval for data pkt
   uint16_t listenTime;  // time of listening interval
   uint16_t sleepTime;   // time of sleeping interval 
   uint16_t period;      // period of my schedule =listenTime + sleepTime
   uint16_t timeToTxSync; // timer for sending SYNC packets to all schedules
   uint8_t numSyncPrd;   // number of SYNC peroids, for dependent timers
   uint16_t durDataPkt;  // duration (tx time needed) of data packet 
   uint16_t durCtrlPkt;  // duration (tx time needed) of control packet
   uint16_t durSyncPkt;  // duration (tx time needed) of sync packet
   uint16_t timeWaitCtrl; // time to wait for a control packet
   uint16_t nav;	     // network allocation vector. nav>0 -> medium busy
   uint16_t neighbNav;   // track neighbors' NAV while I'm sending/receiving
   uint16_t geneTime;    // generic timer
   uint8_t neighbListTime;  // timer for track nodes activity in neighbor list
   uint8_t txDelay;      // timer for carrier sense
   uint16_t adapTime;    // timer for adaptive listen
   uint8_t retryTime;    // number of retries in unicast
   uint8_t maxTxSyncTime;  // max time to send a sync to all schedules
   uint32_t maxTxDataTime; // max time to hold a data packet for Tx
   uint16_t txErrTime; // prevent radio stays in Tx state for too long
#ifdef SMAC_NO_SLEEP_CYCLE
   uint32_t dataActiveTime; // timer for updating neighbor list
#endif
   
   // Variables for Tx
   char txRequest;     // if I have accept a tx request;
   char howToSend;     // what action to take for tx
   uint16_t sendAddr;   // node that I'm sending data to
   uint8_t dataSched;  // current schedule I'm talking to
   uint8_t syncSched;  // current schedule I'm talking to
   uint8_t numRetry;   // number of RTS tries for a data pkt
   uint8_t numReTx;  // number of Tx time extensions when ACK timeout
   uint8_t numSyncTx;  // number of times to send a SYNC (multiple schedules)
   uint8_t numBcast;   // number of times to broadcast a pkt (multiple scheds)
   uint8_t txFragAll;  // number of fragments in this transmission
   uint8_t txFragCount; // number of transmitted fragments
   uint8_t txPktLen;   // length of data pkt to be transmitted
   uint8_t syncSeqNo;  // Tx sequence number for SYNC packets
   uint8_t bcastSeqNo; // Tx sequence number for broadcast data packets
   MACHeader* dataPkt; // pointer to tx data pkt, only access MAC header
   MACCtrlPkt ctrlPkt; // MAC control packet
   MACSyncPkt syncPkt; // MAC sync packet
   
   // Variables for Rx
   uint16_t recvAddr;  // node that I'm receiving data from
   uint8_t rxFragAll;  // number of fragments to be received in a msg
   uint8_t lastRxFrag; // fragment no of last received fragment

   // Variables for link state measurement
   uint8_t numTx1Msg;  // number of Tx on a msg, including reTx on frags
   uint8_t numRx1Msg;  // number of Rx on a msg, including duplicated frags
   uint8_t numTxFragOld; // number of Tx on a single fragment
   // Variables for global schedule
#ifdef GLOBAL_SCHEDULE
   uint32_t syncNode;  // the node who initialized this schedule
#endif

#ifdef SMAC_REPORT
    uint32_t crcErr;
    uint32_t lenErr;
    uint32_t retx;
    int32_t ctrlPkts;
    uint16_t slpErr;
    uint16_t macTxErr;
#endif

#ifdef SMAC_SNOOPER_DEBUG
   char numLenErr;
   char numCrcErr;
   char numSlpPkt;
   char numCTStimeout;
   char numDATAtimeout;
   char numACKtimeout;
   char numSleeps;
   char numWakeups;
   char syncDiff1;  // debug time difference in received SYNC pkt
   char syncDiff2;  // time difference bigger than GUARDTIME
   uint8_t numTxDataReset;
   uint8_t numTxSyncReset;
   uint8_t numRxSync;
   uint8_t numRxSyncErr;
   uint8_t setMySchedDone;
   uint8_t txSyncLed;
   uint8_t txSyncDoneLed;
   uint8_t numTxSyncCount;
#endif
#ifdef SMAC_PERFORMANCE
   char cntRadioTime;
   RadioTime radioTime;
#endif

// update my NAV
#define UPDATE_NAV(duration) { \
	if (nav < duration) nav = duration; \
}

// track my neighbors' NAV
#define TRACK_NAV(duration) { \
	if (neighbNav < duration) neighbNav = duration; \
}


   // function prototypes
   result_t getNodeIdx(uint16_t nodeAddr, uint8_t* nodeIdx, uint8_t* emptyIdx);
   void sleep(char manner);
   void wakeup();
   void setMySched(MACSyncPkt* packet, uint16_t refTime);
   void update_neighbList();
   void checkMySched();
   void check_schedFlag();
   void update_schedTab_neighbList();
   void handleRTS(void* packet);
   void handleCTS(void* packet);
   void* handleDATA(void* packet);
   void handleACK(void* packet);
   void handleSYNC(void* packet);
#ifdef GLOBAL_SCHEDULE
   void globalScheduleChange(uint8_t schedId);
#endif
#ifdef FAST_PATH_SCHEDULE
   void setFastPathSched(uint16_t pos); 
   uint16_t nextHopFastPathPos(uint16_t toAddr, uint16_t pos, uint16_t duration); 
#endif
   void startBcast();
   void sendRTS();
   void sendCTS();
   void sendDATA();
   void sendACK();
   void sendSYNC();
   void tryToSend();
   void tryToResend(uint8_t delay);
   void txMsgDone();


   command result_t MACControl.init()
   {
      uint8_t i;
      
      // initialize constants
      durCtrlPkt = (PRE_PKT_BYTES + sizeof(MACCtrlPkt) * ENCODE_RATIO)
                        * 8 / BANDWIDTH + 1;
      durSyncPkt = (PRE_PKT_BYTES + sizeof(MACSyncPkt) * ENCODE_RATIO)
                        * 8 / BANDWIDTH + 1;
      syncTime = DIFS + SLOTTIME * SYNC_CW + durSyncPkt + GUARDTIME;
      // added time for overhearing CTS so that can do adaptive listen
      dataTime = DIFS + SLOTTIME * DATA_CW + durCtrlPkt + 
                     PROC_DELAY + SIFS + durCtrlPkt + GUARDTIME;
      listenTime = syncTime + dataTime;
      period = listenTime * 100 / SMAC_DUTY_CYCLE + 1;
      sleepTime = period - listenTime;
      // time to wait for CTS or ACK
      timeWaitCtrl = PROC_DELAY + SIFS + durCtrlPkt + PROC_DELAY;

      // initialize state variables
      state = IDLE;
      radioState = RADIO_IDLE;

      // initialize neighbor list
      numNeighb = 0;  // number of known neighbors
      neighbListTime = 0;
      updateNeighbList = 0;
      for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
         neighbList[i].state = 0;  // invalid or dead
      }

      // initialize schedule table
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         schedTab[i].numNodes = 0;
#ifdef GLOBAL_SCHEDULE
         schedTab[i].syncNode = 0;
         schedTab[i].counter = 0;
         schedTab[i].age = 0;
         schedTab[i].lastAgeUpdate = 0;
#endif
      }

#ifdef FAST_PATH_SCHEDULE
      // initialize fast path schedule table
      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
         fastPathSchedTab[i].pos = 0;
         fastPathSchedTab[i].counter = 0;
      }

     fastPathListen = 0;
     numFastPathSched = 0;
#endif
	
#ifndef SMAC_NO_SLEEP_CYCLE
      // choose a tentative schedule, but don't broadcast until
      // listening for a whole SYNC_PERIOD.
      schedState = 1;  // this is my first schedule
      numSched = 1;
      schedTab[0].numNodes = 1;  // I'm the only one on this schedule
      schedTab[0].txData = 0;
      schedTab[0].txSync = 0;
      schedTab[0].chkSched = 0; 
      schedTab[0].counter = listenTime;
#ifdef GLOBAL_SCHEDULE
      schedTab[0].syncNode = TOS_LOCAL_ADDRESS;  // I'm the schedule initializer
      schedTab[0].age = 0;
      schedTab[0].lastAgeUpdate = 0;
#endif

      // Don't go to sleep for the first SYNC period.
      srchNeighb = 1;
      numSyncPrd = 1;
      schedListen = 1;
#ifdef SMAC_SLAVE_SCHED
      // don't broadcast my schedule, and keeps listening for an existing one
      timeToTxSync = 0;
#else
      timeToTxSync = SYNC_PERIOD; // set timer to broadcast my schedule
#endif // SMAC_SLAVE_SCHED
#endif // SMAC_NO_SLEEP_CYCLE

      // initialize timing variables
      clockTime = 0;
      nav = 0;
      neighbNav = 0;
      txDelay = 0;
      adapTime = 0;
#ifdef SMAC_NO_SLEEP_CYCLE
      retryTime = 0;
#endif
      maxTxSyncTime = 0;
      maxTxDataTime = 0;
	
      // initialize Tx variables
      txRequest = 0;
      syncSeqNo = 0;
      bcastSeqNo = 0;
      
      // fill in fixed portion of control packet
      ctrlPkt.fromAddr = TOS_LOCAL_ADDRESS;
      // fill in fixed portion of sync packet
      syncPkt.fromAddr = TOS_LOCAL_ADDRESS;
      syncPkt.type = SYNC_PKT << 4;
      syncPkt.state = schedState;

#ifdef SMAC_SNOOPER_DEBUG
      numLenErr = 0;
      numCrcErr = 0;
      numCTStimeout = 0;
      numDATAtimeout = 0;
      numACKtimeout = 0;
      numSleeps = 0;
      numWakeups = 0;
      syncDiff1 = 100;  // initialize to a big value to easily see its change
      syncDiff2 = 100;
      numTxDataReset = 0;
      numTxSyncReset = 0;
      numRxSync = 0;
      numRxSyncErr = 0;
      setMySchedDone = 0;
      txSyncLed = 0;
      txSyncDoneLed = 0;
      numTxSyncCount = 0;
#endif
#ifdef SMAC_PERFORMANCE
      cntRadioTime = 0;
#endif

#ifdef SMAC_REPORT
      lenErr=0;
      crcErr=0;
      retx=0;
      ctrlPkts=0;
      slpErr=0;
      macTxErr=0;
#endif // SMAC_REPORT
      
      // initialize UART debugging
      smacUartDebug_init();
      //smacUartDebug_state(state);
      //smacUartDebug_byte(durCtrlPkt);
      //smacUartDebug_byte(timeWaitCtrl);
      //smacUartDebug_byte(listenTime);

      // initialize random number generator
      call Random.init();
      // initialize and start clock
      //call Clock.start();
      call Timer.start(TIMER_REPEAT, 2); /* every 2ms, supposedly */
      //initialize physical layer
      trace(DBG_USR1, "s-mac calling PhyControl.init() (1)\r\n");
      call PhyControl.init();
      
      // Mica2 radio seems sensitive to power supply. With 3V DC power adapter
      // and the old programming board, the radio sometimes can't be correctly
      // initialized (most of time, it can receive but can't transmit). Use
      // this timer to reset radio after booting for sometime.
      resetPhyFlag = 0;
      
      return SUCCESS;
   }

   event result_t PhyControl.initDone()
   {
       uint8_t phyrst;

        atomic phyrst = phy_reset;
        trace(DBG_USR1, "PhyControl.initDone() called in SMACM\r\n");
        //if (phyrst)
            call PhyControl.start();

        signal MACControl.initDone();
        return SUCCESS;
   }

   event result_t PhyControl.startDone()
   {
       uint8_t phyidl;
       atomic {
          phy_reset = 0;
          phyidl = phy_idle_call;
       }
        trace(DBG_USR1, "PhyControl.startDone() called in SMACM\r\n");
       radioState = RADIO_IDLE;
      
       /* see #if 0ed section at phy_idle_call = 1 location */
       if (phyidl) {
           trace(DBG_USR1, "SMACM: PhyControl.startDone() called with phy_idle_call set\r\n");
           atomic phy_idle_call = 0;
           state = IDLE;
           smacUartDebug_state(state);
#ifdef SMAC_SNOOPER_DEBUG
           numWakeups++;
#endif
           signal MACTest.MACWakeup(); /* signal upper layer */
       }
       return SUCCESS;
   }

   event result_t PhyControl.stopDone()
   {
       uint8_t phyrst;

      atomic phyrst = phy_reset;
        trace(DBG_USR1, "PhyControl.stopDone() called in SMACM, phyrst = %d\r\n", phyrst);

       if (phyrst)
           call PhyControl.init();

       signal MACControl.stopDone();
       return SUCCESS;
   }

   command result_t MACControl.start()
   {
      // stopping and restarting is the same as a full reset of S-MAC
      call MACControl.init();
      signal MACControl.startDone();
      return SUCCESS;
   }


   command result_t MACControl.stop()
   {
      // stop clock and PHY, but S-MAC states are cleared when start again
      trace(DBG_USR1, "MACControl.stoP() called\r\n");
      call Timer.stop();  // stop clock
      call PhyControl.stop();  // stop physical layer
      return SUCCESS;
   }


   void resetPhy()
   {
      // this function resets physical layer, but does not change MAC
      trace(DBG_USR1, "resetPhy() called ...\r\n");
      smacUartDebug_event(PHY_RESET_CALLED);
      //atomic phy_reset = 1; /* stop will call init and start when this is set */
      //call PhyControl.stop();  // stop physical layer
      radioState = RADIO_IDLE;
   }


   result_t getNodeIdx(uint16_t nodeAddr, uint8_t* nodeIdx, uint8_t* emptyIdx)
   {
      // get the index of a node in the neighbor list
      // emptyIdx is the first empty entry in the neighbor list
      uint8_t i, foundEmpty;
      // return SMAC_MAX_NUM_NEIGHB if cannot find it
      *nodeIdx = SMAC_MAX_NUM_NEIGHB;
      *emptyIdx = SMAC_MAX_NUM_NEIGHB;
      foundEmpty = 0;
      for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
         if (neighbList[i].state > 0 &&
            neighbList[i].nodeId == nodeAddr) { // a known neighbor
            *nodeIdx = i;
            return TRUE;
         } else if (neighbList[i].state == 0 && foundEmpty == 0) {
            foundEmpty = 1;
            *emptyIdx = i;
         }
      }
      return FALSE;
   }


   command uint32_t MACPerformance.getTime()
   {
	  return clockTime;
   }


   command uint16_t MACPerformance.getPeriod()
   {
#ifdef SMAC_NO_SLEEP_CYCLE
      return 0;
#else
      return period;
#endif
   }


   command void* MACPerformance.countRadioTime(char start)
   {
#ifdef SMAC_PERFORMANCE
      // start/stop counting time of radio in different states
      if (start) {
         radioTime.sleepTime = 0;
         radioTime.idleTime = 0;
         radioTime.rxTime = 0;
         radioTime.txTime = 0;
         cntRadioTime = 1;
      } else {
         cntRadioTime = 0;
      }
      return &radioTime;
#else
      return 0;
#endif
   }
		

   void setMySched(MACSyncPkt* packet, uint16_t refTime)
   {
      // switch my schedule (first entry of schedule table)
      // now only happens when the first neighbor is found
      // follow the schedule in syncPkt
      schedTab[0].counter = refTime;  //packet->sleepTime;
#ifdef GLOBAL_SCHEDULE
      // update schedule age and record current time
      schedTab[0].age = packet->age + (packet->sleepTime - refTime);
      schedTab[0].lastAgeUpdate = clockTime;
      schedTab[0].syncNode = packet->syncNode;  // initializer of my current schedule
#endif
      schedTab[0].txSync = 1;  // need to broadcast my schedule
      schedTab[0].numNodes++;  // 2 nodes on this schedule now
      schedState++;
      numSyncTx = 1;  // only have one schedule to send SYNC
      // fill in the field in my syncPkt
      syncPkt.state = schedState;
      // add my first neighbor
      neighbList[0].state = packet->state;
      neighbList[0].nodeId = packet->fromAddr;
      neighbList[0].schedId = 0;
      neighbList[0].active = 1;
      neighbList[0].txSeqNo = 0;
      neighbList[0].rxSeqNo = SMAC_MAX_UCAST_SEQ_NO;
      numNeighb = 1;
      //start setting timer for update neighbor list
      neighbListTime = UPDATE_NEIGHB_PERIOD; 
   }

#ifdef FAST_PATH_SCHEDULE
   void setFastPathSched(uint16_t pos)
   {
      uint8_t schedId,i;
      // check if a fast path schedue has already been set up at the position

      schedId = SMAC_MAX_NUM_FASTPATH_SCHED;
      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
         if (fastPathSchedTab[i].active == 1) {
            if (fastPathSchedTab[i].pos == pos || 
                fastPathSchedTab[i].neighbPos == pos) {
               schedId = i;
               // existing fast path schedule, update expiration timer  
               if (fastPathSchedTab[i].pos == pos) {
                  fastPathSchedTab[i].timeToExpire = SMAC_FAST_PATH_EXPIRE;        
               }  
               break;
            }
         }
      }

      if (schedId == SMAC_MAX_NUM_SCHED) {  // unknown fast path schedule
        // add an entry to the fast path schedule table
         if (numFastPathSched < SMAC_MAX_NUM_FASTPATH_SCHED){
            for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
              if (fastPathSchedTab[i].active == 0) { // found an empty entry
                 // schedule the extra wake up period
                 fastPathSchedTab[i].pos = pos;
                 fastPathSchedTab[i].active = 1;
                 fastPathSchedTab[i].timeToExpire = SMAC_FAST_PATH_EXPIRE;
                 numFastPathSched++;  // increment number of fast path schedules
                 break;
              }
            }
         }
     }
   }  
   uint16_t nextHopFastPathPos(uint16_t toAddr, uint16_t pos, uint16_t duration) 
   {
      // compute next hop fast path schedule slot position
      uint8_t i, offset, schedId, neighbPos;

      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
         if (fastPathSchedTab[i].active == 1) {
            if (fastPathSchedTab[i].pos == pos) {
               schedId = i;
               break;
            }
         }
      }

      // compute next hop fast path schedule slot position, assume global schedule is used
      // do not overlap with the regular listen time
      // offset = Tcs+Ttx  
      offset = DIFS + SLOTTIME * DATA_CW + durCtrlPkt + PROC_DELAY + 
        SIFS + durCtrlPkt + GUARDTIME + PROC_DELAY + SIFS + 
        duration + PROC_DELAY + SIFS + durCtrlPkt + PROC_DELAY + SIFS;

      if (pos == 0) { // I do not set up the fast path schedule 
         neighbPos = syncTime + offset;  
      }
      else {   
         neighbPos = pos + offset;
      }
      if (neighbPos >= period) {
         neighbPos = neighbPos - period;
      }
      // position relative to next regular sleep time
      neighbPos = period + listenTime - neighbPos;
      // next hop's fast path schedule overlaps with its regular listen time  
      if (neighbPos < offset + listenTime) neighbPos = 0;   
      else if (neighbPos > period - offset + dataTime) {    // this should not happen
      };
      fastPathSchedTab[schedId].neighbPos = neighbPos;
   } 
#endif

   void checkMySched()
   {
      // check if I am the only one on schedTab[0]
      // if yes, should switch and follow the next available schedule
      // happens when an old node switches to a new schedule 
      // and when I drop some inactive nodes from neighbor list(updating)
      uint8_t i, schedId;
      schedId = 0;
      if (schedTab[0].numNodes == 1 && numSched > 1 && numNeighb > 0) {
         for (i = 1; i < SMAC_MAX_NUM_SCHED; i++) {
            if (schedTab[i].numNodes > 0) {  // switch to next schedule
               schedTab[0].counter = schedTab[i].counter;
#ifdef GLOBAL_SCHEDULE
               schedTab[0].syncNode = schedTab[i].syncNode;
               schedTab[0].age = schedTab[i].age;
               schedTab[0].lastAgeUpdate = schedTab[i].lastAgeUpdate; 
#endif
               schedTab[0].txSync = 1;
               schedTab[0].txData = schedTab[i].txData;
               schedTab[0].numNodes = schedTab[i].numNodes + 1;
               // delete this schedule          
               schedTab[i].numNodes = 0;
               numSched--;
               schedId = i;
               break;
            }
         }
         if (schedId > 0){
            schedState++;
            // fill in the field in my syncPkt
            syncPkt.state = schedState;
            // update my neighbor list which relative to this schedId
            for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
               if (neighbList[i].state > 0 )
                  if (neighbList[i].schedId == schedId)
                     neighbList[i].schedId = 0;
            }
         }
      }
   }


   task void update_myNeighbList()
   {
#ifdef SMAC_NO_SLEEP_CYCLE
      // in fully active mode, directly update neighbor list
      uint8_t i;
      smacUartDebug_event(TIMER_FIRE_DATA_ACTIVE);
      for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
         if (neighbList[i].state > 0 ){
            if (neighbList[i].active != 1){ // this node is not active recently
               neighbList[i].state = 0;
               numNeighb--;
               signal LinkState.nodeGone(neighbList[i].nodeId);
            } else {
               neighbList[i].active = 0;
            }
         }
      }
      if (numNeighb == 0) {
         dataActiveTime = 0; // clear neighbor list update timer
         smacUartDebug_event(NUM_NEIGHB_BECOMES_0);
      }
#else
      // in low duty cycle mode, avoid messing up data Tx
      //char intEnabled = inp(SREG) & 0x80;
      //cli();
      if (txRequest == 0) { // No data waiting to be transmitted
         txRequest = 1; // temporarily disable tx when updating
         //if (intEnabled) sei();
         //we should update the schedTab[].numNodes before checkMySched()
         //to ensure the next available schedule is correct
         check_schedFlag();
         update_neighbList();
         updateNeighbList = 0;
         schedTab[0].chkSched = 0; // did checkMySched() in update_neighbList()
         txRequest = 0; // re-enable Tx
      } else {
         //if (intEnabled) sei();
         updateNeighbList = 1; // set flag to update when tx done
      }
#endif
   }
   
   
   void update_neighbList()
   {
      // update neighbor list, 
      // if the node is not active (moved away or died) for a certain time, 
      // need to drop it from neighbor list
      uint8_t i, schedId;
      for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
         if (neighbList[i].state > 0 ){
            if (neighbList[i].active != 1){ // this node is not active recently
               schedId = neighbList[i].schedId;
               schedTab[schedId].numNodes--;
               if (schedTab[schedId].numNodes == 0)
                  numSched--;
               neighbList[i].state = 0;
               numNeighb--;
               signal LinkState.nodeGone(neighbList[i].nodeId);
            }else 
               neighbList[i].active = 0;
         }
      }
      // maybe the inactive nodes were dropped from schedTab[0]
      // check if I am the only one on schedTab[0] 
      // if yes, I should follow the next available schedule
      checkMySched();	  
      if (numNeighb == 0) {
         neighbListTime = 0; // clear neighbor list update timer
      }
   }


   void check_schedFlag()
   {
      uint8_t i;
      // decrease the numNodes in the old schedule first
      for (i = 1; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0 && schedTab[i].chkSched == 1){
            schedTab[i].chkSched = 0;
            schedTab[i].numNodes--;
            if (schedTab[i].numNodes == 0)
               numSched--;	
         }         
      }
   }
   
   
   task void txSyncTimerFire()
   {
      // need to send a sync
      uint8_t i;
      smacUartDebug_event(TIMER_FIRE_NEED_TX_SYNC);
      if (numSyncTx > 0) { // previous SYNC Tx is not done
         maxTxSyncTime++;
         if (maxTxSyncTime == MAX_TX_SYNC_TIME) {
            resetPhyFlag = 1; // radio may not work properly
            maxTxSyncTime = 0;
            smacUartDebug_event(SYNC_BLOCKED);
#ifdef SMAC_SNOOPER_DEBUG
            numTxSyncReset++;
#endif
         }
      }
      // set flag in each schedule to tx SYNC
      numSyncTx = numSched;
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) {
            schedTab[i].txSync = 1;
         }
      }
      
      // neighbor discovery timer depends on tx sync timer
      numSyncPrd--;  // for neighbor discovery
      if (numSyncPrd == 1) {
         srchNeighb = 1;
      } else if (numSyncPrd == 0) {
         srchNeighb = 0;
         if (numNeighb == 0) { // reset neighbor discovery timer
            numSyncPrd = SMAC_SRCH_NBR_SHORT_PERIOD;
         } else {
            numSyncPrd = SMAC_SRCH_NBR_LONG_PERIOD;
         }
      }
   }

   
   
   void sleep(char manner)
   {
      // try or force to enter sleep mode
      // if manner == FORCE, turn off radio immediately
      // if manner == TRY, check other status. May stay in idle.
      if (state == SLEEP) return;
#ifdef FAST_PATH_SCHEDULE
      if (manner == FORCE || (radioState == RADIO_IDLE &&
         srchNeighb == 0 && schedListen == 0 && fastPathListen == 0)) {
#else
      if (manner == FORCE || (radioState == RADIO_IDLE && 
         srchNeighb == 0 && schedListen == 0)) {
#endif
        /* XXX: */
         //call PhyState.sleep();  // turn off the radio

         radioState = RADIO_SLEEP;
         state = SLEEP;
#ifdef SMAC_SNOOPER_DEBUG
         numSleeps++;
#endif
         signal MACTest.MACSleep();  // signal upper layer
      } else {
         state = IDLE;
      }
      smacUartDebug_state(state);
   }


   void wakeup()
   {
      // wake up from sleep, turn on radio and signal upper layer
      if (state != SLEEP) return;

      atomic phy_idle_call = 1;
      call PhyState.idle();  // turn on the radio
#if 0
      /* XXX: this needs work... */
      //radioState = RADIO_IDLE;
      state = IDLE;
      smacUartDebug_state(state);
#ifdef SMAC_SNOOPER_DEBUG
      numWakeups++;
#endif
      /* XXX: possibly move this to the idle callback, i.e. startDone() */
      signal MACTest.MACWakeup();  // signal upper layer
#endif
   }


   void tryToSend()
   {
      // try to send a buffered packet
      uint16_t backoffSlots, listenBits;
      char intEnabled;
      if (state == IDLE && nav == 0 && neighbNav == 0) {
         if (sendAddr == TOS_BCAST_ADDR) {
            howToSend = BCAST_DATA;
         } else {
            howToSend = SEND_RTS;
         }
         backoffSlots = call Random.rand() & (uint16_t)DATA_CW;
         listenBits = (DIFS + SLOTTIME * backoffSlots) * LISTEN_RATE;
         // start carrier sense and change state needs to be atomic
         // to prevent start symbol is detected between them
         //intEnabled = inp(SREG) & 0x80;
         //cli();
         state = CARR_SENSE;
         if (call CarrierSense.start(/*listenBits*/) == SUCCESS) {
            trace(DBG_USR1, "CarrierSense.start() called successfully\r\n");
         }
         //if(intEnabled) sei();
         smacUartDebug_state(state);
      } else {
         if (state != IDLE) {
            smacUartDebug_event(TRYTOSEND_FAIL_NOT_IDLE);
         }
         if (nav != 0) {
            smacUartDebug_event(TRYTOSEND_FAIL_NAV);
         }
         if (neighbNav != 0) {
            smacUartDebug_event(TRYTOSEND_FAIL_NEIGHBNAV);
         }
      }
      return;
   }
		

   command result_t MACComm.broadcastMsg(void* data, uint8_t length)
   {
      char intEnabled;
#ifndef SMAC_NO_SLEEP_CYCLE
      uint8_t i;
      if (numNeighb == 0) {
          trace(DBG_USR1, "no neighbours found, failing\r\n");
#ifndef SMAC_ERRCODES_ON
          return FAIL;
#else
          return -ENEIGHB;
#endif
      } 
#endif
      // Don't accept Tx request if I have already accepted a request
      if (data == 0 || length == 0 || length > PHY_MAX_PKT_LEN) {
         if (data == 0) {
            smacUartDebug_event(SMAC_BCAST_REQUEST_REJECTED_DATA_IS_0);
#ifdef SMAC_ERRCODES_ON
            return -EDATA;
#endif
            trace(DBG_USR1, "data == 0 in broadcastMsg()\r\n");
         }
         if (length == 0 || length > PHY_MAX_PKT_LEN) {
            trace(DBG_USR1, "length = %d (fail, fail, fail) vs max_pkt =%d\r\n", length, PHY_MAX_PKT_LEN);
            smacUartDebug_event(SMAC_BCAST_REQUEST_REJECTED_PKTLEN_ERROR);
#ifdef SMAC_ERRCODES_ON
            return -ELENGTH;
#endif
         }
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#endif
      }
      trace(DBG_USR1, "MACComm.broadcastMsg() called\r\n");
      // disable interrupt when check the value of txRequest
      //intEnabled = inp(SREG) & 0x80;
      //cli();
      if (txRequest == 0) {
         txRequest = 1;
         //if (intEnabled) sei();
      } else {     
         //if (intEnabled) sei();
         smacUartDebug_event(SMAC_BCAST_REQUEST_REJECTED_TXREQUEST_IS_1);
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#else
         return -ETXBUSY;
#endif
      }
      dataPkt = (MACHeader*)data;
      txPktLen = length;
      sendAddr = TOS_BCAST_ADDR;
      // fill in MAC header fields
      dataPkt->type = DATA_PKT << 4;  // higher 4 bits
      dataPkt->toAddr = TOS_BCAST_ADDR;
      dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
      dataPkt->duration = 0;
      dataPkt->seqFragNo = bcastSeqNo;
      bcastSeqNo++;
#ifdef SMAC_NO_SLEEP_CYCLE
      // try to send now
      tryToSend();
#else
      // set flag in each schedule
      numBcast = numSched;
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) {
            schedTab[i].txData = 1;
         }
      }
#endif
      maxTxDataTime = SMAC_MAX_TX_MSG_TIME;
      return SUCCESS;
   }

#ifdef FAST_PATH_SCHEDULE
   command result_t MACComm.unicastMsgFastPath(void* data, uint8_t length, 
        uint16_t toAddr, uint8_t numFrags, 
        uint8_t fastPathFlag, uint8_t isSource, uint16_t fastPathPos)
   {
      uint8_t nodeIdx, emptyIdx;
#ifndef SMAC_NO_SLEEP_CYCLE
      uint8_t schedId;
#endif
      uint8_t d;
      char intEnabled;
      // sanity check
      if (data == 0 || length == 0 ||
         length > PHY_MAX_PKT_LEN || numFrags == 0 || numFrags > 8) {
         if (data == 0) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_DATA_IS_0);
#ifdef SMAC_ERRCODES_ON
            return -EDATA;
#endif  // SMAC_ERRCODES_ON
         }
         if (length == 0 || length > PHY_MAX_PKT_LEN) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_PKTLEN_ERROR);
#ifdef SMAC_ERRCODES_ON
            return -ELENGTH;
#endif  // ERRCODES
         }
         if (numFrags == 0) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_NUMFRAGS_IS_0);
#ifdef SMAC_ERRCODES_ON
            return -EFRAGS;
#endif
         }
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#endif
      }
      getNodeIdx(toAddr, &nodeIdx, &emptyIdx);
#ifndef SMAC_NO_SLEEP_CYCLE
      if (nodeIdx == SMAC_MAX_NUM_NEIGHB) { // unknown neighbor
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#else   
         return -ENEIGHB;
#endif

      } else {
         schedId = neighbList[nodeIdx].schedId;
      }
#endif
      // Don't accept Tx request if I have already accepted a request
      // disable interrupt when check the value of txRequest
      //intEnabled = inp(SREG) & 0x80;
      //cli();
      if (txRequest == 0) {
         txRequest = 1;
         //if (intEnabled) sei();
      } else {     
         //if (intEnabled) sei();
         smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_TXREQUEST_IS_1);
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#else
         return -ETXBUSY;
#endif
      }      
      dataPkt = (MACHeader*)data;
      sendAddr = toAddr;
      txPktLen = length;
      txFragAll = numFrags;
      txFragCount = 0;
      numRetry = 0;
      numReTx = 0;
      // calculate duration of data packet/fragment
      durDataPkt = (PRE_PKT_BYTES + length * ENCODE_RATIO) 
                  * 8 / BANDWIDTH + 1;
      // fill in MAC header fields except duration
      dataPkt->type = DATA_PKT << 4;  // higher 4 bits
      dataPkt->toAddr = sendAddr;
      dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
      if (nodeIdx < SMAC_MAX_NUM_NEIGHB) { // known neighbor
         neighbList[nodeIdx].txSeqNo = (neighbList[nodeIdx].txSeqNo++) & 0x1f;
         dataPkt->seqFragNo = neighbList[nodeIdx].txSeqNo << 3; //higher 5 bits

         if (fastPathFlag == 1) { // fast path request
         // set up fast path flag in the packet
            dataPkt->type = dataPkt->type|0x80;  // first bit indicating fast path request
         // calculate next hop fast path position based on my fast path schedule position
         // check if I am the source, if so, no need to set up a fast path on the next hop
            if (isSource != 1) { 
               // find the fast path schedule position for next hop
               d = nextHopFastPathPos(sendAddr, fastPathPos, durDataPkt);
               dataPkt->pos = d;
            }
            else {
               dataPkt->pos = 0;
            }
         }  
      }
#ifdef SMAC_NO_SLEEP_CYCLE
      // try to send now
      tryToSend();
#else
      // set flag for data transmission
      dataSched = schedId;
      schedTab[schedId].txData = 1;
#endif
      maxTxDataTime = SMAC_MAX_TX_MSG_TIME;
      return SUCCESS;
   }
#endif

   command result_t MACComm.unicastMsg(void* data, uint8_t length, 
      uint16_t toAddr, uint8_t numFrags)
   {
      uint8_t nodeIdx, emptyIdx;
#ifndef SMAC_NO_SLEEP_CYCLE
      uint8_t schedId;
#endif
      char intEnabled;
      // sanity check
      if (data == 0 || length == 0 ||
         length > PHY_MAX_PKT_LEN || numFrags == 0 || numFrags > 8) {
         if (data == 0) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_DATA_IS_0);
#ifdef SMAC_ERRCODES_ON
            return -EDATA;
#endif  // SMAC_ERRCODES_ON
         }
         if (length == 0 || length > PHY_MAX_PKT_LEN) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_PKTLEN_ERROR);
#ifdef SMAC_ERRCODES_ON
            return -ELENGTH;
#endif  // ERRCODES
         }
         if (numFrags == 0) {
            smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_NUMFRAGS_IS_0);
#ifdef SMAC_ERRCODES_ON
            return -EFRAGS;
#endif
         }
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#endif
      }
      getNodeIdx(toAddr, &nodeIdx, &emptyIdx);
#ifndef SMAC_NO_SLEEP_CYCLE
      if (nodeIdx == SMAC_MAX_NUM_NEIGHB) { // unknown neighbor
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#else   
         return -ENEIGHB;
#endif

      } else {
         schedId = neighbList[nodeIdx].schedId;
      }
#endif
      // Don't accept Tx request if I have already accepted a request
      // disable interrupt when check the value of txRequest
      //intEnabled = inp(SREG) & 0x80;
      //cli();
      trace(DBG_USR1, "MACComm.unicastMsg() called\r\n");
      if (txRequest == 0) {
         txRequest = 1;
         //if (intEnabled) sei();
      } else {     
         //if (intEnabled) sei();
         smacUartDebug_event(SMAC_UCAST_REQUEST_REJECTED_TXREQUEST_IS_1);
#ifndef SMAC_ERRCODES_ON
         return FAIL;
#else
         return -ETXBUSY;
#endif
      }      
      dataPkt = (MACHeader*)data;
      sendAddr = toAddr;
      txPktLen = length;
      txFragAll = numFrags;
      txFragCount = 0;
      numRetry = 0;
      numReTx = 0;
      // calculate duration of data packet/fragment
      durDataPkt = (PRE_PKT_BYTES + length * ENCODE_RATIO) 
                  * 8 / BANDWIDTH + 1;
      // fill in MAC header fields except duration
      dataPkt->type = DATA_PKT << 4;  // higher 4 bits
      dataPkt->toAddr = sendAddr;
      dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
      if (nodeIdx < SMAC_MAX_NUM_NEIGHB) { // known neighbor
         neighbList[nodeIdx].txSeqNo = (neighbList[nodeIdx].txSeqNo++) & 0x1f;
         dataPkt->seqFragNo = neighbList[nodeIdx].txSeqNo << 3; //higher 5 bits
      }
#ifdef SMAC_NO_SLEEP_CYCLE
      // try to send now
      tryToSend();
#else
      // set flag for data transmission
      dataSched = schedId;
      schedTab[schedId].txData = 1;
#endif
      maxTxDataTime = SMAC_MAX_TX_MSG_TIME;
      return SUCCESS;
   }


   command result_t MACComm.txNextFrag(void* data)
   {
      // Send subsequent fragments
      if (state != TX_NEXT_FRAG || data == 0) return FAIL;
      dataPkt = (MACHeader*)data;
      // fill in MAC header fields except duration
      dataPkt->type = DATA_PKT << 4;  // data pkt
      dataPkt->toAddr = sendAddr;
      dataPkt->fromAddr = TOS_LOCAL_ADDRESS;
      dataPkt->seqFragNo++; // = txFragCount; // fragNo <= 7
      //if (neighbNav >= (SIFS + durDataPkt + timeWaitCtrl)) {
         // schedule to send this fragment
         state = TX_PKT;
         howToSend = SEND_DATA;
         txDelay = SIFS;
      //} // else will retry when neighbNav timeout
      return SUCCESS;
   }


   command result_t MACComm.txReset()
   {
      // drop currently tx packet, and reset physical layer
      // after resetting, MAC can accept a new Tx request
      // this command does not change MAC state
#ifndef SMAC_NO_SLEEP_CYCLE
      uint8_t i, numSchedCount = 0;
      // clean up schedule table
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) {
            schedTab[i].txData = 0;
            numSchedCount++;
         }
      }
      numSched = numSchedCount;
#endif
      maxTxDataTime = 0;  // clear Tx reset timer
      resetPhy();  // reset physical layer
      state = IDLE;  // reset MAC state too
      txRequest = 0;
      smacUartDebug_state(state);
      return SUCCESS;
   }


   void startBcast()
   {
      // broadcast data directly; don't use RTS/CTS
/*
#ifdef SMAC_SNOOPER_DEBUG
      *((char*)dataPkt + sizeof(MACHeader) + 10) = numLenErr;
      *((char*)dataPkt + sizeof(MACHeader) + 11) = numCrcErr;
      *((char*)dataPkt + sizeof(MACHeader) + 12) = numSlpPkt;
      *((char*)dataPkt + sizeof(MACHeader) + 13) = numCTStimeout;
      *((char*)dataPkt + sizeof(MACHeader) + 14) = numDATAtimeout;
      *((char*)dataPkt + sizeof(MACHeader) + 15) = numACKtimeout;
	
      *((char*)dataPkt + sizeof(MACHeader) + 17) = numSleeps;
      *((char*)dataPkt + sizeof(MACHeader) + 18) = numWakeups;
      *((char*)dataPkt + sizeof(MACHeader) + 19) = neighbNav;
      *((char*)dataPkt + sizeof(MACHeader) + 20) = syncDiff1;
      *((char*)dataPkt + sizeof(MACHeader) + 21) = syncDiff2;
      *(uint16_t*)((char*)dataPkt + sizeof(MACHeader) + 22) = schedTab[0].counter;
      *(uint16_t*)((char*)dataPkt + sizeof(MACHeader) + 24) = schedTab[1].counter;
#endif
*/
#ifdef SMAC_TX_TIME_STAMP
      dataPkt->txTimeStamp = clockTime;
#endif
      trace(DBG_USR1, "startBcast()\r\n");
      call PhyComm.txPkt(dataPkt, txPktLen);
      radioState = RADIO_TX;
      state = TX_PKT;
      txErrTime = TX_PKT_DONE_TIME;
   }

   
   void txMsgDone()
   {
      // unicast is done
      maxTxDataTime = 0;  // clear Tx hold timer
      // update schedTab and neighbList if flags are set when txRequest=1
      update_schedTab_neighbList();
      // prepare to tx next msg
      txRequest = 0;
#ifdef SMAC_NO_SLEEP_CYCLE
      state = IDLE;
      smacUartDebug_state(state);
#else
      schedTab[dataSched].txData = 0;
      sleep(TRY);
#endif
      signal MACComm.unicastDone(dataPkt, txFragCount);
   }


   task void unicastMsgDone()
   {
      txMsgDone();  // unicast is done
   }
   
   
   task void txDataTimeout()
   {
#ifdef SMAC_SNOOPER_DEBUG
      numTxDataReset++;
#endif
#ifdef SMAC_REPORT
            macTxErr++;
#endif
      smacUartDebug_event(TIMER_FIRE_TX_HOLD_PKT);
      call MACComm.txReset();  // reset tx variables and radio
      // signal Tx is done
      if (dataPkt->toAddr == TOS_BCAST_ADDR) {
         // unfortunately, no indication of failure in broadcast
         signal MACComm.broadcastDone(dataPkt);
      } else {
         txMsgDone();  // signal unicast failure
      }
   }
   

   void update_schedTab_neighbList()
   {
      //update schedTab and neighbList if flag is set
      //we should update the schedTab[].numNodes before we call checkMySched()
      //to ensure the next available schedule is correct
      check_schedFlag();
      if (updateNeighbList == 1) {
         update_neighbList();
         updateNeighbList = 0;
         schedTab[0].chkSched = 0;  //we already did checkMySched() in update_neighbList()   
      }else if (schedTab[0].chkSched == 1) {
         checkMySched();
         schedTab[0].chkSched = 0;   
      }
   }

   event result_t PhyComm.txPktDone(void* packet, uint8_t error)
   {
      char pktType;
      txErrTime = 0;  // cancel tx error timer
      if (resetPhyFlag == 1) { // radio may not work properly
         resetPhyFlag = 0;
         resetPhy();
      } else {
         radioState = RADIO_IDLE;
      }
      if (packet == 0 || state != TX_PKT) return FAIL;  // CHECK if needed
      pktType = (*((char*)packet + sizeof(PhyHeader))) >> 4;
      switch (pktType) {  // the type field
      case SYNC_PKT:
#ifdef SMAC_SNOOPER_DEBUG
         numTxSyncCount++;
#endif
         schedTab[syncSched].txSync = 0;
         state = IDLE;
         smacUartDebug_event(TX_SYNC_DONE);
         smacUartDebug_state(state);
         // reset SYNC timer after sending to my schedule
         if (syncSched == 0) timeToTxSync = SYNC_PERIOD;
         if (numSyncTx > 0) {
            numSyncTx--;  // SYNC is done for one schedule
            if (numSyncTx == 0) {
               syncSeqNo++;  // increase sequence number
               maxTxSyncTime = 0;  // don't reset PHY
               // check if need to update neighbor list
               if (neighbListTime > 0) {
                  neighbListTime--;
                  if (neighbListTime == 0) {  // time to update neighbor list
                     smacUartDebug_event(TIMER_FIRE_UPD_NEIGHB_LIST);
                     if (post update_myNeighbList()) {
                        neighbListTime = UPDATE_NEIGHB_PERIOD; // reset timer
                     } else {
                        neighbListTime = 1;  // try again after next SYNC
                     }
                  }
               }
            }
         }
         break;
      case RTS_PKT:
         // track neighbors' NAV, need to clear it if CTS timeout
         TRACK_NAV(((MACCtrlPkt*)packet)->duration);
         // just sent RTS, set timer for CTS timeout
         state = WAIT_CTS;
         geneTime = timeWaitCtrl;
         smacUartDebug_event(TX_RTS_DONE);
         smacUartDebug_state(state);
         break;
      case CTS_PKT:  // just sent CTS
         // track my neighbors' NAV
         // they update NAV and go to sleep after recv CTS
         TRACK_NAV(((MACCtrlPkt*)packet)->duration);
         state = WAIT_DATA;
         // no data timeout, just use neighbors' NAV
         // since they went to sleep, just wait data for the entire time
         smacUartDebug_event(TX_CTS_DONE);
         smacUartDebug_state(state);
         break;
      case DATA_PKT:
         if (((MACHeader*)packet)->toAddr == TOS_BCAST_ADDR) {
            smacUartDebug_event(TX_BCAST_DONE);
#ifdef SMAC_NO_SLEEP_CYCLE
            state = IDLE;
            maxTxDataTime = 0;  // clear Tx hold timer
            smacUartDebug_state(state);
            txRequest = 0;
            signal MACComm.broadcastDone(dataPkt);
#else
            // broadcast data is done for one schedule
            schedTab[dataSched].txData = 0;
            sleep(TRY);
            numBcast--;
            if (numBcast == 0) {
               // broadcast is done for all schedules
               maxTxDataTime = 0;  // clear Tx hold timer
               // update schedTab and neighbList if flags are set when txRequest=1
               update_schedTab_neighbList();
               txRequest = 0;
               signal MACComm.broadcastDone(dataPkt);
            }
#endif
         } else {  // unicast is done
            // waiting for ACK, set timer for ACK timeout
            state = WAIT_ACK;
            geneTime = timeWaitCtrl;
            smacUartDebug_event(TX_UCAST_DONE);
            smacUartDebug_state(state);
         }
         break;
      case ACK_PKT:
         TRACK_NAV(((MACCtrlPkt*)packet)->duration); // in case tx extended.
         // MAC stay in WAIT_DATA state until neighbNav becomes zero
         state = WAIT_DATA;
         smacUartDebug_event(TX_ACK_DONE);
         smacUartDebug_state(state);
         break;
      }
      return SUCCESS;
   }


   void tryToResend(uint8_t delay)
   {
      // try to re-send a packet when CTS or ACK timeout
      if (numRetry < SMAC_RTS_RETRY_LIMIT) {
         numRetry++;
#ifdef SMAC_NO_SLEEP_CYCLE
         state = IDLE;
         smacUartDebug_state(state);
         if (delay == 0) tryToSend();
         else retryTime = delay;
#else
         // wait unitl receiver's next wake-up time
         sleep(TRY);
#endif
      } else {
#ifdef SMAC_REPORT
         macTxErr++;
#endif
         // reached retry limit, give up Tx
         if (!post unicastMsgDone()) { // if can't post task, signal directly
            txMsgDone(); // with txFragCount < txFragAll;
         }
      }
   }


   void adaptiveListen()
   {
      // adaptively wake-up at the end of current transmission. Will try to 
      // send only if the buffered packet is unicast. Since my next-hop 
      // neighbor may not be aware of the Tx of my previous-hop neighbor, 
      // broadcast now is unreliable.
#ifdef FAST_PATH_SCHEDULE
      uint8_t i, overlap;
      // fast path schedule has priority when it overlaps with adaptive listen  
      overlap = 0;
      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
         if (fastPathSchedTab[i].active == 1) {
            if ((period + listenTime - fastPathSchedTab[i].pos - dataTime) <= schedTab[0].counter 
                <= (period + listenTime - fastPathSchedTab[i].pos + dataTime)) {
               overlap = 1;
               break;
            }
            if ((period + listenTime - fastPathSchedTab[i].neighbPos - dataTime) <= schedTab[0].counter
                <= (period + listenTime - fastPathSchedTab[i].neighbPos + dataTime)) {
               overlap = 1;
               break;
            }
         }
      }
      if ((srchNeighb == 1 ||
         schedTab[0].counter > dataTime + listenTime ||
         schedTab[0].counter < dataTime) && overlap == 0) {
#else
      if (srchNeighb == 1 || 
         schedTab[0].counter > dataTime + listenTime ||
         schedTab[0].counter < dataTime) {
#endif
         adapTime = dataTime; // set timer to bring me back to sleep
         if (state == SLEEP) wakeup();
         else state = IDLE;
         smacUartDebug_state(state);
         if (txRequest == 1 && sendAddr != TOS_BCAST_ADDR &&
            (schedTab[dataSched].counter > 
            dataTime + listenTime ||
            schedTab[dataSched].counter < dataTime)) {
            tryToSend();
         }
      } else {
         sleep(TRY);
      }
   }


   task void geneTimerFire()
   {
      // generic timer fired
      // this timer is used to wait for packet transmission and reception
      uint8_t numRemain;
      if (state == DATA_SENSE1) {
         smacUartDebug_event(TIMER_FIRE_DATA_SENSE1);
         // didn't see a CTS reply to a previous RTS
         geneTime = timeWaitCtrl;
         state = DATA_SENSE2;
         smacUartDebug_state(state);
      } else if (state == DATA_SENSE2) {
         smacUartDebug_event(TIMER_FIRE_DATA_SENSE2);
         // didn't see data tx, sender of RTS didn't get a CTS
         nav = 0;  // cancel my NAV
#ifdef SMAC_NO_SLEEP_CYCLE
         state = IDLE;
         smacUartDebug_state(state);
         if (txRequest == 1) {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
            tryToSend();
         } else {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
         }
#else
         sleep(TRY);
#endif
      } else if (state == WAIT_CTS) {	// CTS timeout
         smacUartDebug_event(TIMER_FIRE_WAIT_CTS);
#ifdef SMAC_SNOOPER_DEBUG
         numCTStimeout++;
#endif
         neighbNav = 0; // neighbors won't follow my reserved time
         tryToResend(EIFS); // wait neighbors in data_sense2
      } else if (state == WAIT_ACK) { // ACK timeout
         smacUartDebug_event(TIMER_FIRE_WAIT_ACK);
#ifdef SMAC_SNOOPER_DEBUG
         numACKtimeout++;
#endif
         if (numReTx < SMAC_DATA_RETX_LIMIT && 
            neighbNav > durDataPkt) {
            // can extend tx time
            numReTx++;
            // need to increase neighbNav for one frag
            numRemain = txFragAll - txFragCount;
            neighbNav = numRemain * durCtrlPkt + 
               numRemain * durDataPkt +
               (numRemain + numRemain - 1) * (PROC_DELAY + SIFS);
            dataPkt->type++;  // lower 4 bits are a counter for retx
            sendDATA();
            state = TX_PKT;
            smacUartDebug_state(state);
#ifdef SMAC_REPORT
            retx++;
#endif
         }
      }
   }


   task void navFire()
   {
      smacUartDebug_event(TIMER_FIRE_NAV);
#if defined SMAC_NO_SLEEP_CYCLE
      wakeup();
      if (txRequest == 1) {
         smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
         tryToSend();
      } else {
         smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
      }
#elif defined SMAC_NO_ADAPTIVE_LISTEN
      if (srchNeighb == 1) wakeup();
#else
      adaptiveListen();
#endif
   }


   task void neighbNavFire()
   {
      smacUartDebug_event(TIMER_FIRE_NEIGHBOR_NAV);
      if (state == WAIT_ACK) {
         // last tx is not done within reserved time
         if (numReTx < SMAC_DATA_RETX_LIMIT) { // data reTx allowed
            numReTx++;
            dataPkt->type++; // lower 4 bits are a counter for retx
#ifdef SMAC_REPORT
            retx++;
#endif
            tryToResend(0); // try to resend now
         } else {
            txMsgDone(); // with txFragCount < txFragAll;
         }
      } else { // could be in sleep, idle or wait_data state
#if defined SMAC_NO_SLEEP_CYCLE
         state = IDLE;
         smacUartDebug_state(state);
         if (txRequest == 1) {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
            tryToSend();
         } else {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
         }
#elif defined SMAC_NO_ADAPTIVE_LISTEN
         sleep(TRY);
#else			
         adaptiveListen();
#endif
      }
   }


   task void txErrTimerFire()
   {
      // radio in Tx mode for too long
      smacUartDebug_event(TIMER_FIRE_TX_ERR);
      resetPhy();  // radio may not work properly
      state = IDLE;  // reset MAC state too
      smacUartDebug_state(state);
#ifdef SMAC_NO_SLEEP_CYCLE            
      if (txRequest == 1) {
         smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
         tryToSend();
      } else {
         smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
      }
#endif            
   }


   event result_t Timer.fired()
   {
      // handle clock event
#ifndef SMAC_NO_SLEEP_CYCLE
      uint8_t i;
      char intEnabled;
      uint16_t backoffSlots, listenBits;
#endif
#ifdef FAST_PATH_SCHEDULE
      uint8_t j;
#endif
#ifdef SMAC_PERFORMANCE
      if (cntRadioTime) {
         if (radioState == RADIO_SLEEP)
            radioTime.sleepTime++;
         else if (radioState == RADIO_IDLE)
            radioTime.idleTime++;
         else if (radioState == RADIO_RX)
            radioTime.rxTime++;
         else if (radioState == RADIO_TX)
            radioTime.txTime++;
      }
#endif

      // advance clock
      clockTime++;

#ifdef FAST_PATH_SCHEDULE
      // update status of each fast pass schedule
      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
        if (fastPathSchedTab[i].active == 1) {
            if (fastPathSchedTab[i].counter > 0) {
                fastPathSchedTab[i].counter--;
                if (fastPathSchedTab[i].counter == dataTime) { // wakeup
                    fastPathListen ++;
                    if (state == SLEEP) wakeup();
                    //else state = IDLE;
                }
                else if (fastPathSchedTab[i].counter == 0) { // sleep
                    fastPathListen --;
                    if (state == IDLE && txDelay == 0) sleep(TRY);
                }
            }
                                                                                                                                                            
            if (fastPathSchedTab[i].neighbCounter > 0) {
                fastPathSchedTab[i].neighbCounter--;
                if (fastPathSchedTab[i].neighbCounter == dataTime) { // wakeup
                    fastPathListen ++;
                    if (state == SLEEP) wakeup();
                    //else state = IDLE;
                }
                else if (fastPathSchedTab[i].neighbCounter == 0) { // sleep
                    fastPathListen --;
                    if (state == IDLE && txDelay == 0) sleep(TRY);
                }
            }
         }
      }
                                                                                                                                                            
      // check if any fast pass schedule expires
      for (i = 0; i < SMAC_MAX_NUM_FASTPATH_SCHED; i++) {
         if (fastPathSchedTab[i].active == 1) {
            fastPathSchedTab[i].timeToExpire--;
            if (fastPathSchedTab[i].timeToExpire == 0) { // sleep
                fastPathSchedTab[i].active = 0;
            }
         }
      }  
#endif

      // NAV timer
      if (nav > 0) {
         nav--;
         if (nav == 0) {  // my neighbor is done Tx/Rx
            if (!post navFire()) nav = 1;
         }
      }

      // timer to track my neighbor's NAV
      if (neighbNav > 0) {
         neighbNav--;
         if (neighbNav == 0) {  // used up reserved Tx/Rx time
            if (!post neighbNavFire()) neighbNav = 1;
         }
      }

      // generic timer, used to wait for packet Tx and Rx
      if (geneTime > 0) {
         geneTime--;
         if (geneTime == 0) {
            if (!post geneTimerFire()) geneTime = 1;
         }
      }

#ifndef SMAC_NO_SLEEP_CYCLE
      // check if it's time to send a SYNC packet
      if (timeToTxSync > 0) {
         timeToTxSync--;
         if (timeToTxSync == 0) {
            if (post txSyncTimerFire()) {
               timeToTxSync = SYNC_PERIOD; //reset timer
            } else {
               timeToTxSync = 1;
            }
         }
      }
      
      // update status of each schedule and arrange tx if needed
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) {
            schedTab[i].counter--;
            if (schedTab[i].counter == listenTime) {
               smacUartDebug_event(TIMER_FIRE_LISTEN_SYNC);
#ifdef FAST_PATH_SCHEDULE
               // set up each fast path schedule 
               if (i == 0) {
                  for (j = 0; j < SMAC_MAX_NUM_FASTPATH_SCHED; j++) {
                    if (fastPathSchedTab[j].active == 1) {
                        if (fastPathSchedTab[j].pos != 0) {
                            fastPathSchedTab[j].counter = fastPathSchedTab[j].pos + dataTime;
                        }
                        if (fastPathSchedTab[j].neighbPos != 0) {
                            fastPathSchedTab[j].neighbCounter = fastPathSchedTab[j].neighbPos + dataTime;
                        }
                     }
                  }  
               };
#endif
               if (i == 0) schedListen = 1; // it's my scheduled wake-up time
               if (nav == 0 && neighbNav == 0 &&
                  (state == SLEEP || state == IDLE)) {
                  if (state == SLEEP &&
                     (i == 0 || schedTab[i].txSync == 1)) {
                     wakeup();  // wake up to listen or send sync
                  }
                  if (schedTab[i].txSync == 1) {
                     // start carrier sense for sending sync
                     howToSend = SEND_SYNC;
                     syncSched = i;
                     backoffSlots = call Random.rand() & (uint16_t)SYNC_CW;
                     listenBits = (DIFS + SLOTTIME * backoffSlots) * LISTEN_RATE;
                     // start carrier sense and change state needs to be atomic
                     // to prevent start symbol is detected between them
                     //intEnabled = inp(SREG) & 0x80;
                     //cli();
                     state = CARR_SENSE;
                     if (call CarrierSense.start(/*listenBits*/) == SUCCESS) {
                        //state = CARR_SENSE;
                     }
                     //if (intEnabled) sei();
                     smacUartDebug_state(state);
                  }
               }
            } else if (schedTab[i].counter == dataTime) {
               smacUartDebug_event(TIMER_FIRE_LISTEN_DATA);
               if (schedTab[i].txData == 1 &&
                  nav == 0 && neighbNav == 0 &&
                  (state == SLEEP || state == IDLE)) {
                  // schedule sending of data
                  if (state == SLEEP) wakeup();  // wake up to tx
                  dataSched = i;
                  tryToSend();
               }
            } else if (schedTab[i].counter == 0) {
               smacUartDebug_event(TIMER_FIRE_SCHED_SLEEP);
               schedTab[i].counter = period;
               if (i == 0) {  // my scheduled lisen time is over
                  schedListen = 0;
                  if (state == IDLE && txDelay == 0) sleep(TRY); // sleep if idle
               }
            }
         }
      }
#endif

      // tx delay timer
      if (txDelay > 0) {
         txDelay--;
         if (txDelay == 0) {
            //if (!post txDelayTimerFire()) txDelay = 1;
            if (state == TX_PKT) {  // start sending
               smacUartDebug_event(TIMER_FIRE_TX_DELAY);
               if (howToSend == SEND_CTS) {
                  sendCTS();
               } else if (howToSend == SEND_DATA) {
                  sendDATA();
               } else if (howToSend == SEND_ACK) {
                  sendACK();
               }
            }
         }
      }
      
      // adaptive wake-up timer
      if (adapTime > 0) {
         adapTime--;
         if (adapTime == 0) {
            smacUartDebug_event(TIMER_FIRE_ADAP_LISTEN_DONE);
            if (state == IDLE && txDelay == 0) sleep(TRY);
         }
      }
      
      // max time to hold a packet for Tx
      if (maxTxDataTime > 0) {
         maxTxDataTime--;
         if (maxTxDataTime == 0) {  // Tx message took too long
            if (!post txDataTimeout()) maxTxDataTime = 1;
         }
      }
      
      // tx error timer
      if (txErrTime > 0) {
         txErrTime--;
         if (txErrTime == 0) { // radio in tx state for too long
            if (!post txErrTimerFire()) txErrTime = 1;
         }
      }

#ifdef SMAC_NO_SLEEP_CYCLE
      if (retryTime > 0) {
         retryTime--;
         if (retryTime == 0) {
            smacUartDebug_event(TIMER_FIRE_TX_RETRY);
            tryToSend();
         }
      }
      // need a separate timer for updating neighbor list in fully active mode
      if (dataActiveTime > 0) {
         dataActiveTime--;
         if (dataActiveTime == 0) {
            // there is no mess with sending in fully active mode, since
            // sending does not check neighbor list or schedule table
            if (post update_myNeighbList()) {
               dataActiveTime = DATA_ACTIVE_PERIOD; // reset timer
            } else {
               dataActiveTime = 1;
            }
         }
      }
#endif

      signal MACTest.clockFire(); // signal upper layer the clock event
      
      // try to put CPU into idle mode
      if (state == SLEEP) call PowerManagement.adjustPower();

      return SUCCESS;      
   }


   event result_t CarrierSense.channelBusy()
   {
      // physical carrier sense indicate channel busy
      // Do nothing and stay in idle to receive a packet
      // Will sleep at my next sleep time if can't get a packet
      trace(DBG_USR1, "CarrierSense.channelBusy() signalled :(\r\n");
      smacUartDebug_event(CHANNEL_BUSY_DETECTED);
      if (state == CARR_SENSE) {
         state = IDLE;
         smacUartDebug_state(state);
#ifdef SMAC_NO_SLEEP_CYCLE
         retryTime = EIFS;
#endif
      }
      return SUCCESS;
   }


   event result_t CarrierSense.channelIdle()
   {
      // physical carrier sense indicate channel idle
      // start sending
      trace(DBG_USR1, "CarrierSense.channelIdle() signalled :)\r\n");
      smacUartDebug_event(CHANNEL_IDLE_DETECTED);
      if (state != CARR_SENSE) return FAIL;
      if (howToSend == SEND_SYNC) {
         sendSYNC();
      } else if (howToSend == BCAST_DATA) {
         startBcast();
      } else if (howToSend == SEND_RTS) {
         sendRTS();
      }	
      return SUCCESS;
   }


   event result_t PhyComm.startSymDetected(void* pkt)
   {
      smacUartDebug_event(START_SYMBOL_DETECTED);
      radioState = RADIO_RX;
      if (state == IDLE || state == CARR_SENSE) {	
         state = BACKOFF;
         smacUartDebug_state(state);
         // put in coarse time stamp in ms
         ((PhyPktBuf*)pkt)->info.timeCoarse = clockTime;
         // put in fine/external time stamp
         //XXX: call TimeStamp.getTime32(&(((PhyPktBuf*)pkt)->info.timestamp));

#ifdef SMAC_NO_SLEEP_CYCLE
         retryTime = 0;
#endif
      } else if (state == DATA_SENSE2) {
         // data tx started from the sender of RTS
         geneTime = 0;
         sleep(FORCE);
      } else {
         smacUartDebug_state(state);
      }
      return SUCCESS;
   }


   void handleErrPkt()
   {
      
      if (state == BACKOFF) {
         state = IDLE;
         smacUartDebug_state(state);
      }
      if (state == IDLE) {
#ifdef SMAC_NO_SLEEP_CYCLE
         if (txRequest == 1) {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
            retryTime = EIFS;
         } else {
            smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
         }
#else
         sleep(TRY);  // wait until next tx time
#endif
      }
   }
   

   event void* PhyComm.rxPktDone(void* packet, uint8_t error)
   {
      char pktType;
      trace(DBG_USR1, "rxPktDone() called in SMACM.nc, state = %d\n", state);
      if (state == SLEEP) {
#ifdef SMAC_SNOOPER_DEBUG
         numSlpPkt++;
#endif
#ifdef SMAC_REPORT
         slpErr++;
#endif

         return packet; // if in sleep, reject any pkt. A bug occurs!
      }

      radioState = RADIO_IDLE;
      
      if (error) {  // if received an erroneous pkt, a sign of collision
         smacUartDebug_event(RX_ERROR);
#ifdef SMAC_SNOOPER_DEBUG
         if (packet == NULL) numLenErr++;
         else numCrcErr++;
#endif

#ifdef SMAC_REPORT
	if (packet == NULL) {
	    lenErr++;
	} else {
    	    crcErr++;
	}
#endif // SMAC_REPORT
         handleErrPkt();
         return packet;
      }
      pktType = (*((char*)packet + sizeof(PhyHeader))) >> 4;
      trace(DBG_USR1, "rxPktDone(), pktType=%d\r\n", pktType);
      // dispatch to actual packet handlers
      if (pktType == DATA_PKT) {
         return handleDATA(packet);
      } else if (pktType == RTS_PKT) {
#ifdef SMAC_REPORT
	ctrlPkts++;
#endif
         smacUartDebug_event(RX_RTS_DONE);
         handleRTS(packet);
      } else if (pktType == CTS_PKT) {
#ifdef SMAC_REPORT
	ctrlPkts++;
#endif
         smacUartDebug_event(RX_CTS_DONE);
         handleCTS(packet);
      } else if (pktType == ACK_PKT) {
#ifdef SMAC_REPORT
	ctrlPkts++;
#endif
         smacUartDebug_event(RX_ACK_DONE);
         handleACK(packet);
#ifndef SMAC_NO_SLEEP_CYCLE
      } else if (pktType == SYNC_PKT) {
         smacUartDebug_event(RX_SYNC_DONE);
#ifdef SMAC_REPORT
	ctrlPkts++;
#endif
			
         handleSYNC(packet);
#endif  // SMAC_NO_SLEEP_CYCLE
      } else {  // unknown packet
         smacUartDebug_event(RX_UNKNOWN_PKT);
         handleErrPkt();
      }
      return packet;
   }


   void handleRTS(void* pkt)
   {
      // internal handler for RTS
      MACCtrlPkt* packet;
      if (state != IDLE && state != BACKOFF) return;
      packet = (MACCtrlPkt*)pkt;
      if (packet->toAddr == TOS_LOCAL_ADDRESS) {
         recvAddr = packet->fromAddr;  // remember sender's address
         rxFragAll = packet->type & 0xf; // lower 4 bits are number of frags
         lastRxFrag = 250;
         TRACK_NAV(packet->duration); // track neighbors' NAV
         // schedule sending CTS
         state = TX_PKT;
         howToSend = SEND_CTS;
         txDelay = SIFS;
      } else { // packet destined to another node
         // assume sender will get a CTS and tx data
         UPDATE_NAV(packet->duration);
         // keep listening until confirm sender gets a CTS or starts tx data
         state = DATA_SENSE1; // wait for a CTS
         geneTime = timeWaitCtrl;
      }
      smacUartDebug_state(state);
   }
		

   void handleCTS(void* pkt)
   {
      // internal handler for CTS
      MACCtrlPkt* packet;
      packet = (MACCtrlPkt*)pkt;
      if (packet->toAddr == TOS_LOCAL_ADDRESS) {
         if (state == WAIT_CTS && packet->fromAddr == sendAddr) {
            // cancel CTS timer
            geneTime = 0;
            // schedule sending DATA
            state = TX_PKT;
            howToSend = SEND_DATA;
            txDelay = SIFS;
            smacUartDebug_state(state);
         } else {
            handleErrPkt();
         }
      } else { // packet destined to another node
         if (state == DATA_SENSE1 || state == DATA_SENSE2) {
            geneTime = 0;
         }
         if (state == IDLE || state == BACKOFF || 
            state == DATA_SENSE1 || state == DATA_SENSE2) {
            UPDATE_NAV(packet->duration);
            sleep(FORCE);  // avoid overhearing other's packet
            if (nav == 0) nav = 1;  // in case duration is 0 by mistake
         }
      }
   }


   uint8_t checkNeighbor(uint16_t nodeAddr)
   {
      // check if the neighbor is on my neighbor list
      // return its index on neighbor list
      // if in fully active mode and the node is not on list, try to add it
      uint8_t nodeIdx, emptyIdx;
      // check if it is from a known neighbor
      getNodeIdx(nodeAddr, &nodeIdx, &emptyIdx);
      if (nodeIdx < SMAC_MAX_NUM_NEIGHB) { // known neighbor
         neighbList[nodeIdx].active = 1;
#ifdef SMAC_NO_SLEEP_CYCLE
      } else { // unknown neighbor
         if(emptyIdx < SMAC_MAX_NUM_NEIGHB) { // have space on list
            nodeIdx = emptyIdx;
            // put the new node into neighbor list
            neighbList[nodeIdx].state = 1;
            neighbList[nodeIdx].nodeId = nodeAddr;
            neighbList[nodeIdx].schedId = 0;
            neighbList[nodeIdx].active = 1;
            neighbList[nodeIdx].txSeqNo = 0;
            neighbList[nodeIdx].rxSeqNo = SMAC_MAX_UCAST_SEQ_NO;
            numNeighb++;  // increment number of neighbors
            if (numNeighb == 1) { // start timer to update neighbor list
               dataActiveTime = DATA_ACTIVE_PERIOD;
            }
            signal LinkState.nodeJoin(nodeAddr);
         }
#endif
      }
      return nodeIdx;
   }
      

   void* handleDATA(void* pkt)
   {
      // internal handler for DATA packet
      void* tmp;
      uint8_t nodeIdx, seqNo, fragNo, numTxFragNew;
      MACHeader* packet = (MACHeader*)pkt;
      trace(DBG_USR1, "handleDATA() called\r\n"); 
      // check if it is from a known neighbor
      nodeIdx = checkNeighbor(packet->fromAddr);
      
      if (packet->toAddr == TOS_BCAST_ADDR) {  // broadcast packet
         smacUartDebug_event(RX_BCAST_DONE);
#ifdef SMAC_NO_SLEEP_CYCLE
         if (state == BACKOFF) state = IDLE;
         smacUartDebug_state(state);
         if (state == IDLE) {         
            if (txRequest == 1) {
               smacUartDebug_event(SMAC_TX_REQUEST_IS_1);
               tryToSend();
            } else {
               smacUartDebug_event(SMAC_TX_REQUEST_IS_0);
            }
         }
#else
         sleep(TRY);
#endif
         if (nodeIdx < SMAC_MAX_NUM_NEIGHB) { // known neighbor
            signal LinkState.rxBcastPkt(packet->fromAddr, packet->seqFragNo);
         }
         tmp = signal MACComm.rxMsgDone(packet);
         return tmp;
      } else if (packet->toAddr == TOS_LOCAL_ADDRESS) {  // unicast packet
         // could receive data in idle, backoff and wait_data state
         smacUartDebug_event(RX_UCAST_DONE);
         if (state == WAIT_DATA && packet->fromAddr == recvAddr) {
            // Should track neighbors' NAV, in case tx extended
            TRACK_NAV(packet->duration);
            // schedule sending ACK
            state = TX_PKT;
            howToSend = SEND_ACK;
            txDelay = SIFS;
            smacUartDebug_state(state);
            // check neighbor list
            if (nodeIdx == SMAC_MAX_NUM_NEIGHB) {  // unknow neighbor
               // just pass data up
               tmp = signal MACComm.rxMsgDone(packet);
               return tmp;
            } else {  // a know neighbor
               // update link quality and check duplicate
               seqNo = packet->seqFragNo >> 3;  // higher 5 bits
               fragNo = packet->seqFragNo & 0x7;  // lower 3 bits
               numTxFragNew = (packet->type & 0xf) + 1; // re-Tx + original Tx
               if (neighbList[nodeIdx].rxSeqNo != seqNo) {  // new msg
#ifdef FAST_PATH_SCHEDULE
               // check if need to setup fath path schedule
                if ((packet->type & 0x80) == 1) {
                   if (packet->pos != 0) { 
                        setFastPathSched(packet->pos); 
                   }
                }
#endif
                  neighbList[nodeIdx].rxSeqNo = seqNo;
                  lastRxFrag = fragNo;
                  numTxFragOld = numTxFragNew;
                  numTx1Msg = numTxFragNew;
                  numRx1Msg = 1; // first fragment received
                  if (lastRxFrag == rxFragAll - 1) { // only 1 frag in this msg
                     signal LinkState.rxUcastPkt(recvAddr, seqNo, numTx1Msg,
                                                 numRx1Msg);
                  }
                  tmp = signal MACComm.rxMsgDone(packet);
                  return tmp;
               } else {  // the same msg
                  numTx1Msg += numTxFragNew;
                  numRx1Msg++;
                  if (lastRxFrag != fragNo) {  // new fragment
                     lastRxFrag = fragNo;
                     if (lastRxFrag == rxFragAll - 1) { // all frags received
                        signal LinkState.rxUcastPkt(recvAddr, seqNo, numTx1Msg,
                                                 numRx1Msg);
                     }
                     tmp = signal MACComm.rxMsgDone(packet);
                     return tmp;
                  } else {  // duplicated fragment
                     numTx1Msg -= numTxFragOld;
                     if (lastRxFrag == rxFragAll - 1) { // last fragment
                        signal LinkState.rxUcastPkt(recvAddr, seqNo, numTx1Msg,
                                                   numRx1Msg);
                     }
                  }
                  numTxFragOld = numTxFragNew;
               }
            }
         } else {
            handleErrPkt();
         }
      } else { // unicast packet destined to another node
         if (state == IDLE || state == BACKOFF ||
            state == DATA_SENSE1 || DATA_SENSE2) {
            UPDATE_NAV(packet->duration);
            sleep(FORCE); // avoid overhearing other's packet
            if (nav == 0) nav = 1;  // in case duration is 0 by mistake
         }
      }
      return pkt;
   }


   void handleACK(void* pkt)
   {
      // internal handler for ACK packet
      MACCtrlPkt* packet;
      packet = (MACCtrlPkt*)pkt;
      if (packet->toAddr == TOS_LOCAL_ADDRESS) {
         if (state == WAIT_ACK && packet->fromAddr == sendAddr) {
            // cancel ACK timer
            geneTime = 0;
            txFragCount++;  // number of successfully transmitted frags

            if (txFragCount < txFragAll) {
               state = TX_NEXT_FRAG;
               smacUartDebug_state(state);
               if (signal MACComm.txFragDone(dataPkt) == FAIL)
                  txMsgDone();
            } else { // no more fragment, tx done
               txMsgDone();
            }
            checkNeighbor(packet->fromAddr); // check and update activity
         } else {
            handleErrPkt();
         }
      } else { // packet destined to another node
         if (state == IDLE || state == BACKOFF) {
            UPDATE_NAV(packet->duration);
            sleep(FORCE);
            if (nav == 0) nav = 1;
         }
      }
   }


   void handleSYNC(void* pkt)
   {
      // internal handler for SYNC packet
      MACSyncPkt* packet = (MACSyncPkt*)pkt;
      uint8_t i, nodeId, emptyId, schedId;
      uint16_t refTime, rxDelay;
      char timeDiff;
#ifdef GLOBAL_SCHEDULE
      uint32_t updateAgeTime;
      char ageDiff;
#endif      
      // if carrier sense failed and received a sync pkt, go back to idle
      if (state == BACKOFF) {
         state = IDLE;
         smacUartDebug_state(state);
      }
      
      // calculate Rx delay of sync packet
      // adjust TX_TRANSITION_TIME to make rxDelay calculated correctly
      rxDelay = (uint16_t)(clockTime - ((PhyPktBuf*)pkt)->info.timeCoarse) + 
               PRE_PKT_BYTES * 8 / BANDWIDTH + TX_TRANSITION_TIME;
      //smacUartDebug_byte(rxDelay);
      // sanity check
      if (rxDelay < durSyncPkt - 2 || rxDelay > durSyncPkt + 10) {
#ifdef SMAC_SNOOPER_DEBUG
         numRxSyncErr++;
#endif
         return;
      }

      refTime = packet->sleepTime - rxDelay;
#ifdef GLOBAL_SCHEDULE
      updateAgeTime = clockTime;
#endif
#ifdef SMAC_SNOOPER_DEBUG
      // output syncDiff1 in every data packet
      syncDiff1 = schedTab[0].counter - refTime;
      numRxSync++;
#endif
      //smacUartDebug_byte((uint8_t)(schedTab[0].counter - refTime));
      if (numNeighb == 0) {  // I have no neighbor now
         setMySched(packet, refTime);  // follow this schedule
         signal LinkState.nodeJoin(packet->fromAddr);
         signal LinkState.rxSyncPkt(packet->fromAddr, packet->seqNo);
#ifdef SMAC_SLAVE_SCHED
         timeToTxSync = 1; // will send SYNC and enter low duty cycles
#endif
         return;
      }

      // check if sender is on my neighbor list
      if (getNodeIdx(packet->fromAddr, &nodeId, &emptyId)) { //a known neighbor
         schedId = neighbList[nodeId].schedId;
         if (neighbList[nodeId].state == packet->state) {
            // update the existing schedule
            schedTab[schedId].counter = (schedTab[schedId].counter + refTime) >> 1;
#ifdef GLOBAL_SCHEDULE
            // update schedule age 
            schedTab[schedId].age = (schedTab[schedId].age + clockTime - 
                schedTab[schedId].lastAgeUpdate 
                + packet->age + rxDelay ) >> 1;
            schedTab[schedId].lastAgeUpdate = updateAgeTime;
            // switch to schedule with elder age or same age with smaller initializer Id
            ageDiff = schedTab[schedId].age - (schedTab[0].age + clockTime - 
                schedTab[0].lastAgeUpdate);
            if (ageDiff > GUARDTIME) {
               globalScheduleChange(schedId);
            }
            // same age but initialized by different nodes
            else if ((ageDiff >= -GUARDTIME && ageDiff <= GUARDTIME && 
                schedTab[schedId].syncNode < schedTab[0].syncNode)) {
               globalScheduleChange(schedId);
            }
#endif
            neighbList[nodeId].active = 1;
            signal LinkState.rxSyncPkt(packet->fromAddr, packet->seqNo);
            return;
         } else {  // the node just changed schedule
            if (schedTab[schedId].numNodes == 1 && txRequest == 1) {
               // set flag to decrement numNodes after tx pkt is done
               schedTab[schedId].chkSched = 1;
            } else {
	           // decrement number of nodes on old schedule
               schedTab[schedId].numNodes--;
               if (schedTab[schedId].numNodes == 0) numSched--;
            }
         }
      } else {  // a new neighbor
         // if neighbor list is full, drop the node
         if (emptyId == SMAC_MAX_NUM_NEIGHB) return;
      }

      // now it's either a new node or an old node switching to a new schedule
      // check if its schedule is a known one to me
      schedId = SMAC_MAX_NUM_SCHED;
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) {
            timeDiff = schedTab[i].counter - refTime;
#ifdef GLOBAL_SCHEDULE
            ageDiff = schedTab[i].age + clockTime - 
                schedTab[i].lastAgeUpdate - packet->age -rxDelay; 
            if (timeDiff >= -GUARDTIME && timeDiff <= GUARDTIME && 
                schedTab[i].syncNode == packet->syncNode 
                && ageDiff >= -GUARDTIME && ageDiff <= GUARDTIME) {
               schedTab[i].age = (schedTab[i].age + clockTime - 
                    schedTab[i].lastAgeUpdate + packet->age + rxDelay ) >> 1;
               schedTab[i].lastAgeUpdate = updateAgeTime;

#else
            if (timeDiff >= -GUARDTIME && timeDiff <= GUARDTIME) {
#endif
               schedTab[i].counter = (schedTab[i].counter + refTime) >> 1;
               schedTab[i].numNodes++; // it will follow this schedule
               schedId = i;
               break;
#ifdef SMAC_SNOOPER_DEBUG
            } else {
               syncDiff2 = timeDiff;       //bigger GUARDTIME only
#endif
            }
         }
      }
      if (schedId == SMAC_MAX_NUM_SCHED) {  // unknow schedule
         // add an entry to the schedule table
         if (numSched < SMAC_MAX_NUM_SCHED){
            for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
              if (schedTab[i].numNodes == 0) { // found an empty entry
                 schedTab[i].counter = refTime;
#ifdef GLOBAL_SCHEDULE
                 schedTab[i].syncNode = packet->syncNode;
                 schedTab[i].age = packet->age + rxDelay;
                 schedTab[i].lastAgeUpdate = updateAgeTime;
#endif
                 schedTab[i].txSync = 0;
                 schedTab[i].txData = 0;
                 schedTab[i].numNodes = 1; // 1st node following this sched
                 schedId = i;
                 numSched++;  // increment number of schedules
                 break;
              }
            }
         }  
      }
      
      if (nodeId == SMAC_MAX_NUM_NEIGHB) {  // a new node
         // if no room in schedule table for a new schedule, drop the new node
         if (schedId == SMAC_MAX_NUM_SCHED) return;
         // add it to my neighbor list -- we know there's still room
         neighbList[emptyId].state = packet->state;
         neighbList[emptyId].nodeId = packet->fromAddr;
         neighbList[emptyId].schedId = schedId;
         neighbList[emptyId].active = 1;
         neighbList[emptyId].txSeqNo = 0;
         neighbList[emptyId].rxSeqNo = SMAC_MAX_UCAST_SEQ_NO;
         numNeighb++;  // increment number of neighbors
         signal LinkState.nodeJoin(packet->fromAddr);
      } else {  // old node switches to a new schedule
         // if no room in schedule table, delete the old node
         if (schedId == SMAC_MAX_NUM_SCHED) { 
            neighbList[nodeId].state = 0;
            numNeighb--;  // decrement number of neighbors
         } else {
            neighbList[nodeId].state = packet->state;
            neighbList[nodeId].schedId = schedId;
            neighbList[nodeId].active = 1;
         }
         // maybe the old node switches from schedTab[0]
         // check if I am the only one on schedTab[0] now
         // if yes, I should follow the next available schedule
         if (txRequest == 0) {
            checkMySched();
         } else {
            // set flag to call checkMySched() when txRequest becomes 0
            schedTab[0].chkSched = 1;
         }
      }
#ifdef GLOBAL_SCHEDULE
      if (schedId != 0) {
         ageDiff = schedTab[schedId].age - (schedTab[0].age + 
            clockTime - schedTab[0].lastAgeUpdate);
         if (ageDiff > GUARDTIME) {
            globalScheduleChange(schedId);
         }
         // same age but initialized by different nodes
         else if ((ageDiff >= -GUARDTIME && ageDiff <= GUARDTIME && 
            schedTab[schedId].syncNode < schedTab[0].syncNode)) {
            globalScheduleChange(schedId);
         }
      }
#endif
      //smacUartDebug_byte(numNeighb);
      signal LinkState.rxSyncPkt(packet->fromAddr, packet->seqNo);
   }

#ifdef GLOBAL_SCHEDULE
   void globalScheduleChange(uint8_t schedId)
   {
        uint8_t j;
        SchedTable tmpSched;
                                                                                                                                                            
        // change state
        schedState++;
        // fill in the field in my syncPkt
        syncPkt.state = schedState;
                                                                                                                                                            
        // Switch schedule entries
        tmpSched.counter = schedTab[schedId].counter;
        tmpSched.syncNode = schedTab[schedId].syncNode;
        tmpSched.txSync = schedTab[schedId].txSync;  // need send sync
        tmpSched.txData = schedTab[schedId].txData;
        tmpSched.numNodes = schedTab[schedId].numNodes ; 
        tmpSched.chkSched = schedTab[schedId].chkSched;
        tmpSched.age = schedTab[schedId].age;
        tmpSched.lastAgeUpdate = schedTab[schedId].lastAgeUpdate;                                                                                                                                                            
        if (schedTab[0].numNodes == 1) {
           numSched--;
        }
                                                                                                                                                            
        schedTab[schedId].counter = schedTab[0].counter;
        schedTab[schedId].syncNode = schedTab[0].syncNode;
        schedTab[schedId].txSync = schedTab[0].txSync;  // need send sync
        schedTab[schedId].txData = schedTab[0].txData;
        schedTab[schedId].numNodes = schedTab[0].numNodes - 1; // I switch schedule
        schedTab[schedId].chkSched = schedTab[0].chkSched;
        schedTab[schedId].age = schedTab[0].age;
        schedTab[schedId].lastAgeUpdate = schedTab[0].lastAgeUpdate;
                                                                                                                                                            
        // new schedule is schedule 0 now
        schedTab[0].counter = tmpSched.counter; //packet->sleepTime;
        schedTab[0].syncNode = tmpSched.syncNode;
        schedTab[0].txSync = 1;  // need send sync
        schedTab[0].txData = tmpSched.txData;
        schedTab[0].numNodes = tmpSched.numNodes + 1; // I are following this schedule
        schedTab[0].chkSched = tmpSched.chkSched;
        schedTab[0].age = tmpSched.age;
        schedTab[0].lastAgeUpdate = tmpSched.lastAgeUpdate;
                                                                                                                                                            
        // change all the neighbor who was following shedule 0 or schedId
        for (j = 0; j < SMAC_MAX_NUM_NEIGHB; j++) {
           if (neighbList[j].schedId == 0) {
              neighbList[j].schedId = schedId;
           }
           else if (neighbList[j].schedId == schedId) {
              neighbList[j].schedId = 0;
           }
        }
   }
                                                                                                                                                            
#endif

   void sendRTS()
   {
      // construct and send RTS packet
      ctrlPkt.type = (RTS_PKT << 4) + txFragAll;
      ctrlPkt.toAddr = sendAddr;
      // reserve time for CTS + all fragments + all ACKs
      ctrlPkt.duration = (txFragAll + 1) * durCtrlPkt + 
               txFragAll * durDataPkt + 
               (txFragAll + txFragAll + 1) * (PROC_DELAY + SIFS);
      // send RTS
      trace(DBG_USR1, "sendRTS()\r\n");
      call PhyComm.txPkt(&ctrlPkt, sizeof(ctrlPkt));
      radioState = RADIO_TX;
      state = TX_PKT;
      txErrTime = TX_PKT_DONE_TIME;
      smacUartDebug_state(state);
   }


   void sendCTS()
   {
      // construct and send CTS
      // input duration is the duration field from received RTS pkt
      ctrlPkt.type = CTS_PKT << 4;
      ctrlPkt.toAddr = recvAddr;
      // should track neighbors' NAV as soon as RTS is received
      ctrlPkt.duration = neighbNav - durCtrlPkt;
      // send CTS
      trace(DBG_USR1, "sendCTS()\r\n");
      call PhyComm.txPkt(&ctrlPkt, sizeof(ctrlPkt));
      txErrTime = TX_PKT_DONE_TIME;
      radioState = RADIO_TX;
   }


   void sendDATA()
   {
      // send a unicast data packet
      // assume all MAC header fields have been filled except the duration
/*
#ifdef SMAC_SNOOPER_DEBUG
      *((char*)dataPkt + sizeof(MACHeader) + 10) = numLenErr;
      *((char*)dataPkt + sizeof(MACHeader) + 11) = numCrcErr;
      *((char*)dataPkt + sizeof(MACHeader) + 12) = numSlpPkt;
      *((char*)dataPkt + sizeof(MACHeader) + 13) = numCTStimeout;
      *((char*)dataPkt + sizeof(MACHeader) + 14) = numDATAtimeout;
      *((char*)dataPkt + sizeof(MACHeader) + 15) = numACKtimeout;
	
      *((char*)dataPkt + sizeof(MACHeader) + 17) = numSleeps;
      *((char*)dataPkt + sizeof(MACHeader) + 18) = numWakeups;
      *((char*)dataPkt + sizeof(MACHeader) + 19) = neighbNav;
      *((char*)dataPkt + sizeof(MACHeader) + 20) = syncDiff1;
      *((char*)dataPkt + sizeof(MACHeader) + 21) = syncDiff2;
      *(uint16_t*)((char*)dataPkt + sizeof(MACHeader) + 22) = schedTab[0].counter;
      *(uint16_t*)((char*)dataPkt + sizeof(MACHeader) + 24) = schedTab[1].counter;
#endif
*/
#ifdef SMAC_TX_TIME_STAMP
      //XXX: /dataPkt->txTimeStamp = clockTime;
#endif
      // neighbNav tracks the time I have left for tx
      dataPkt->duration = neighbNav - durDataPkt;
      trace(DBG_USR1, "sendDATA()\r\n"); 
      call PhyComm.txPkt(dataPkt, txPktLen);
      radioState = RADIO_TX;
      txErrTime = TX_PKT_DONE_TIME;
   }


   void sendACK()
   {
      // construct and send ACK
      ctrlPkt.type = ACK_PKT << 4;
      ctrlPkt.toAddr = recvAddr;
      // stick to neighbNav -- should update it when rx data packet
      ctrlPkt.duration = neighbNav - durCtrlPkt;
      trace(DBG_USR1, "sendACK()\r\n");
      call PhyComm.txPkt(&ctrlPkt, sizeof(ctrlPkt));
      radioState = RADIO_TX;
      txErrTime = TX_PKT_DONE_TIME;
   }


   void sendSYNC()
   {
      // construct and send SYNC packet
#ifdef SMAC_SNOOPER_DEBUG
//      uint8_t i, numSchedCount, numNeighbCount;
      // For easy display, higher 4 bits are the number of schedules, and
      // lower 4 bits are the number of neighbors (max 15 can be shown)
      syncPkt.numSchedNeighb = (numSched << 4) + (numNeighb & 0xf);
/*      
      // check consistancy by counting actual number of schedules and neighbors
      numSchedCount = 0;
      for (i = 0; i < SMAC_MAX_NUM_SCHED; i++) {
         if (schedTab[i].numNodes > 0) numSchedCount++;
      }
      syncPkt.numSched = (numSched << 4) + (numSchedCount & 0xf);
      // count actual number of neighbors
      numNeighbCount = 0;
      for (i = 0; i < SMAC_MAX_NUM_NEIGHB; i++) {
         if (neighbList[i].state > 0) numNeighbCount++;
      }
      syncPkt.numNeighb = (numNeighb << 4) + (numNeighbCount & 0xf);
*/
      // display the number of times a Tx timeout occured
      syncPkt.txDataReset = numTxDataReset;
      syncPkt.txSyncReset = numTxSyncReset;
#endif
      syncPkt.seqNo = syncSeqNo;
#ifdef GLOBAL_SCHEDULE
      syncPkt.syncNode = schedTab[0].syncNode;
      syncPkt.age = schedTab[0].age + (clockTime - schedTab[0].lastAgeUpdate);
#endif
      syncPkt.sleepTime = schedTab[0].counter;
      trace(DBG_USR1, "sendSYNC()\r\n");
      call PhyComm.txPkt(&syncPkt, sizeof(syncPkt));
      radioState = RADIO_TX;
      state = TX_PKT;
      txErrTime = TX_PKT_DONE_TIME;
      smacUartDebug_state(state);
   }

#if 0
//XXX: foo
   // default internal time stamp.
   // To use external time stamp, provide wiring at application
   default command void TimeStamp.getTime32(uint32_t *timePtr)
   {
	  *timePtr = clockTime;
   }
#endif

   // default command if PowerManagement is not used
   default async command uint8_t PowerManagement.adjustPower()
   {
      return 1;
   }

   // default do-nothing handler for LinkState interface
   default event void LinkState.nodeJoin(uint16_t nodeAddr) { }
   default event void LinkState.nodeGone(uint16_t nodeAddr) { }
   default event void LinkState.rxSyncPkt(uint16_t fromAddr, uint8_t seqNo){ }
   default event void LinkState.rxBcastPkt(uint16_t fromAddr, uint8_t seqNo){ }
   default event void LinkState.rxUcastPkt(uint16_t fromAddr, uint8_t seqNo,
                                 uint8_t numTx, uint8_t numRx) { }

   // default do-nothing handler for MACTest interface
   default event void MACTest.MACSleep() { }
   default event void MACTest.MACWakeup() { }
   default event void MACTest.clockFire() { }


// Report functions
command uint32_t MACReport.getCrcErrors()
{
#ifdef SMAC_REPORT
	return crcErr;
#else
	return -1;
#endif
}

command uint32_t MACReport.getRetx()
{
#ifdef SMAC_REPORT
	return retx;
#else
	return -1;
#endif
}

command uint32_t MACReport.getLenErrors()
{
#ifdef SMAC_REPORT
	return lenErr;
#else
	return -1;
#endif
}



command uint32_t MACReport.getCtrlPkts()
{
#ifdef SMAC_REPORT
	return ctrlPkts;
#else
	return -1;
#endif
}


command uint8_t MACReport.getNumNeighb()
{
#ifdef SMAC_REPORT
	return numNeighb;
#else 
	return -1;
#endif
}


command uint8_t MACReport.getNumSched()
{
#ifdef SMAC_REPORT
	return numSched;
#else 
	return -1;
#endif
}


command uint8_t MACReport.getMacState()
{
#ifdef SMAC_REPORT
	return state;
#else 
	return -1;
#endif
}


command uint8_t MACReport.getRadioState()
{
#ifdef SMAC_REPORT
	return radioState;
#else 
	return -1;
#endif
}

command uint8_t MACReport.getTxState()
{
#ifdef SMAC_REPORT
	return txRequest;
#else
	return -1;
#endif
}

command uint16_t MACReport.getNav()
{
#ifdef SMAC_REPORT
	return nav;
#else
	return -1;
#endif
}

command uint16_t MACReport.getNeighbNav()
{
#ifdef SMAC_REPORT
	return neighbNav;
#else
	return -1;
#endif
}

command uint16_t MACReport.getSleepErrors()
{
#ifdef SMAC_REPORT
    return slpErr;
#else
    return -1;
#endif
}

command uint16_t MACReport.getMacTxErrors()
{
#ifdef SMAC_REPORT
    return macTxErr;
#else
    return -1;
#endif
}


   
}  // end of implementation
