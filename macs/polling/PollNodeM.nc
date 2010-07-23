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

module PollNodeM
{
	provides
	{
		interface SplitControl;
		interface PollNodeComm;
	}
	uses
	{
		interface SplitControl as PhyControl;
		interface PhyState;
		//interface CarrierSense;
		interface PhyComm;
		interface Timer as WakeupTimer;
		interface Leds;
		interface PrecisionTimer as Timestamp;
		interface StdControl as TSControl;
	}
}

implementation {
#include "PollMsg.h"

	enum
	{
		STATE_WAKEUP = 0,
		STATE_SLEEP,
		STATE_IDLE,
		STATE_DATA_REQ,
		STATE_DATA_TX,
		STATE_WAIT_ACK,
	};

	enum
	{
		RADIO_SLEEP,
		RADIO_IDLE,
		RADIO_RX,
		RADIO_TX
	};

	uint32_t timestamp;
	uint32_t sleep_interval;
	uint8_t state;
	uint8_t radioState;
	uint8_t flags;
	MACHeader txPkt;
	MACHeader *pkt;
	MACHeader *rxPkt;
	uint8_t pkt_len;
	uint16_t node_id;
	uint16_t head_id;

	/*
	 * We are supposed to wakeup. Get the current timestamp and see how long
	 * we really sleeped. Then set us into wakeup state and wake up the PHY.
	 */
	void wakeup()
	{
		uint32_t now_ts;
		uint8_t ret = 0;
		now_ts = call Timestamp.getTime32();
		//trace(DBG_USR1, "Timer fired (precision): Timestamp difference: %d\r\n", now_ts - timestamp);
		atomic {
			if (radioState != RADIO_SLEEP) {
				trace(DBG_USR1,
				      "wakeup() called, but radioState != RADIO_SLEEP, = %d\r\n",
				      radioState);
				ret = 1;
			}
		}

		if (ret)
			return;

		atomic state = STATE_WAKEUP;

		call PhyState.idle();
	}

	/*
	 * The Precision timer fired for wakeup, so we call the wakeup function
	 * (see above).
	 */
	async event result_t Timestamp.alarmFired(uint32_t val)
	{
		wakeup();
		return SUCCESS;
	}

	/*
	 * The sleep function puts the PHY to sleep for 'jfs' jiffies and sets
	 * up a timer to signal the wakeup.
	 */
	void sleep(uint32_t jfs)
	{
		if (jfs == 0)
			return;
#if 0
		timestamp = call Timestamp.getTime32();
		call WakeupTimer.start(TIMER_ONE_SHOT, ms - 1);
#else
		timestamp = call Timestamp.getTime32();
		//trace(DBG_USR1, "setting alarm to %d\r\n", timestamp + jfs);
		call Timestamp.setAlarm(timestamp + jfs);
#endif
		call SplitControl.stop();
		call Leds.yellowOff();
	}

	event result_t WakeupTimer.fired()
	{
		wakeup();
		return SUCCESS;
	}

	/*
	 * Startup is done (this is also called as callback from PhyState.idle).
	 * If we were waking up, see how much the wakeup took until radio was
	 * back to idle.
	 * Otherwise notify upper layer of the MAC start completion.
	 * Always set the radiostate and our state to idle.
	 */
	event result_t PhyControl.startDone()
	{
		uint32_t now_ts;
		atomic {
			switch (state) {
			case STATE_WAKEUP:
				now_ts = call Timestamp.getTime32();
				//trace(DBG_USR1, "Timestamp difference: %d\r\n", now_ts - timestamp);
				break;
			default:
				signal SplitControl.startDone();
			}
			state = STATE_IDLE;
			radioState = RADIO_IDLE;
		}
		call Leds.yellowOn();
		return SUCCESS;
	}

	/*
	 * Initialize the PHY and Precision Timer, as well as our state to an
	 * invalid state.
	 */
	command result_t SplitControl.init()
	{
		atomic sleep_interval = 0;
		atomic state = 0xAA;
		call TSControl.init();
		return call PhyControl.init();
	}

	/*
	 * When initialization completed, the radio is basically sleeping, until
	 * a startup sequence is started. Change our states accordingly.
	 */
	event result_t PhyControl.initDone()
	{
		atomic radioState = RADIO_SLEEP;
		atomic state = STATE_SLEEP;
		signal SplitControl.initDone();
		return SUCCESS;
	}

	default event result_t SplitControl.initDone() {
		return SUCCESS;
	}

	/*
	 * Startup precision timer and PHY.
	 */
	command result_t SplitControl.start()
	{
		call TSControl.start();
		return call PhyControl.start();
	}

	default event result_t SplitControl.startDone() {
		return SUCCESS;
	}

	/*
	 * Stopping the PHY means putting it to sleep.
	 */
	command result_t SplitControl.stop()
	{
		atomic state = STATE_SLEEP;
		return call PhyControl.stop();
	}

	event result_t PhyControl.stopDone()
	{
		atomic radioState = RADIO_SLEEP;
		signal SplitControl.stopDone();
		return SUCCESS;
	}

	default event result_t SplitControl.stopDone() {
		atomic {
			if (state == STATE_WAKEUP) {
				/*
				 * We have gone to sleep, but should be waking up already, so
				 * we deactivate idle sleeping.
				 */
				//trace (DBG_USR1, "Wakeup timer has fired before we managed to go to sleep...\r\n");
				sleep_interval = 0;
			}
		}
		return SUCCESS;
	}

	default event result_t PollNodeComm.dataRequested(void *data)
	{
		return SUCCESS;
	}

	default event result_t PollNodeComm.ackReceived(void *data)
	{
		return SUCCESS;
	}

	default event result_t PollNodeComm.dataTxFailed() {
		return SUCCESS;
	}

	/*
	 * We've been asked to send some data back to whoever requested the data
	 * in the first place.
	 */
	command result_t PollNodeComm.txData(void *data, uint8_t length)
	{
		uint8_t chkState;
		atomic chkState = state;

		/* If no data was requested, fail immediately */
		if (chkState != STATE_DATA_REQ)
			return FAIL;

		atomic {
			pkt = data;
			pkt_len = length;
		}
		pkt->src_id = TOS_LOCAL_ADDRESS;
		/*
		 * Set the destination id to the id of the cluster head, as seen
		 * in the data request.
		 */
		atomic pkt->dest_id = head_id;
		pkt->type = POLL_DATA;
		atomic state = STATE_DATA_TX;

		/* Finally (try to) send the packet or fail. */
		if (call PhyComm.txPkt(pkt, pkt_len) == FAIL) {
			atomic state = STATE_IDLE;
			return FAIL;
		}
	}

	event result_t PhyComm.startSymDetected(void *data)
	{
		return SUCCESS;
	}

	/*
	 * A packet has been successfully received. Check that the packet is
	 * intended for us and process accordingly.
	 */
	event void *PhyComm.rxPktDone(void *data, uint8_t error)
	{
		MACPkt *pMacPkt;
		uint8_t chkState;
		atomic chkState = state;

		if (error)
			return data;

		atomic rxPkt = data;
		/*
		 * PFLAGS+=-DMY_ADDRESS=N make imote2  (to set
		 * TOS_LOCAL_ADDRESS)
		 */
		//trace(DBG_USR1, "rxPktDone() marker 1, TOS_LOCAL_ADDRESS = %d, dest_id = %d, src_id = %d, pkt_type = %d\r\n", TOS_LOCAL_ADDRESS, rxPkt->dest_id, rxPkt->src_id, rxPkt->type);
		/*
		 * If the packet isn't a broadcast and not intended for us
		 * either, we just drop it.
		 */
		if ((rxPkt->dest_id != TOS_LOCAL_ADDRESS)
		    && (rxPkt->dest_id != POLL_BROADCAST_ID))
			return data;
		//trace(DBG_USR1, "rxPktDone() marker 2, chkState = %d\r\n", chkState);

		/*
		 * If we are waiting for an ACK that never arrived and we've
		 * been requested data, set us back to idle to prepare for
		 * sending data again, no matter if we received the ack before
		 * or not.
		 */
		if ((chkState == STATE_WAIT_ACK) && (rxPkt->type == POLL_REQ))
			atomic chkState = state = STATE_IDLE;

		//trace(DBG_USR1, "POLL_BEACON=%d, type=%d\r\n", POLL_BEACON, rxPkt->type);
		/*
		 * If the packet is a beacon, extract relevant information.
		 */
		if (rxPkt->type == POLL_BEACON) {
			pMacPkt = (MACPkt *) data;
			atomic sleep_interval = pMacPkt->sleep_jf;
			//trace(DBG_USR1, "beacon received, setting sleep interval to %d jiffies\r\n", pMacPkt->sleep_jf);
		}
		if (chkState == STATE_IDLE) {
			switch (rxPkt->type) {
				/*
				 * If we are idle and we received a data request, signal
				 * upper layer and change our state accordingly.
				 */
			case POLL_REQ:
				//trace(DBG_USR1, "rxPktDone() marker 4\r\n");
				atomic state = STATE_DATA_REQ;
				atomic head_id = rxPkt->src_id;
				signal PollNodeComm.dataRequested(data);
				break;
			default:
				//trace(DBG_USR1, "rxPktDone() marker 5, id=%d\r\n", rxPkt->type);
			}
		} else if (chkState == STATE_WAIT_ACK) {
			/* We've been waiting for an ACK. If the received packet
			 * is not an ACK, just drop it. Otherwise signal the
			 * upper layer about the ACK receipt.
			 */
			//trace(DBG_USR1, "rxPktDone() marker 6\r\n");
			if (rxPkt->type != POLL_ACK)
				return data;
			else {
				sleep(sleep_interval);	/* XXX: maybe should sleep after ackReceived() callback */
				signal PollNodeComm.ackReceived(data);
				//sleep(sleep_interval);
			}

		}
		//trace(DBG_USR1, "rxPktDone() marker 7\r\n");
		return data;
	}


	event result_t PhyComm.txPktDone(void *data, uint8_t error)
	{

		atomic {
			//trace(DBG_USR1, "txPktDone called with state = %d\r\n", state);
			switch (state) {
			case STATE_DATA_TX:
				if (error) {
					/* XXX: retry first? */
					signal PollNodeComm.dataTxFailed();
					state = STATE_IDLE;
				} else {
					atomic state = STATE_WAIT_ACK;
				}
				break;
			default:
			}
		}
		return SUCCESS;
	}

}
