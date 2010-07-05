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

module PollHeadM
{
	provides {
		interface SplitControl;
		interface PollHeadComm;
	}
	uses {
		interface SplitControl as PhyControl;
		interface PhyState;
		//interface CarrierSense;
		interface PhyComm;
		interface Timer as BeaconTimer;
	}
}

implementation
{
#include "PollMsg.h"
#define BEACON_INTERVAL	5500 /* every 5.5s */

	enum {
		STATE_SLEEP,
		STATE_IDLE,
		STATE_PRE_TX,
		STATE_REQ_TX,
		STATE_DATA_WAIT,
		STATE_DISCOVERY_WAIT,
		STATE_ACK_TX,
		STATE_TX_DONE
	};

	enum {
		RADIO_SLEEP,
		RADIO_IDLE,
		RADIO_RX,
		RADIO_TX
	};

	uint32_t timestamp;
	uint32_t sleep_interval;
	uint8_t state;
	uint8_t radioState;
	uint8_t	flags;
	MACPkt txPkt;
	MACHeader *pkt;
	MACHeader *rxPkt;
	uint8_t pkt_len;
	uint8_t node_id;

	void wakeup()
	{
		uint8_t ret = 0;
		atomic {
			if (radioState != RADIO_SLEEP)
				ret = 1;
		}

		if (ret)
			return;
		call PhyState.idle();
	}

	default event result_t PollHeadComm.requestDataDone(uint8_t id, void *data, uint8_t error)
	{
		return SUCCESS;
	}

	/* Fail a data request and get us back to idle */
	task void rqDataFail()
	{
		signal PollHeadComm.requestDataDone(rxPkt->src_id, (void *)rxPkt, 1);
		atomic state = STATE_IDLE;
	}

	/*
	 * Notify that a data request has been fulfilled (i.e. data received,
	 * ack sent.
	 */
	task void rqDataDone()
	{
		signal PollHeadComm.requestDataDone(rxPkt->src_id, (void *)rxPkt, 0);
		atomic state = STATE_IDLE;
	}

	/* Send an ACK to the current node_id */
	task void sendAck()
	{
		MACHeader *mh;

		atomic mh = (MACHeader *)&txPkt;
		atomic mh->dest_id = node_id;
		atomic mh->src_id = TOS_LOCAL_ADDRESS;
		atomic mh->type = POLL_ACK;

		if ((call PhyComm.txPkt(&txPkt, sizeof(txPkt))) == FAIL)
			post rqDataFail();

		return;
	}

	/* Send a Beacon to the whole cluster */
	void sendBeacon()
	{
		MACHeader *mh;
		MACPkt	*mp;

		trace(DBG_USR1, "beacon timer fired, sending beacon\r\n");

		atomic {
			mh = (MACHeader *)&txPkt;
			mp = (MACPkt *)&txPkt;
			mh->dest_id = POLL_BROADCAST_ID;
			mh->src_id = TOS_LOCAL_ADDRESS;
			mh->type = POLL_BEACON;
			mp->sleep_jf = sleep_interval;
		}
		

		if ((call PhyComm.txPkt(&txPkt, sizeof(txPkt))) == FAIL)
			trace(DBG_USR1, "Failed to send beacon :(\r\n");

		return;
	}

	/* 
	 * Send the sample start signal. One should wait for the sample send
	 * completion
	 */
	command result_t PollHeadComm.sendSampleStart()
	{
		MACHeader *mh;
		MACPkt	*mp;

		trace(DBG_USR1, "beacon timer fired, sending beacon\r\n");

		atomic {
			mh = (MACHeader *)&txPkt;
			mp = (MACPkt *)&txPkt;
			mh->dest_id = POLL_BROADCAST_ID;
			mh->src_id = TOS_LOCAL_ADDRESS;
			mh->type = POLL_SAMPLE;
		}
	
		if ((call PhyComm.txPkt(&txPkt, sizeof(txPkt))) == FAIL)
			trace(DBG_USR1, "Failed to send beacon :(\r\n");

		return SUCCESS;
	}

	/*
	 * Separate task to send the beacon.
	 */
	task void sendBeaconTask()
	{
		sendBeacon();
	}

	/*
	 * The beacon timer has fired, so it's time to send a beacon to the
	 * network. Try to post the sendBeacon() task, but if it isn't possible,
	 * try to send the beacon directly.
	 */
	event result_t BeaconTimer.fired()
	{
		if ((post sendBeaconTask()) == FAIL)
			sendBeacon();

		return SUCCESS;
	}

	/*
	 * Set the sleep interval for nodes. This is broadcast in the beacon
	 * packet.
	 */
	command result_t PollHeadComm.setSleepInterval(uint32_t sleep_jiffies)
	{
		atomic sleep_interval = sleep_jiffies;
		return SUCCESS;
	}

	/*
	 * The startup sequence is done. If we were woken up to send a packet,
	 * immediately send it. Otherwise set us to idle and start the beacon
	 * timer.
	 */
	event result_t PhyControl.startDone()
	{
		uint8_t chkState;

		atomic {
			chkState = state;
		}

		atomic radioState = RADIO_IDLE;
			
		switch (chkState) {
		case STATE_REQ_TX:
			if (call PhyComm.txPkt(pkt, pkt_len) == FAIL)
				post rqDataFail();
			return SUCCESS;
			/* NOTREACHED */
		default:
			signal SplitControl.startDone();
			atomic state = STATE_IDLE;
			call BeaconTimer.start(TIMER_REPEAT, BEACON_INTERVAL);
		}
		return SUCCESS;
	}

	/*
	 * Init by setting the state to an invalid state and initialize PHY.
	 */
	command result_t SplitControl.init()
	{
		atomic state = 0xAA;
		return call PhyControl.init();
	}

	/*
	 * Phy initialization is done. Until the startup sequence the radio is
	 * basically sleeping.
	 */
	event result_t PhyControl.initDone()
	{
		atomic radioState = RADIO_SLEEP;
		atomic state = STATE_SLEEP;
		signal SplitControl.initDone();
		return SUCCESS;
	}

	default event result_t SplitControl.initDone()
	{
		return SUCCESS;
	}

	/* On startup, also start the PHY */
	command result_t SplitControl.start()
	{
		return call PhyControl.start();
	}

	default event result_t SplitControl.startDone()
	{
		return SUCCESS;
	}

	/*
	 * Stopping means putting to sleep, so change state accordingly and call
	 * lower layer.
	 */
	command result_t SplitControl.stop()
	{
		atomic state = STATE_SLEEP;
		return call PhyControl.stop();
	}

	/*
	 * Stopping the PHY has completed, so change the radio state
	 * accordingly.
	 */
	event result_t PhyControl.stopDone()
	{
		atomic radioState = RADIO_SLEEP;
		signal SplitControl.stopDone();
		return SUCCESS;
	}

	default event result_t SplitControl.stopDone()
	{
		return SUCCESS;
	}

	/*
	 * Cancel the current request by setting an invalid node_id and putting
	 * us back into idle state.
	 */
	command result_t PollHeadComm.cancelRequest()
	{
		atomic node_id = 65534;
		atomic state = STATE_IDLE;
		return SUCCESS;
	}

	/*
	 * Command to request data from a single node. We basically wake up the
	 * radio if needed. The startDone event will then send the request for
	 * us, otherwise, if we were not sleeping, we send the request.
	 */
	command result_t PollHeadComm.requestData(uint8_t u_node_id, void *data, uint8_t length)
	{
		uint8_t chkState;

		atomic chkState = state;

		if ((chkState != STATE_IDLE) && (chkState != STATE_SLEEP)) {
			trace(DBG_USR1, "chkState = %d, hence failing\r\n", chkState);
			return FAIL;
		}

		pkt = data;
		pkt_len = length;
		pkt->dest_id = u_node_id;
		pkt->src_id = TOS_LOCAL_ADDRESS;
		pkt->type = POLL_REQ;
		atomic node_id = u_node_id;

		if (chkState == STATE_SLEEP) {
			atomic state = STATE_REQ_TX;
			wakeup();
			return SUCCESS;
		}

		atomic state = STATE_REQ_TX;
		return call PhyComm.txPkt(pkt, pkt_len);
	}

	/*
	 * Transmitting a packet completed. If we were sending a data request,
	 * put us into data_wait state.
	 * If we were sending an ACK, signal either failure or completion of the
	 * data request via the relevant tasks.
	 */
	event result_t PhyComm.txPktDone(void *data, uint8_t error)
	{
		atomic {
			switch(state) {
			case STATE_REQ_TX:
				if (error) {
					post rqDataFail();
				} else {
					atomic state = STATE_DATA_WAIT;
				}
				break;

			case STATE_ACK_TX:
				if (error) {
					post rqDataFail();
				} else {
					post rqDataDone();
				}
				break;

			default:
				trace(DBG_USR1, "sendSampleStartDone() assumed in txPktDone.\r\n");
				signal PollHeadComm.sendSampleStartDone();
				break;
			}
		}
		return SUCCESS;
	}

	event result_t PhyComm.startSymDetected(void *data)
	{
		return SUCCESS;
	}

	/*
	 * We've successfully received a packet. Now we check if the request was
	 * cancelled. If so, we just ignore it, otherwise we do a few more
	 * sanity checks and then send the ACK if appropriate.
	 */
	event void *PhyComm.rxPktDone(void *data, uint8_t error)
	{
		uint8_t chkState;

		atomic chkState = state;

		/* Was the last request cancelled? If so, drop the packet */
		if (node_id == 65534)
			return data;

		/*
		 * If we are not waiting for data or discovery, drop the packet
		 */
		if ((chkState != STATE_DATA_WAIT) && (chkState != STATE_DISCOVERY_WAIT))
			return data;
		
		rxPkt = data;
		/*
		 * If we are in data wait state but the received packet is not
		 * of the data type or an error occured, post the data request
		 * fail task.
		 */
		if ((chkState == STATE_DATA_WAIT) && ((rxPkt->type != POLL_DATA) || (error))) {
			post rqDataFail();
			return data;
		}

		/*
		 * If we are waiting for data but the data we received is from a
		 * node we haven't requested any data from recently, just drop
		 * it.
		 */
		if ((chkState == STATE_DATA_WAIT) && (rxPkt->src_id != node_id))
			return data;

		switch (rxPkt->type) {
		case POLL_DATA:
			atomic state = STATE_ACK_TX;
			post sendAck();
			break;
		case POLL_PRESENT:
			if (chkState != STATE_DISCOVERY_WAIT) {
#if 0
				post rqDiscoveryFail();
				return SUCCESS;
				/* XXX: not implemented */
#endif
			}
			break;
		default:
		}

		return data;
	}
}
