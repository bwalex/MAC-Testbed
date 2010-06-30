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
	provides {
		interface SplitControl;
		interface PollNodeComm;
	}
	uses {
		interface SplitControl as PhyControl;
		interface PhyState;
		interface CarrierSense;
		interface PhyComm;
		interface Timer;
		interface Leds;
	}
}

implementation
{
#include "PollMsg.h"

	enum {
		STATE_WAKEUP,
		STATE_SLEEP,
		STATE_IDLE,
		STATE_DATA_REQ,
		STATE_DATA_TX,
		STATE_WAIT_ACK,
	};

	enum {
		RADIO_SLEEP,
		RADIO_IDLE,
		RADIO_RX,
		RADIO_TX
	};

	uint8_t state;
	uint8_t radioState;
	uint8_t	flags;
	MACHeader txPkt;
	MACHeader *pkt;
	MACHeader *rxPkt;
	uint8_t pkt_len;
	uint8_t node_id;

	void wakeup()
	{
		atomic {
			if (radioState != RADIO_SLEEP)
				return;
		}
		atomic state = STATE_WAKEUP;
		
		PhyComm.idle();
	}

	void sleep(uint16_t ms)
	{
		if ((ms == 0) || ((ms -1) <= 1))
			return;

		call Timer.start(TIMER_ONE_SHOT, ms - 1);
		call SplitControl.stop();
		call Leds.yellowOff();
	}

	event result_t Timer.fired()
	{
		wakeup();
	}

	event result_t PhyControl.startDone()
	{
		atomic {
			radioState = RADIO_IDLE;
			switch (state) {
			case STATE_WAKEUP:
				break;
			default:
				signal SplitControl.startDone();
			}
		}
		call Leds.yellowOn();
		return SUCCESS;
	}

	command result_t SplitControl.init()
	{
		atomic state = 0xAA;
		return call PhyControl.init();
	}

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


	command result_t SplitControl.start()
	{
		return call PhyControl.start();
	}

	default event result_t SplitControl.startDone()
	{
		return SUCCESS;
	}

	command result_t SplitControl.stop()
	{
		atomic state = STATE_SLEEP;
		return call PhyControl.stop();
	}

	event result_t PhyControl.stopDone()
	{
		atomic radioState = STATE_SLEEP;
		signal SplitControl.stopDone();
		return SUCCESS;
	}

	default event result_t SplitControl.stopDone()
	{
		return SUCCESS;
	}

	default event result_t PollNodeComm.dataRequested()
	{
		return SUCCESS;
	}

	default event result_t PollNodeComm.ackReceived()
	{
		return SUCCESS;
	}

	default event result_t PollNodeComm.dataTxFailed()
	{
		return SUCCESS;
	}

	command result_t PollNodeComm.txData(void *data, uint8_t length)
	{
		uint8_t chkState;
		atomic chkState = state;
		
		if (chkState != STATE_DATA_REQ)
			return FAIL;

		atomic {
			txPkt = data;
			pkt_len = length;
		}

		if (call PhyComm.txPkt(txPkt, pkt_len) == FAIL)
			return FAIL;
		else {
			atomic state = STATE_DATA_TX;
		}
	}

	event result_t PhyComm.startSymDetected()
	{
		return SUCCESS;
	}

	event result_t PhyComm.txPktDone()
	{
	}

	event result_t PhyComm.rxPktDone(void *data, uint8_t error)
	{
		uint8_t chkState;
		atomic chkState = state;

		if (error)
			return SUCCESS;
	
		atomic rxPkt = data;

		if ((rxPkt->node_id != TOS_LOCAL_ADDRESS) && (rxPkt->node_id != POLL_BROADCAST_ID))
			return SUCCESS;

		if (chkState == STATE_IDLE) {
			switch(rxPkt->type) {
			case POLL_ADV_SLEEP:
				sleep(rxPkt->sleep_ms);
				/* We've been adviced to go to sleep for rxPkt->sleep_ms ms */
				break;
			case POLL_REQ:
				atomic state = STATE_DATA_REQ;
				signal PollNodeComm.dataRequested(data);
				break;
			}
		} else if (chkState == STATE_WAIT_ACK) {
			if (rxPkt->type != POLL_ACK)
				return SUCCESS;
			else
				PollNodeComm.ackReceived(data);
		}

		return SUCCESS;
	}


	event result_t PhyComm.txPktDone(void *data, uint8_t error)
	{
		atomic {
			switch(state) {
			case STATE_DATA_TX:
				if (error) {
					/* XXX: retry first? */
					signal PollNodeComm.dataTxFailed(data);
				} else {
					atomic state = STATE_DATA_WAIT;
				}
				break;
			default:
			}
		}
		return SUCCESS;
	}

	event result_t PhyComm.startSymDetected(void *data)
	{
		return SUCCESS;
	}
}
