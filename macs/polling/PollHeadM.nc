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
		interface CarrierSense;
		interface PhyComm;
		interface Timer;
	}
}

implementation
{
#include "PollMsg.h"

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

		PhyComm.idle();
	}

	default event result_t PollHeadComm.requestDataDone(uint8_t id, void *data, int error)
	{
		return SUCCESS;
	}

	task void rqDataFail()
	{
		signal PollHeadComm.requestDataDone(rxPkt->node_id, rxPkt, 1);
		atomic state = STATE_IDLE;
	}

	task void rqDataDone()
	{
		signal PollHeadComm.requestDataDone(rxPkt->node_id, rxPkt, 0);
		atomic state = STATE_IDLE;
	}

	task void sendAck()
	{
		atomic txPkt->node_id = node_id;
		txPkt->type = POLL_ACK;

		if ((call PhyComm.txPkt(txPkt, sizeof(txPkt))) == FAIL)
			post rqDataFail();

		return;
	}

	event result_t PhyControl.startDone()
	{
		atomic {
			radioState = RADIO_IDLE;
			switch (state) {
			case STATE_REQ_TX:
				if (PhyComm.txPkt(pkt, pkt_len) == FAIL)
					post rqDataFail();
				return SUCCESS;
				/* NOTREACHED */
			default:
				signal SplitControl.startDone();
			}
		}
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

	command result_t SplitControl.stopDone()
	{
		atomic radioState = STATE_SLEEP;
		signal SplitControl.stopDone();
		return SUCCESS;
	}

	command result_t PollHeadComm.requestData(uint8_t u_node_id, void *data, uint8_t length)
	{
		uint8_t chkState;
		PollMsg *pMsg;

		if ((chkState != STATE_IDLE) && (chkState != STATE_SLEEP))
			return FAIL;

		pkt = data;
		pkt_len = length;
		pkt->node_id = u_node_id;
		pkt->type = POLL_REQ;
		atomic node_id = u_node_id;

		if (chkState == STATE_SLEEP) {
			atomic state = STATE_REQ_TX;
			wakeup();
			return SUCCESS;
		}

		atomic state = STATE_REQ_TX;
		return PhyComm.txPkt(pkt, pkt_len);
	}

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
			}
		}
		return SUCCESS;
	}

	event result_t PhyComm.startSymDetected(void *data)
	{
		return SUCCESS;
	}

	event result_t PhyComm.rxPktDone(void *data, uint8_t error)
	{
		uint8_t chkState;

		atomic chkState = state;

		if ((chkState != STATE_DATA_WAIT) && (chkState != STATE_DISCOVERY_WAIT))
			return SUCCESS;
		
		rxPkt = data;
		if ((chkState == STATE_DATA_WAIT) && ((rxPkt->type != POLL_DATA) || (error))) {
			post rqDataFail;
			return SUCCESS;
		}

		if ((chkState == STATE_DATA_WAIT) && (rxPkt->node_id != node_id))
			return SUCCESS;

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

		return SUCCESS:
	}
}
