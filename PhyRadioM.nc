/*
 * XXX: missing copyright notices.
 */

/* NOTE: this implementation is absolutely CC2420 specific!! */
/* @author Alex Hornung <ahornung@gmail.com> */

module PhyRadioM
{
	provides {
		interface SplitControl;
		interface PhyState;
		interface PhyComm;
		interface CarrierSense;
		interface SignalStrength;
	}
	uses {
		interface SplitControl as CC2420SplitControl;
		interface CC2420Control;
		interface HPLCC2420Capture as SFD;
		interface HPLCC2420Interrupt as FIFOP;
		interface HPLCC2420FIFO as FIFO;
		interface HPLCC2420 as HPL;
	}
}

implementation
{
	/* PHY states */
	enum {
		STATE_STARTUP = 0,
		STATE_IDLE, /* 1 */
		STATE_TRANSMITTING, /* 2 */
		STATE_TRANSMITTING_WAIT, /* 3 */
		STATE_TRANSMITTING_DONE,
		STATE_SLEEP,
		STATE_DEEP_SLEEP
	};

#include "PhyRadioMsg.h"
#define FLAG_RECV		0x01
#define FLAG_RXBUF_BUSY	0x02
#define FLAG_CCA		0x04
#define RSSI_OFFSET		-45

/*
 * Comment this out if you don't want debug information to be printed over the
 * serial port. Keep in mind that debug prints have a major overhead.
 */
//#define PHYRADIO_DEBUG

#ifdef PHYRADIO_DEBUG
 #define DBG_OUT		trace
#else
 #define DBG_OUT		while(0) trace
#endif

	uint8_t state;
	//uint8_t stateLock;
	uint8_t flags;
	uint8_t	lastPower;
	uint16_t lastFreq;

	PhyPktBuf pktBuf;
	uint8_t *txBuf;
	uint8_t *rxBuf;
	uint8_t txBufSz;
	uint8_t rxBufSz;

	/* Set the CCA mode to '1', refer to datasheet */
	static inline result_t setCCAMode()
	{
		uint16_t reg;
		reg = call HPL.read(CC2420_MDMCTRL0);
		reg |= (1 << CC2420_MDMCTRL0_CCAMODE);
		return call HPL.write(CC2420_MDMCTRL0, reg);
	}

	/* Flush the RXFIFO (twice) and clear the receive flag */
	static inline void flushRXFIFO() {
		/* disable interrupts while we flush */
		call FIFOP.disable();
		call HPL.read(CC2420_RXFIFO);          //flush Rx fifo
		call HPL.cmd(CC2420_SFLUSHRX);
		call HPL.cmd(CC2420_SFLUSHRX);
		atomic {
			flags &= ~FLAG_RECV;
		}
		/* reenable interrupts */
		call FIFOP.startWait(FALSE);
	}

	/* load our default config, as specified below */
	static void reloadConfig()
	{
		/*
		 * Defaults are:
		 *  - AutoACK disabled
		 *  - AddrDecode disabled
		 *  - CRC enabled (?)
		 */
		call CC2420Control.SetRFPower(lastPower);
		call CC2420Control.disableAutoAck();
		call CC2420Control.disableAddrDecode();
		call CC2420Control.TuneManual(lastFreq);
	}

	command result_t SplitControl.init()
	{
		atomic {
			flags = 0; /* Clear all flags */
			state = 0xAA; /* set an invalid state to catch problems */
			lastPower = 31; /* Default to maximum power */
			lastFreq = 2440; /* Default to frequency 2440 MHz */
		}
		trace(DBG_USR1, "Version 0.2 of PhyRadioM started, LOCAL_ADDR = %d\r\n", TOS_LOCAL_ADDRESS);
		return call CC2420SplitControl.init();
		/* call init in the underlying layers */
	}

	event result_t CC2420SplitControl.initDone() {
		atomic {
			state = STATE_SLEEP;
		}
		DBG_OUT(DBG_USR1, "initDone called\r\n");
		reloadConfig();
		return signal SplitControl.initDone();
	}

	default event result_t SplitControl.initDone() {
		return SUCCESS;
	}

	command result_t SplitControl.start()
	{
		uint8_t chkState;

		DBG_OUT(DBG_USR1, "SplitControl.start() called\r\n");
		atomic {
			chkState = state;
		}

		if (chkState != STATE_SLEEP) {
			return FAIL;
		}
		atomic {
			state = STATE_STARTUP;
		}
		DBG_OUT(DBG_USR1, "calling CC2420SplitControl.start()\r\n");
		return call CC2420SplitControl.start();
	}

	event result_t CC2420SplitControl.startDone() {
		uint8_t chkState;

		DBG_OUT(DBG_USR1, "starDone() called in PhyRadioM\r\n");
		atomic {
			chkState = state;
		}
		if (chkState == STATE_STARTUP) {
			DBG_OUT(DBG_USR1, "startDone with STATE_STARTUP called\r\n");
			/* set radio to receive mode */
			call CC2420Control.RxMode();
			/* enable interrupts */
			call FIFOP.startWait(FALSE);
			/* enable SFD capture (for RX at this point) */
			call SFD.enableCapture(TRUE);
			/* set the default CCA mode */
			setCCAMode();
			atomic {
				state = STATE_IDLE;
			}
		}

		signal SplitControl.startDone();
		return SUCCESS;
	}

	default event result_t SplitControl.startDone() {
		return SUCCESS;
	}

	command result_t SplitControl.stop()
	{
		DBG_OUT(DBG_USR1, "SplitControl.stop() called!\r\n");
		call SFD.disable();
		call FIFOP.disable();
		atomic {
			state = STATE_SLEEP;
		}
		return call CC2420SplitControl.stop();
	}

	event result_t CC2420SplitControl.stopDone() {
		return signal SplitControl.stopDone();
	}

	default event result_t SplitControl.stopDone() {
		return SUCCESS;
	}

	command result_t PhyState.idle()
	{
		uint8_t chkState;
	
		atomic {
			chkState = state;
		}
		DBG_OUT(DBG_USR1, "PhyState.idle() called\r\n");
		/* If we are asked to return to idle from sleep, wake us up */
		switch (chkState) {
		case STATE_SLEEP:
			return call SplitControl.start();
			/* NOTREACHED */

		default:
			/* do nothing */
		}

		return FAIL;
	}

	command result_t PhyState.sleep()
	{
		/*
		 * the underlying stop will put us to sleep completely (including the
		 * voltage regulator).
		 */
		DBG_OUT(DBG_USR1, "PhyState.sleep() called\r\n");
		call SplitControl.stop();
	}

	/****************************************************************************/
	
	/* fail a transmit and notify the consumer */
	inline void txFail(uint8_t *pkt) {
		DBG_OUT(DBG_USR1, "txFail() started\r\n");
		atomic {
			state = STATE_IDLE;
		}
		DBG_OUT(DBG_USR1, "txFail() called\r\n");
		signal PhyComm.txPktDone(pkt, 1);
	}

	/*
	 * fail a receive (only after notifying the start symbol) and notify the
	 * consumer
	 */
	task void rxFail() {
		DBG_OUT(DBG_USR1, "rxFail() started \r\n");
		signal PhyComm.rxPktDone(rxBuf, 1);
		atomic {
			flags &= ~FLAG_RECV;
		}
		DBG_OUT(DBG_USR1, "rxFail() called\r\n");
	}

	/*
	 * a packet has been fully received; clear the receiving flag and notify the
	 * consumer.
	 */
	task void rxPktDone() {
		DBG_OUT(DBG_USR1, "rxPktDone() started\r\n");
		signal PhyComm.rxPktDone(rxBuf, 0);
		atomic {
			flags &= ~FLAG_RECV;
		}
		DBG_OUT(DBG_USR1, "rxPktDone() called\r\n");
	}

	/*
	 * a packet has been sent successfully. get us back into the idle state and
	 * notify the consumer.
	 */
	task void txPktDone() {
		DBG_OUT(DBG_USR1, "txPktDone() started\r\n");
		signal PhyComm.txPktDone(txBuf, 0);
		atomic {
			state = STATE_IDLE;
		}
		DBG_OUT(DBG_USR1, "txPktDone() called\r\n");
	}

	/*
	 * Task to start transmitting a packet by flushing the buffers if needed to
	 * get the radio into a useful state, and then write the buffer into the
	 * transmit FIFO.
	 */
	task void txPkt() {
		uint8_t status;

		/* If needed, flush the RXBUF to get the radio into a good state */
		if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())) {
			flushRXFIFO();
			DBG_OUT(DBG_USR1, "txPkt() task: flush RX FIFO\r\n");
		}

		/* Flush the TXFIFO to make sure there's no mess in there */
		if (!(call HPL.cmd(CC2420_SFLUSHTX))) {
			txFail(txBuf);
			DBG_OUT(DBG_USR1, "txPkt() task: SFLUSHTX failed\r\n");
			return;
		}

		DBG_OUT(DBG_USR1, "txPkt() task: writing %d bytes to TXFIFO\r\n", txBufSz);
		DBG_OUT(DBG_USR1, "txPkt() task: first byte of txBuf is = %d\r\n", *((uint8_t *)txBuf));

		/* Write the buffer into the TXFIFO */
		if (!(call FIFO.writeTXFIFO(txBufSz, txBuf))) {
			txFail(txBuf);
			DBG_OUT(DBG_USR1, "txPkt() task: writeTXFIFO failed\r\n");
			return;
		}
	}

	/*
	 * Task to receive a packet. If a receive is already in progress, we
	 * schedule this receive for later again, if possible. If not, we just flush
	 * it.
	 * Afterwards we start reading the RXFIFO.
	 */
	task void rxPkt() {
		uint8_t chkFlags;

		/* RXFIFO overflow */
		if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())) {
			flushRXFIFO();
			DBG_OUT(DBG_USR1, "rxPkt() task: flush RX FIFO\r\n");
		}

		atomic {
			chkFlags = flags;
		}
		/*
		 * Receive is in progress. try to schedule this receive for later or
		 * fail.
		 */
		if (chkFlags & FLAG_RECV) {
			DBG_OUT(DBG_USR1, "rxPkt() task: post rxPkt\r\n");
			if (!post rxPkt())
				flushRXFIFO();
		} else {
			DBG_OUT(DBG_USR1, "rxPkt() task: set FLAG_RECV\r\n");
			atomic {
				flags |= FLAG_RECV;
			}
		}
		

		/* If no receive was previously active... */
		if (!(chkFlags & FLAG_RECV)) {
			/*
			 * ... try to read RXFIFO. If we fail, try to schedule the receive
			 * again for later or fail.
			 */
			rxBuf = (void *)&pktBuf;
			rxBufSz = sizeof(pktBuf);
			/*
			 * readRXFIFO will read at most rxBufSz bytes, but not necessarily
			 * exactly that amount, it could be less if the packet that was
			 * received is smaller.
			 */
			DBG_OUT(DBG_USR1, "rxPkt() task: call readRXFIFO\r\n");

			if (!call FIFO.readRXFIFO(rxBufSz, rxBuf)) {
				DBG_OUT(DBG_USR1, "rxPkt() task: readRXFIFO failed!\r\n");
				atomic {
					flags &= ~FLAG_RECV;
				}
				if (!post rxPkt()) {
					DBG_OUT(DBG_USR1, "could not post rxPkt!!!!!!!!!!!!!\r\n");
					flushRXFIFO();
				}
				return;
			}
		}
		/*
		 * XXX: this flush will be trouble if readRXFIFO becomes split phase as
		 * it happens in TinyOS 2.X.
		 */
		flushRXFIFO(); 
	}

	void sendpkt()
	{
		uint8_t status;

		DBG_OUT(DBG_USR1, "sendpkt(): going into TXON mode (STXON)\r\n");
		/* If necessary, return radio to good state by flushing RXFIFO */
		if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())) {
			DBG_OUT(DBG_USR1, "sendpkt(): flushRXFIFO() again\r\n");
			flushRXFIFO();
		}

		/*
		 * Set radio into transmit mode so it starts transmitting the data in
		 * the TXFIFO.
		 */
		call HPL.cmd(CC2420_STXON); /* XXX */

		/*
		 * Check the status byte. This is not strictly necessary when using
		 * STXON instead of STXONCCA, but we do it anyways, just to make sure
		 * the send is really active and no TX FIFO underflow occured.
		 */
		status = call HPL.cmd(CC2420_SNOP);
		DBG_OUT(DBG_USR1, "sendpkt(): status from CC2420_SNOP = %d\r\n", status);
		if ((status >> CC2420_TX_UNDERFLOW) & 0x01) {
			DBG_OUT(DBG_USR1, "sendpkt(): tx fifo underflow, flushing and failing\r\n");
			call HPL.cmd(CC2420_SFLUSHTX);
			txFail(txBuf);
			return;
		}
		if ((status >> CC2420_TX_ACTIVE) & 0x01) {
			/*
			 * XXX: enable SFD capture to see when send finishes... check
			 * datasheet again anyways
			 */
			DBG_OUT(DBG_USR1, "sendpkt(): enabling SFD, TX_ACTIVE set\r\n");
			/* capture rising edge */
			call SFD.enableCapture(TRUE);
		} else {
			DBG_OUT(DBG_USR1, "sendpkt(): status no good, fail TX\r\n");
			txFail(txBuf);
		}
	
	}

	command result_t PhyComm.reTxPkt()
	{
		sendpkt();
	}

	/* Command to transmit a single packet */
	command result_t PhyComm.txPkt(void *pkt, uint8_t pkt_sz)
	{
		uint8_t chkState;

		DBG_OUT(DBG_USR1, "PhyComm.txPkt(): called\r\n");
		/* Check if the packet is out of bounds, and if so, fail. */
		if (pkt_sz > PHY_MAX_PKT_LEN || pkt_sz < PHY_MIN_PKT_LEN) {
			DBG_OUT(DBG_USR1, "PhyComm.txPkt(): pkt_sz is out of bounds: %d\r\n", pkt_sz);
			return FAIL;
		}
		atomic {
			chkState = state;
		}
		/*
		 * If a transmit is already active, fail. We cannot handle simultaneous
		 * sends.
		 */
		if ((chkState != STATE_IDLE) && (chkState != STATE_TRANSMITTING_DONE)) {
			DBG_OUT(DBG_USR1, "PhyComm.txPkt(): not idle or tx_done, failing, but in state=%d\r\n", chkState);
			return FAIL;
		} else {
			DBG_OUT(DBG_USR1, "PhyComm.txPkt(): going into STATE_TRANSMITTING\r\n");
			/* Set us into transmit state */
			atomic {
				state = STATE_TRANSMITTING;
			}
		}

		/* Set up the txBuf and its size */
		atomic {
			txBuf = (uint8_t *)pkt;
			txBufSz = pkt_sz;
			/*
			 * length is pkt_sz-1, since the length field is not included in the
			 * length calculation.
			 */
			((PhyHeader *)txBuf)->length = pkt_sz-1;
		}
		DBG_OUT(DBG_USR1, "PhyComm.txPkt(): post txPkt()\r\n");
		/* post the actual transmit task */
		if ((post txPkt()) == FAIL) {
			DBG_OUT(DBG_USR1, "PhyComm.txPkt(): post txPkt() failed!\r\n");
			atomic {
				state = STATE_IDLE;
			}
			return FAIL;
		}
	
		return SUCCESS;
	}

	/*
	 * Notification that the TXFIFO write has completed. Once this happens, we
	 * just need to make sure no overflow occured and then send the packet.
	 */
	async event result_t FIFO.TXFIFODone(uint8_t length, uint8_t *data) {
		sendpkt();
		return SUCCESS;
	}


	/* From datasheet:
	 * The SFD pin goes active when the SFD
	 * field has been completely transmitted. It
	 * goes inactive again when the complete
	 * MPDU (as defined by the length field) has
	 * been transmitted or if an underflow is
	 * detected. See the RF Data Buffering
	 * section on page 39 for more information
	 * on TXFIFO underflow.
	 *
	 * in HPLCC2420M.nc, reactivate the code in CaptureSFD.enableCapture!!!
	 *
	 * Apart from that, the SFD interrupt also goes high when a packet is
	 * inbound and the SFD has been reached.
	 */		
	async event result_t SFD.captured(uint16_t time) {
		uint8_t chkState;

	  atomic {
		chkState = state;
		
		DBG_OUT(DBG_USR1, "SFD.captured(): called, chkState = %d\r\n", chkState);
		switch (chkState) {
		/*
		 * If we are transmitting, this means that the SFD field has already
		 * been completely transmitted.
		 */
		case STATE_TRANSMITTING:
			/* the SFD field has been completely transmitted */
			DBG_OUT(DBG_USR1, "SFD.captured(): STATE_TRANSMITTING\r\n");
			/* capture falling edge only */
			call SFD.enableCapture(FALSE);
			/*
			 * Check if it also fell already again, notifying that the sent has
			 * finished completely.
			 */
			if (!TOSH_READ_CC_SFD_PIN()) {
				DBG_OUT(DBG_USR1, "SFD.captured(): SFD pin fell already\r\n");
				call SFD.disable();
			} else {
				/* If it didn't fall already, put us into xmit wait */
				atomic {
					DBG_OUT(DBG_USR1, "SFD.captured(): going into STATE_TRANSMITTING_WAIT\r\n");
					state = STATE_TRANSMITTING_WAIT;
				}
				break;
			}
			/* FALLTHROUGH when SFD already fell */
	
		case STATE_TRANSMITTING_WAIT:
			/*
			 * If we have been waiting or the packet has been sent completely
			 * already without going through the tx wait state, we want to
			 * enable the SFD interrupt for receive again and notify the
			 * consumer.
			 */
			DBG_OUT(DBG_USR1, "SFD.captured(): STATE_TRANSMITTING_WAIT\r\n");
			//atomic state = STATE_TRANSMITTING_DONE;
			call SFD.disable();
			atomic state = STATE_TRANSMITTING_DONE;
			/* capture rising edge only */
			//call SFD.enableCapture(TRUE); /* recv SFD capture */
			/*
			 * Try and notify the consumer via a task. If not possible, signal
			 * directly.
			 * We are put back into idle state either after signalling directly
			 * or by the txPktDone() task after it notifies the consumer.
			 */
			if ((post txPktDone()) == FAIL) {
				DBG_OUT(DBG_USR1, "SFD.captured(): post txPktDone() failed\r\n");
				signal PhyComm.txPktDone(txBuf, 0);
				atomic {
					state = STATE_IDLE;
				}
			}
			if (!TOSH_READ_CC_SFD_PIN()) {
				/*
				 * if the SFD is still low, we finish here, otherwise there was
				 * another interrupt.
				 */
				call SFD.enableCapture(TRUE);
				break;
			}
			DBG_OUT(DBG_USR1, "SFD.captured(): FALLTHROUGH after setting TXDONE state\r\n");
			/* FALLTHROUGH when SFD is high again */
		default: /* XXX: this should be default: */
			/*
			 * No transmit is active, so the SFD signals the detection of the
			 * SFD frame in the incoming packet. Notify the user that we are
			 * about to receive a packet.
			 */
#if 0
			DBG_OUT(DBG_USR1, "SFD.captured(): NOOOOOOOOOOT calling startSymDetected on rxBuf\r\n");
#else
			DBG_OUT(DBG_USR1, "SFD.captured(): call startSymDetected on rxBuf\r\n");
			signal PhyComm.startSymDetected(rxBuf); 
#endif
			/*
			 * XXX: Ok, here is the big problem: whenever we send, we get called
			 * again (SFD.captured() in state TRANSMITTING_DONE. No idea why,
			 * but it's not an RX SFD interrupt.
			 */
			call SFD.enableCapture(TRUE);
			break;
		//default:
		}
	  }
		return SUCCESS;
	}

	/*
	 * The FIFO interrupt has fired, which happens when an incoming packet is
	 * ready to be read from the RXFIFO or an RXFIFO overflow occured.
	 */
	async event result_t FIFOP.fired() {
		DBG_OUT(DBG_USR1, "FIFOP.fired(): !\r\n");
		/* If the RXFIFO has overflown, flush the fifo */
		if (!TOSH_READ_CC_FIFO_PIN()) { /* OVERFLOW */
			DBG_OUT(DBG_USR1, "FIFOP.fired(): flushRXFIFO !\r\n");
			flushRXFIFO();
			if (!post rxFail()) {
				signal PhyComm.rxPktDone(rxBuf, 1);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}
			return SUCCESS;
		}

#if 0
		/* XXX: this SHOULD NOT BE HERE!!!!!!!!!!!!!!!!!!!!!!!! */
		signal PhyComm.startSymDetected(rxBuf);
		/* XXX: the above line REALLY SHOULD NOT BE HERE !!!!! */
#endif

		/*
		 * Start receiving a packet in the separate task we got for this. If
		 * it's not possible, flush the RXFIFO and forget about it.
		 */
		
		if (post rxPkt()) {
			DBG_OUT(DBG_USR1, "FIFOP.fired(): rxPkt posted, disable FIFOP\r\n");
			call FIFOP.disable();
		} else {
			DBG_OUT(DBG_USR1, "FIFOP.fired(): flushRXFIFO, post rxPkt() failed\r\n");
			flushRXFIFO();
			if (!post rxFail()) {
				signal PhyComm.rxPktDone(rxBuf, 1);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}
		}

		return SUCCESS;
	}

	/*
	 * The read of the RXFIFO has finished. The overall aim is to check for
	 * errors such as a CRC error or overflow, and finally notify the consumer
	 */
	async event result_t FIFO.RXFIFODone(uint8_t length, uint8_t *data) {
		uint8_t chkState;
		PhyPktBuf *pBuf;

		atomic {
			chkState = state;
		}

		/* RXFIFO overflow */
		if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())) {
			if (!post rxFail()) {
				signal PhyComm.rxPktDone(rxBuf, 1);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}

			return SUCCESS;
		}
	
		/* XXX: possibly signal error to upper layer */
		/* packet isn't of the right size, flush RXFIFO and end recv */
		if ((length < PHY_MIN_PKT_LEN) || (length > PHY_MAX_PKT_LEN)) {
			DBG_OUT(DBG_USR1, "length in RXFIFODone() is invalid, failing...\r\n");
			if (!post rxFail()) {
				signal PhyComm.rxPktDone(rxBuf, 1);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}

			return SUCCESS;
		}

		rxBuf = data; /* XXX: ? this should be done already. */
		pBuf = (PhyPktBuf *)rxBuf;	
		/*
		 * NOTE: note that we basically receive everything from frame length on:
		 * frame length till FCS (FCS is actually checksum, etc)
		 */
		/*
		 * try to post the rxDone notification. If we can't, signal up directly.
		 */
		pBuf->info.strength = data[length-2];
		/* NOTE: CRC is just OK/ NOT OK (1 / 0) */
		pBuf->crc = data[length-1] >> 7;
		DBG_OUT(DBG_USR1, "crc = %d, info.strength=%d\r\n", pBuf->crc, pBuf->info.strength);
		if (pBuf->crc == 0) {
			/* XXX: possibly move this into a rxPktFail() */
			if (!post rxFail()) {
				signal PhyComm.rxPktDone(rxBuf, 1);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}
		} else {
			/*
			 * XXX: link quality index is at data[length-1] & 0x7F
			 */
			if (!post rxPktDone()) {
				signal PhyComm.rxPktDone(rxBuf, 0);
				atomic {
					flags &= ~FLAG_RECV;
				}
			}
		}
		/* If there's more to read, read the next packet */
		if (!(TOSH_READ_CC_FIFOP_PIN())) {
			if (post rxPkt())
				return SUCCESS;
		}

		flushRXFIFO();
		return SUCCESS;
	}

	default event result_t PhyComm.txPktDone(void *pkt, uint8_t error)
	{
		return SUCCESS;
	}

	default event result_t PhyComm.startSymDetected(void *pkt)
	{
		return SUCCESS;
	}

	default event void *PhyComm.rxPktDone(void *pkt, uint8_t error)
	{
		return pkt; /* XXX? */
	}

	/*
	 * Poll the CCA pin to see if the channel is clear. Notify the consumer
	 * accordingly.
	 */
	command result_t CarrierSense.start()
	{
		uint8_t chkFlags;

		atomic {
			chkFlags = flags;
		}

		if (chkFlags & FLAG_CCA)
			return FAIL;

		atomic {
			flags |= FLAG_CCA;
		}

		DBG_OUT(DBG_USR1, "CarrierSense.start() - 1\r\n");
		if (TOSH_READ_RADIO_CCA_PIN()) {
			signal CarrierSense.channelIdle();
		} else {
			signal CarrierSense.channelBusy();
		}
		DBG_OUT(DBG_USR1, "CarrierSense.start() - 2 (done)\r\n");
		atomic {
			flags &= ~FLAG_CCA;
		}

		return SUCCESS;
	}

	/*
	 * Set the threshold of the CCA decision. Refer to the datasheet for
	 * information on the threshold setting. It supposedly is in dBm.
	 */
	command result_t CarrierSense.setThreshold(int8_t thr)
	{
		uint16_t reg;
		thr -= RSSI_OFFSET;
		reg = call HPL.read(CC2420_RSSI);
		reg |= (thr << CC2420_RSSI_CCA_THRESH);
		return call HPL.write(CC2420_RSSI, reg);
	}

	default event result_t CarrierSense.channelIdle()
	{
		return SUCCESS;
	}

	default event result_t CarrierSense.channelBusy()
	{
		return SUCCESS;
	}

	/* Returned RSSI value is in dBm */
	command int8_t SignalStrength.getRSSI()
	{
		uint16_t reg;
		int8_t rssi;

		reg = call HPL.read(CC2420_RSSI);
		rssi = (int8_t)(reg & 0xff);

		rssi += RSSI_OFFSET;

		return rssi;
	}
}
