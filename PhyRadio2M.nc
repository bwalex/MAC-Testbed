/*
 * XXX: missing copyright notices.
 */

/* NOTE: this implementation is absolutely CC2420 specific!! */

module PhyRadio2M
{
	provides {
		interface SplitControl;
		interface PhyState;
		interface PhyComm;
		interface CarrierSense;
		interface SignalStrength;
		interface BackoffControl;
		interface PhyRadioControl;
	}
  uses {
    interface SplitControl as CC2420SplitControl;
    interface CC2420Control;
    interface HPLCC2420 as HPLChipcon;
    interface HPLCC2420FIFO as HPLChipconFIFO; 
    interface HPLCC2420Interrupt as FIFOP;
    interface HPLCC2420Capture as SFD;
    interface StdControl as TimerControl;
    interface TimerJiffyAsync as BackoffTimerJiffy;
    interface Random;
    interface Leds;
  }
}

implementation {
#include "PhyRadioMsg.h"
  enum {
    DISABLED_STATE = 0,
    DISABLED_STATE_STARTTASK,
    IDLE_STATE,
    TX_STATE,
    TX_WAIT,
    PRE_TX_STATE,
    POST_TX_STATE,
    POST_TX_ACK_STATE,
    RX_STATE,
    POWER_DOWN_STATE,
    WARMUP_STATE,

    TIMER_INITIAL = 0,
    TIMER_BACKOFF,
    TIMER_ACK
  };

#define MAX_SEND_TRIES 14
#define FLAG_CCA			0x04
#define FLAG_BACKOFF		0x08
#define FLAG_TXONCCA		0x10
#define FLAG_BACKOFF_RANDOM	0x20

#define RSSI_OFFSET		-45


  norace uint8_t countRetry;
  uint8_t stateRadio;
  norace uint8_t stateTimer;
  norace uint8_t currentDSN;
  norace bool bAckEnable;
  bool bPacketReceiving;
  uint8_t txlength;
  norace uint8_t* txbufptr;  // pointer to transmit buffer
  norace uint8_t* rxbufptr;  // pointer to receive buffer
  PhyPktBuf RxBuf;	// save received messages
  
  int16_t min_backoff_us;
  int16_t def_backoff_us;
  int16_t max_backoff_us;
  uint8_t backoff_retries;
  uint16_t flags;

  volatile uint16_t LocalAddr;

  int16_t getInitialBackoff(uint8_t *pkt);
  int16_t getCongestionBackoff(uint8_t *pkt);
  ///**********************************************************
  //* local function definitions
  //**********************************************************/

   void sendFailed() {
     atomic stateRadio = IDLE_STATE;
     signal PhyComm.txPktDone(txbufptr, 1);
   }

   void flushRXFIFO() {
     call FIFOP.disable();
     call HPLChipcon.read(CC2420_RXFIFO);          //flush Rx fifo
     call HPLChipcon.cmd(CC2420_SFLUSHRX);
     call HPLChipcon.cmd(CC2420_SFLUSHRX);
     atomic bPacketReceiving = FALSE;
     call FIFOP.startWait(FALSE);
   }

   inline result_t setInitialTimer( uint16_t jiffy ) {
     stateTimer = TIMER_INITIAL;
     if (jiffy == 0)
       // set the minimum timer time
       return call BackoffTimerJiffy.setOneShot(2);
     return call BackoffTimerJiffy.setOneShot(jiffy);
   }

   inline result_t setBackoffTimer( uint16_t jiffy ) {
     stateTimer = TIMER_BACKOFF;
     if (jiffy == 0)
       // set the minimum timer time
       return call BackoffTimerJiffy.setOneShot(2);
     return call BackoffTimerJiffy.setOneShot(jiffy);
   }

   inline result_t setAckTimer( uint16_t jiffy ) {
     stateTimer = TIMER_ACK;
     return call BackoffTimerJiffy.setOneShot(jiffy);
   }

  /***************************************************************************
   * PacketRcvd
   * - Radio packet rcvd, signal 
   ***************************************************************************/
   task void PacketRcvd() {
     signal PhyComm.rxPktDone(rxbufptr, 0);
     atomic {
       bPacketReceiving = FALSE;
     }
   }

  
  task void PacketSent() {

    atomic {
      stateRadio = IDLE_STATE;
    }

    signal PhyComm.txPktDone(txbufptr, 0);
  }

  //**********************************************************
  //* Exported interface functions for Std/SplitControl
  //* StdControl is deprecated, use SplitControl
  //**********************************************************/
  
  // Split-phase initialization of the radio
  command result_t SplitControl.init() {

    atomic {
      stateRadio = DISABLED_STATE;
      currentDSN = 0;
      bAckEnable = FALSE;
      bPacketReceiving = FALSE;
      rxbufptr = (uint8_t *)&RxBuf;
    }

    call TimerControl.init();
    call Random.init();
    LocalAddr = TOS_LOCAL_ADDRESS;
    return call CC2420SplitControl.init();
  }

  event result_t CC2420SplitControl.initDone() {
	//trace(DBG_USR1, "init done!\r\n");
	call CC2420Control.TuneManual(2440);
	atomic backoff_retries = MAX_SEND_TRIES;
	atomic def_backoff_us = CC2420_SYMBOL_UNIT * ((call Random.rand() % 0x3F) + 1);
    return signal SplitControl.initDone();
  }

  default event result_t SplitControl.initDone() {
    return SUCCESS;
  }
  
  // split phase stop of the radio stack
  command result_t SplitControl.stop() {
    atomic stateRadio = DISABLED_STATE;

    call SFD.disable();
    call FIFOP.disable();
    call TimerControl.stop();
    return call CC2420SplitControl.stop();
  }

  event result_t CC2420SplitControl.stopDone() {
    return signal SplitControl.stopDone();
  }

  default event result_t SplitControl.stopDone() {
    return SUCCESS;
  }

  task void startRadio() {
    result_t success = FAIL;
    atomic {
      if (stateRadio == DISABLED_STATE_STARTTASK) {
	stateRadio = DISABLED_STATE;
	success = SUCCESS;
      }
    }

    if (success == SUCCESS) 
      call SplitControl.start();
  }

  // split phase start of the radio stack (wait for oscillator to start)
  command result_t SplitControl.start() {
    uint8_t chkstateRadio;

    atomic chkstateRadio = stateRadio;
    atomic bAckEnable = FALSE;
    call CC2420Control.disableAddrDecode();
    call CC2420Control.disableAutoAck();
    if (chkstateRadio == DISABLED_STATE) {
      atomic {
	stateRadio = WARMUP_STATE;
        countRetry = 0;
      }
      call TimerControl.start();
      return call CC2420SplitControl.start();
    }
    return FAIL;
  }

  event result_t CC2420SplitControl.startDone() {
    uint8_t chkstateRadio;

    atomic chkstateRadio = stateRadio;

    if (chkstateRadio == WARMUP_STATE) {
	  //trace(DBG_USR1, "warmup, set rxMode!\r\n"); 
      call CC2420Control.RxMode();
      //enable interrupt when pkt rcvd
      call FIFOP.startWait(FALSE);
      // enable start of frame delimiter timer capture (timestamping)
      call SFD.enableCapture(TRUE);
      
      atomic stateRadio  = IDLE_STATE;
    }
    signal SplitControl.startDone();
    return SUCCESS;
  }

  default event result_t SplitControl.startDone() {
    return SUCCESS;
  }

	command result_t PhyState.idle()
	{
		uint8_t chkState;
	
		atomic {
			chkState = stateRadio;
		}
		/* If we are asked to return to idle from sleep, wake us up */
		switch (chkState) {
		case DISABLED_STATE:
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
		call SplitControl.stop();
	}

  /************* END OF STDCONTROL/SPLITCONTROL INIT FUNCITONS **********/

  /**
   * Try to send a packet.  If unsuccessful, backoff again
   **/
  void sendPacket() {
	uint8_t backoff;
    uint8_t status;

	atomic backoff = (flags & FLAG_TXONCCA);
    call HPLChipcon.cmd((backoff) ? CC2420_STXONCCA : CC2420_STXON);
    status = call HPLChipcon.cmd(CC2420_SNOP);
    if ((status >> CC2420_TX_ACTIVE) & 0x01) {
      // wait for the SFD to go high for the transmit SFD
      call SFD.enableCapture(TRUE);
    }
    else {
      // try again to send the packet
      atomic stateRadio = PRE_TX_STATE;
      if (!(setBackoffTimer(getCongestionBackoff(txbufptr)))) {
        sendFailed();
      }
    }
  }

  /**
   * Captured an edge transition on the SFD pin
   * Useful for time synchronization as well as determining
   * when a packet has finished transmission
   */
  async event result_t SFD.captured(uint16_t time) {
    switch (stateRadio) {
    case TX_STATE:
      // wait for SFD to fall--indicates end of packet
      call SFD.enableCapture(FALSE);
      // if the pin already fell, disable the capture and let the next
      // state enable the cpature (bug fix from Phil Buonadonna)
      if (!TOSH_READ_CC_SFD_PIN()) {
	call SFD.disable();
      }
      else {
	atomic stateRadio = TX_WAIT;
      }
      // fire TX SFD event
      // if the pin hasn't fallen, break out and wait for the interrupt
      // if it fell, continue on the to the TX_WAIT state
      if (stateRadio == TX_WAIT) {
	break;
      }
    case TX_WAIT:
      // end of packet reached
      atomic stateRadio = POST_TX_STATE;
      call SFD.disable();
      // revert to receive SFD capture
      call SFD.enableCapture(TRUE);
      // if no acks or broadcast, post packet send done event
      if (!post PacketSent())
        sendFailed();
      break;
    default:
      // fire RX SFD handler
	  //trace(DBG_USR1, "firing startSymDetected()\r\n");
      signal PhyComm.startSymDetected(rxbufptr);
    }
    return SUCCESS;
  }

  /**
   * Start sending the packet data to the TXFIFO of the CC2420
   */
  task void startSend() {
    // flush the tx fifo of stale data
    if (!(call HPLChipcon.cmd(CC2420_SFLUSHTX))) {
      sendFailed();
      return;
    }
    // write the txbuf data to the TXFIFO
    if (!(call HPLChipconFIFO.writeTXFIFO(txlength+1,(uint8_t*)txbufptr))) {
      sendFailed();
      return;
    }
  }

  /**
   * Check for a clear channel and try to send the packet if a clear
   * channel exists using the sendPacket() function
   */
  void tryToSend() {
	 uint8_t backoff;
     uint8_t currentstate;
     atomic currentstate = stateRadio;

     // and the CCA check is good
     if (currentstate == PRE_TX_STATE) {

       // if a FIFO overflow occurs or if the data length is invalid, flush
       // the RXFIFO to get back to a normal state.
       if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())) {
         flushRXFIFO();
       }

	   atomic backoff = (flags & FLAG_TXONCCA);

       if ((!backoff) || TOSH_READ_RADIO_CCA_PIN()) {
         atomic stateRadio = TX_STATE;
         sendPacket();
       } else {
	 // if we tried a bunch of times, the radio may be in a bad state
	 // flushing the RXFIFO returns the radio to a non-overflow state
	 // and it continue normal operation (and thus send our packet)
         if (countRetry-- <= 0) {
	   flushRXFIFO();
	   atomic countRetry = backoff_retries;
	   if (!post startSend())
	     sendFailed();
           return;
         }
         if (!(setBackoffTimer(getCongestionBackoff(txbufptr)))) {
           sendFailed();
         }
       }
     }
  }

  /**
   * Multiplexed timer to control initial backoff, 
   * congestion backoff, and delay while waiting for an ACK
   */
  async event result_t BackoffTimerJiffy.fired() {
    uint8_t currentstate;
    atomic currentstate = stateRadio;

    switch (stateTimer) {
    case TIMER_INITIAL:
      if (!(post startSend())) {
        sendFailed();
      }
      break;
    case TIMER_BACKOFF:
      tryToSend();
      break;
    case TIMER_ACK:
      if (currentstate == POST_TX_STATE) {
	/* MDW 12-July-05: Race condition here: If ACK comes in before
	 * PacketSent() runs, the task can be posted twice (duplicate
	 * sendDone events). Fix: set the state to a different value to
	 * suppress the later task.
	 */
	atomic {
	  stateRadio = POST_TX_ACK_STATE;
	}
        if (!post PacketSent())
	  sendFailed();
      }
      break;
    }
    return SUCCESS;
  }

 /**********************************************************
   * Send
   * - Xmit a packet
   *    USE SFD FALLING FOR END OF XMIT !!!!!!!!!!!!!!!!!! interrupt???
   * - If in power-down state start timer ? !!!!!!!!!!!!!!!!!!!!!!!!!s
   * - If !TxBusy then 
   *   a) Flush the tx fifo 
   *   b) Write Txfifo address
   *    
   **********************************************************/
  command result_t PhyComm.reTxPkt() {
	atomic stateRadio = PRE_TX_STATE;
    tryToSend();
    return SUCCESS;
  }

  command result_t PhyComm.txPkt(void *pkt, uint8_t pkt_sz) {
    uint8_t currentstate;
	//uint8_t backoff;
    atomic currentstate = stateRadio;

    if (currentstate == IDLE_STATE) { 
      txlength = pkt_sz-1; 
      txbufptr = pkt;
      ((PhyHeader *)txbufptr)->length = pkt_sz-1;
      atomic countRetry = backoff_retries;
	  
	  //atomic backoff = (flags & FLAG_TXONCCA);
	  //if (backoff) {
        if (setInitialTimer(getInitialBackoff(txbufptr))) {
          atomic stateRadio = PRE_TX_STATE;
          return SUCCESS;
        }
      //} else {
	  //atomic stateRadio = PRE_TX_STATE;
	  //tryToSend();
	  //}
    }
    return FAIL;

  }
  
  /**
   * Delayed RXFIFO is used to read the receive FIFO of the CC2420
   * in task context after the uC receives an interrupt that a packet
   * is in the RXFIFO.  Task context is necessary since reading from
   * the FIFO may take a while and we'd like to get other interrupts
   * during that time, or notifications of additional packets received
   * and stored in the CC2420 RXFIFO.
   */
  void delayedRXFIFO();

  task void delayedRXFIFOtask() {
    delayedRXFIFO();
  }

  void delayedRXFIFO() {
    uint8_t len = MSG_DATA_SIZE;  
    uint8_t _bPacketReceiving;
	//trace(DBG_USR1, "delayedRXFIFO 1!\r\n"); 

    if ((!TOSH_READ_CC_FIFO_PIN()) && (!TOSH_READ_CC_FIFOP_PIN())) {
        flushRXFIFO();
	return;
    }
	//trace(DBG_USR1, "delayedRXFIFO 2!\r\n"); 

    atomic {
      _bPacketReceiving = bPacketReceiving;
      
      if (_bPacketReceiving) {
	if (!post delayedRXFIFOtask())
	  flushRXFIFO();
      } else {
	bPacketReceiving = TRUE;
      }
    }
   	//trace(DBG_USR1, "delayedRXFIFO 3!\r\n"); 
    // JP NOTE: TODO: move readRXFIFO out of atomic context to permit
    // high frequency sampling applications and remove delays on
    // interrupts being processed.  There is a race condition
    // that has not yet been diagnosed when RXFIFO may be interrupted.
    if (!_bPacketReceiving) {
		//trace(DBG_USR1, "delayedRXFIFO 4!\r\n"); 
      if (!call HPLChipconFIFO.readRXFIFO(len,(uint8_t*)rxbufptr)) {
	atomic bPacketReceiving = FALSE;
	//trace(DBG_USR1, "delayedRXFIFO 5!\r\n"); 
	if (!post delayedRXFIFOtask()) {
	  flushRXFIFO();
	}
	return;
      }      
    }
    flushRXFIFO();
  }
  
  /**********************************************************
   * FIFOP lo Interrupt: Rx data avail in CC2420 fifo
   * Radio must have been in Rx mode to get this interrupt
   * If FIFO pin =lo then fifo overflow=> flush fifo & exit
   * 
   *
   * Things ToDo:
   *
   * -Disable FIFOP interrupt until PacketRcvd task complete 
   * until send.done complete
   *
   * -Fix mixup: on return
   *  rxbufptr->rssi is CRC + Correlation value
   *  rxbufptr->strength is RSSI
   **********************************************************/
   async event result_t FIFOP.fired() {

     //     call Leds.yellowToggle();

     // if we're trying to send a message and a FIFOP interrupt occurs
     // and acks are enabled, we need to backoff longer so that we don't
     // interfere with the ACK

     /** Check for RXFIFO overflow **/     
     if (!TOSH_READ_CC_FIFO_PIN()){
       flushRXFIFO();
       return SUCCESS;
     }

     atomic {
	 if (post delayedRXFIFOtask()) {
	   call FIFOP.disable();
	 }
	 else {
	   flushRXFIFO();
	 }
     }

     // return SUCCESS to keep FIFOP events occurring
     return SUCCESS;
  }

  /**
   * After the buffer is received from the RXFIFO,
   * process it, then post a task to signal it to the higher layers
   */
  async event result_t HPLChipconFIFO.RXFIFODone(uint8_t length, uint8_t *data) {
    PhyPktBuf *pBuf;
    // JP NOTE: rare known bug in high contention:
    // radio stack will receive a valid packet, but for some reason the
    // length field will be longer than normal.  The packet data will
    // be valid up to the correct length, and then will contain garbage
    // after the correct length.  There is no currently known fix.
    uint8_t currentstate;
    atomic { 
      currentstate = stateRadio;
    }

	//trace(DBG_USR1, "RXFIFODone!\r\n");
    // if a FIFO overflow occurs or if the data length is invalid, flush
    // the RXFIFO to get back to a normal state.
    if ((!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) 
        || (length == 0) || (length > PHY_MAX_PKT_LEN)) {
      flushRXFIFO();
      atomic bPacketReceiving = FALSE;
      return SUCCESS;
    }

    rxbufptr = data;
    pBuf = (PhyPktBuf *)rxbufptr;
	//trace(DBG_USR1, "RXFIFODone 2!\r\n");

    if (pBuf->hdr.length > PHY_MAX_PKT_LEN) {
      flushRXFIFO();
      atomic bPacketReceiving = FALSE;
      return SUCCESS;
    }
 	//trace(DBG_USR1, "RXFIFODone 3!\r\n");

    // if the length is shorter, we have to move the CRC bytes
    pBuf->crc = data[length-1] >> 7;
    // put in RSSI
    pBuf->info.strength = data[length-2];

    atomic {
      if (!post PacketRcvd()) {
	bPacketReceiving = FALSE;
      }
    }

	//trace(DBG_USR1, "RXFIFODone 4!\r\n");

    if ((!TOSH_READ_CC_FIFO_PIN()) && (!TOSH_READ_CC_FIFOP_PIN())) {
        flushRXFIFO();
	return SUCCESS;
    }

    if (!(TOSH_READ_CC_FIFOP_PIN())) {
      if (post delayedRXFIFOtask())
	return SUCCESS;
    }
    flushRXFIFO();
    //    call FIFOP.startWait(FALSE);

    return SUCCESS;
  }

  /**
   * Notification that the TXFIFO has been filled with the data from the packet
   * Next step is to try to send the packet
   */
  async event result_t HPLChipconFIFO.TXFIFODone(uint8_t length, uint8_t *data) { 
     tryToSend();
     return SUCCESS;
  }

  int16_t getInitialBackoff(uint8_t *pkt) {
    return CC2420_SYMBOL_UNIT * ((call Random.rand() & 0xF) + 1);
  }

  int16_t getCongestionBackoff(uint8_t *pkt) {
	int16_t time;

	atomic {
	  if (flags & FLAG_BACKOFF_RANDOM) {
		time = (call Random.rand() % max_backoff_us) + min_backoff_us;
	  } else {
		time = def_backoff_us;
		//time = CC2420_SYMBOL_UNIT * ((call Random.rand() % 0x3F) + 1); 
	  }
	}
	return time;
  }
	command result_t BackoffControl.enableBackoff()
	{
		atomic flags |= FLAG_TXONCCA;
		return SUCCESS;
	}

	command result_t BackoffControl.disableBackoff()
	{
		atomic flags &= (~FLAG_TXONCCA);
		return SUCCESS;
	}

	command result_t BackoffControl.setMode(uint8_t random)
	{
		if (random) {
			atomic flags |= FLAG_BACKOFF_RANDOM;
		} else {
			atomic flags &= (~FLAG_BACKOFF_RANDOM);
		}
		return SUCCESS;
	}

	command result_t BackoffControl.setRandomLimits(int16_t min, int16_t max)
	{
		atomic {
			min_backoff_us = min;
			max_backoff_us = max+1-min;
		}
		return SUCCESS;
	}

	command result_t BackoffControl.setBackoffTime(int16_t time)
	{
		atomic def_backoff_us = time;
		return SUCCESS;
	}

	command result_t BackoffControl.setRetries(uint8_t retr) {
		atomic backoff_retries = retr;
		return SUCCESS;
	}

	command result_t PhyRadioControl.setFrequency(uint16_t freq)
	{
		return call CC2420Control.TuneManual(freq);
	}

	command result_t PhyRadioControl.setPower(uint8_t power)
	{
		return call CC2420Control.SetRFPower(power);
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

		if (TOSH_READ_RADIO_CCA_PIN()) {
			signal CarrierSense.channelIdle();
		} else {
			signal CarrierSense.channelBusy();
		}
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
		reg = call HPLChipcon.read(CC2420_RSSI);
		reg |= (thr << CC2420_RSSI_CCA_THRESH);
		return call HPLChipcon.write(CC2420_RSSI, reg);
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

		reg = call HPLChipcon.read(CC2420_RSSI);
		rssi = (int8_t)(reg & 0xff);

		rssi += RSSI_OFFSET;

		return rssi;
	}

}
