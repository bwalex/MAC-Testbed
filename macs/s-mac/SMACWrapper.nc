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
 * Authors: Jerry Zhao, Wei Ye
 * Date created: 2/21/2003
 * 
 * This is a Wrapper for S-MAC to provide standard tinyos Send/Receive
 *   interface, so that AMStandard can run over S-MAC.
 *
 * This component is to provide compatibilty to Berkeley's comm stack and 
 *   enable applications developed on Berkeley's stack to run over S-MAC 
 *   without modification. However, to use all functionality provided by 
 *   S-MAC, you need to develop your application directly over S-MAC.
 */

includes SMACWrapperMsg;

module SMACWrapper
{
   provides {
      interface StdControl as Control;
      interface ReceiveMsg as Receive;
      interface BareSendMsg as Send;
   }
   uses {
      interface StdControl as MACControl;
      interface MACComm;
   }
}

implementation
{
   WrapMsg txBuf;
   TOS_MsgPtr txMsgPtr;
   bool txBusy;
   TOS_Msg rxBuf;
   TOS_MsgPtr rxMsgPtr;
   bool rxBusy;
   
   command result_t Control.init()
   {
      rxMsgPtr = &rxBuf;
      txBusy = FALSE;
      rxBusy = FALSE;
      return call MACControl.init();
   }


   command result_t Control.start()
   {
      return call MACControl.start();
   }
   

   command result_t Control.stop()
   {
      return call MACControl.stop();
   }


   command result_t Send.send(TOS_MsgPtr msg)
   {
      result_t ok;
      uint8_t msgLen, txLen;
      msgLen = msg->length + MSG_DATA_SIZE - DATA_LENGTH - 2;
      txLen = sizeof(WrapHeader) + msgLen + 2;
      if (txBusy || txLen > PHY_MAX_PKT_LEN) return FAIL;
      txBusy = TRUE;
      memcpy(&(txBuf.tosMsg), msg, msgLen);
#ifdef USE_SMACSCEIVER   // for smacsceiver to understand the msg
      txBuf.wrapHdr.type = msg->type;
#endif
      if (msg->addr == TOS_BCAST_ADDR) {
         ok = call MACComm.broadcastMsg(&txBuf, txLen);
      } else {
         //Each unicast TOS_MSG is sent in one fragment.
         ok = call MACComm.unicastMsg(&txBuf, txLen, msg->addr, 1);
      }
      if (ok) {
         txMsgPtr = msg;
         return SUCCESS;
      } else {
         txBusy = FALSE;
         return FAIL;
      }
   }


   event result_t MACComm.broadcastDone(void* msg)
   {
      txBusy = FALSE;
      return signal Send.sendDone(txMsgPtr, SUCCESS);	
   }


   event result_t MACComm.unicastDone(void* msg, uint8_t txFragCount)
   {
      txBusy = FALSE;
      return signal Send.sendDone(txMsgPtr, (txFragCount==1));	
   }


   event void* MACComm.rxMsgDone(void* msg)
   {
      uint8_t msgLen;
      TOS_MsgPtr tmp = &(((WrapMsg*)msg)->tosMsg);
      if (rxBusy || tmp->length > DATA_LENGTH) return msg;
      rxBusy = TRUE;
      msgLen = tmp->length + MSG_DATA_SIZE - DATA_LENGTH - 2;
      memcpy(rxMsgPtr, tmp, msgLen);
      //CRC is passed in SMAC so 
      rxMsgPtr->crc = 1;
      tmp = signal Receive.receive(rxMsgPtr);
      if (tmp) rxMsgPtr = tmp;
      rxBusy = FALSE;
      return msg;
   }


   event result_t MACComm.txFragDone(void* frag){
      //Each unicast TOS_MSG is sent in one fragment.
      //This event should never be called
      return SUCCESS;
   }
}
