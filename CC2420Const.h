// $Id: CC2420Const.h,v 1.10 2005/11/30 23:28:58 jpolastre Exp $

/* -*- Mode: C; c-basic-indent: 2; indent-tabs-mode: nil -*- */ 
/*									tab:4
 *  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.  By
 *  downloading, copying, installing or using the software you agree to
 *  this license.  If you do not agree to this license, do not download,
 *  install, copy or use the software.
 *
 *  Intel Open Source License 
 *
 *  Copyright (c) 2002 Intel Corporation 
 *  All rights reserved. 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 * 
 *	Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *	Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *      Neither the name of the Intel Corporation nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *  PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE INTEL OR ITS
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 */
/*
 *
 * Authors:	        Joe Polastre, Alan Broad
 *
 */

/**
 * @author Joe Polastre
 * @author Alan Broad
 */



#ifndef _CC2420CONST_H
#define _CC2420CONST_H

// times for the CC2420 in microseconds
enum {
  CC2420_TIME_BIT = 4,
  CC2420_TIME_BYTE = CC2420_TIME_BIT << 3,
  CC2420_TIME_SYMBOL = 16
};

#ifndef CC2420_DEF_RFPOWER
#define CC2420_DEF_RFPOWER          0x1F
#endif

#ifndef CC2420_DEF_CHANNEL
#define CC2420_DEF_CHANNEL	    11  //channel select
#endif

// added for post-compile frequency changing
uint8_t CC2420_CHANNEL =            CC2420_DEF_CHANNEL;
uint8_t CC2420_RFPOWER =            CC2420_DEF_RFPOWER;

enum {
  CC2420_MIN_CHANNEL =              11,
  CC2420_MAX_CHANNEL =              26
};

#define CC2420_DEF_PRESET           2405  //freq select

#define CC2420_DEF_FCF_LO          0x08
#define CC2420_DEF_FCF_HI          0x01  // without ACK
#define CC2420_DEF_FCF_HI_ACK      0x21  // with ACK
#define CC2420_DEF_FCF_TYPE_BEACON 0x00
#define CC2420_DEF_FCF_TYPE_DATA   0x01
#define CC2420_DEF_FCF_TYPE_ACK    0x02
#define CC2420_DEF_FCF_BIT_ACK     5
#define CC2420_DEF_BACKOFF         500
#define CC2420_SYMBOL_TIME         16 // 2^4
// 20 symbols make up a backoff period
// 10 jiffy's make up a backoff period
// due to timer overhead, 30.5us is close enough to 32us per 2 symbols
#define CC2420_SYMBOL_UNIT         10

// delay when waiting for the ack
#ifndef CC2420_ACK_DELAY
#define CC2420_ACK_DELAY           75
#endif

#ifndef CC2420_XOSC_TIMEOUT
#define CC2420_XOSC_TIMEOUT 200     //times to chk if CC2420 crystal is on
#endif

#define CC2420_SNOP            0x00
#define CC2420_SXOSCON         0x01
#define CC2420_STXCAL          0x02
#define CC2420_SRXON           0x03
#define CC2420_STXON           0x04
#define CC2420_STXONCCA        0x05
#define CC2420_SRFOFF          0x06
#define CC2420_SXOSCOFF        0x07
#define CC2420_SFLUSHRX        0x08
#define CC2420_SFLUSHTX        0x09
#define CC2420_SACK            0x0A
#define CC2420_SACKPEND        0x0B
#define CC2420_SRXDEC          0x0C
#define CC2420_STXENC          0x0D
#define CC2420_SAES            0x0E
#define CC2420_MAIN            0x10
#define CC2420_MDMCTRL0        0x11
#define CC2420_MDMCTRL1        0x12
#define CC2420_RSSI            0x13
#define CC2420_SYNCWORD        0x14
#define CC2420_TXCTRL          0x15
#define CC2420_RXCTRL0         0x16
#define CC2420_RXCTRL1         0x17
#define CC2420_FSCTRL          0x18
#define CC2420_SECCTRL0        0x19
#define CC2420_SECCTRL1        0x1A
#define CC2420_BATTMON         0x1B
#define CC2420_IOCFG0          0x1C
#define CC2420_IOCFG1          0x1D
#define CC2420_MANFIDL         0x1E
#define CC2420_MANFIDH         0x1F
#define CC2420_FSMTC           0x20
#define CC2420_MANAND          0x21
#define CC2420_MANOR           0x22
#define CC2420_AGCCTRL         0x23
#define CC2420_AGCTST0         0x24
#define CC2420_AGCTST1         0x25
#define CC2420_AGCTST2         0x26
#define CC2420_FSTST0          0x27
#define CC2420_FSTST1          0x28
#define CC2420_FSTST2          0x29
#define CC2420_FSTST3          0x2A
#define CC2420_RXBPFTST        0x2B
#define CC2420_FSMSTATE        0x2C
#define CC2420_ADCTST          0x2D
#define CC2420_DACTST          0x2E
#define CC2420_TOPTST          0x2F
#define CC2420_RESERVED        0x30
#define CC2420_TXFIFO          0x3E
#define CC2420_RXFIFO          0x3F

#define CC2420_RAM_SHORTADR    0x16A
#define CC2420_RAM_PANID       0x168
#define CC2420_RAM_IEEEADR     0x160
#define CC2420_RAM_CBCSTATE    0x150
#define CC2420_RAM_TXNONCE     0x140
#define CC2420_RAM_KEY1        0x130
#define CC2420_RAM_SABUF       0x120
#define CC2420_RAM_RXNONCE     0x110
#define CC2420_RAM_KEY0        0x100
#define CC2420_RAM_RXFIFO      0x080
#define CC2420_RAM_TXFIFO      0x000

// MDMCTRL0 Register Bit Positions
#define CC2420_MDMCTRL0_FRAME        13  // 0 : reject reserved frame types, 1 = accept
#define CC2420_MDMCTRL0_PANCRD       12  // 0 : not a PAN coordinator
#define CC2420_MDMCTRL0_ADRDECODE    11  // 1 : enable address decode
#define CC2420_MDMCTRL0_CCAHIST      8   // 3 bits (8,9,10) : CCA hysteris in db
#define CC2420_MDMCTRL0_CCAMODE      6   // 2 bits (6,7)    : CCA trigger modes
#define CC2420_MDMCTRL0_AUTOCRC      5   // 1 : generate/chk CRC
#define CC2420_MDMCTRL0_AUTOACK      4   // 1 : Ack valid packets
#define CC2420_MDMCTRL0_PREAMBL      0   // 4 bits (0..3): Preamble length

// MDMCTRL1 Register Bit Positions
#define CC2420_MDMCTRL1_CORRTHRESH   6   // 5 bits (6..10) : correlator threshold
#define CC2420_MDMCTRL1_DEMOD_MODE   5   // 0: lock freq after preamble match, 1: continous udpate
#define CC2420_MDMCTRL1_MODU_MODE    4   // 0: IEEE 802.15.4
#define CC2420_MDMCTRL1_TX_MODE      2   // 2 bits (2,3) : 0: use buffered TXFIFO
#define CC2420_MDMCTRL1_RX_MODE      0   // 2 bits (0,1) : 0: use buffered RXFIFO

// RSSI Register Bit Positions
#define CC2420_RSSI_CCA_THRESH       8   // 8 bits (8..15) : 2's compl CCA threshold

// TXCTRL Register Bit Positions
#define CC2420_TXCTRL_BUFCUR         14  // 2 bits (14,15) : Tx mixer buffer bias current
#define CC2420_TXCTRL_TURNARND       13  // wait time after STXON before xmit
#define CC2420_TXCTRL_VAR            11  // 2 bits (11,12) : Varactor array settings
#define CC2420_TXCTRL_XMITCUR        9   // 2 bits (9,10)  : Xmit mixer currents
#define CC2420_TXCTRL_PACUR          6   // 3 bits (6..8)  : PA current
#define CC2420_TXCTRL_PADIFF         5   // 1: Diff PA, 0: Single ended PA
#define CC2420_TXCTRL_PAPWR          0   // 5 bits (0..4): Output PA level

// Mask for the CC2420_TXCTRL_PAPWR register for RF power
#define CC2420_TXCTRL_PAPWR_MASK (0x1F << CC2420_TXCTRL_PAPWR)

// RXCTRL0 Register Bit Positions
#define CC2420_RXCTRL0_BUFCUR        12  // 2 bits (12,13) : Rx mixer buffer bias current
#define CC2420_RXCTRL0_HILNAG        10  // 2 bits (10,11) : High gain, LNA current
#define CC2420_RXCTRL0_MLNAG          8  // 2 bits (8,9)   : Med gain, LNA current
#define CC2420_RXCTRL0_LOLNAG         6  // 2 bits (6,7)   : Lo gain, LNA current
#define CC2420_RXCTRL0_HICUR          4  // 2 bits (4,5)   : Main high LNA current
#define CC2420_RXCTRL0_MCUR           2  // 2 bits (2,3)   : Main med  LNA current
#define CC2420_RXCTRL0_LOCUR          0  // 2 bits (0,1)   : Main low LNA current

// RXCTRL1 Register Bit Positions
#define CC2420_RXCTRL1_LOCUR         13  // Ref bias current to Rx bandpass filter
#define CC2420_RXCTRL1_MIDCUR        12  // Ref bias current to Rx bandpass filter
#define CC2420_RXCTRL1_LOLOGAIN      11  // LAN low gain mode
#define CC2420_RXCTRL1_MEDLOGAIN     10  // LAN low gain mode
#define CC2420_RXCTRL1_HIHGM          9  // Rx mixers, hi gain mode
#define CC2420_RXCTRL1_MEDHGM         8  // Rx mixers, hi gain mode
#define CC2420_RXCTRL1_LNACAP         6  // 2 bits (6,7) Selects LAN varactor array setting
#define CC2420_RXCTRL1_RMIXT          4  // 2 bits (4,5) Receiver mixer output current
#define CC2420_RXCTRL1_RMIXV          2  // 2 bits (2,3) VCM level, mixer feedback
#define CC2420_RXCTRL1_RMIXCUR        0  // 2 bits (0,1) Receiver mixer current

// FSCTRL Register Bit Positions
#define CC2420_FSCTRL_LOCK            14 // 2 bits (14,15) # of clocks for synch
#define CC2420_FSCTRL_CALDONE         13 // Read only, =1 if cal done since freq synth turned on
#define CC2420_FSCTRL_CALRUNING       12 // Read only, =1 if cal in progress
#define CC2420_FSCTRL_LOCKLEN         11 // Synch window pulse width
#define CC2420_FSCTRL_LOCKSTAT        10 // Read only, = 1 if freq synthesizer is loced
#define CC2420_FSCTRL_FREQ             0 // 10 bits, set operating frequency 

// SECCTRL0 Register Bit Positions
#define CC2420_SECCTRL0_PROTECT        9 // Protect enable Rx fifo
#define CC2420_SECCTRL0_CBCHEAD        8 // Define 1st byte of CBC-MAC
#define CC2420_SECCTRL0_SAKEYSEL       7 // Stand alone key select
#define CC2420_SECCTRL0_TXKEYSEL       6 // Tx key select
#define CC2420_SECCTRL0_RXKEYSEL       5 // Rx key select
#define CC2420_SECCTRL0_SECM           2 // 2 bits (2..4) # of bytes in CBC-MAX auth field
#define CC2420_SECCTRL0_SECMODE        0 // Security mode

// SECCTRL1 Register Bit Positions
#define CC2420_SECCTRL1_TXL            8 // 7 bits (8..14) Tx in-line security
#define CC2420_SECCTRL1_RXL            0 // 7 bits (0..7)  Rx in-line security

// BATTMON  Register Bit Positions
#define CC2420_BATTMON_OK              6 // Read only, batter voltage OK
#define CC2420_BATTMON_EN              5 // Enable battery monitor
#define CC2420_BATTMON_VOLT            0 // 5 bits (0..4) Battery toggle voltage

// IOCFG0 Register Bit Positions
#define CC2420_IOCFG0_FIFOPOL         10 // Fifo signal polarity
#define CC2420_IOCFG0_FIFOPPOL         9 // FifoP signal polarity
#define CC2420_IOCFG0_SFD              8 // SFD signal polarity
#define CC2420_IOCFG0_CCAPOL           7 // CCA signal polarity
#define CC2420_IOCFG0_FIFOTHR          0 // 7 bits, (0..6) # of Rx bytes in fifo to trg fifop

// IOCFG1 Register Bit Positions
#define CC2420_IOCFG1_HSSD            10 // 2 bits (10,11) HSSD module config
#define CC2420_IOCFG1_SFDMUX           5 // 5 bits (5..9)  SFD multiplexer pin settings
#define CC2420_IOCFG1_CCAMUX           0 // 5 bits (0..4)  CCA multiplexe pin settings

// Current Parameter Arrray Positions
enum{
 CP_MAIN = 0,
 CP_MDMCTRL0,
 CP_MDMCTRL1,
 CP_RSSI,
 CP_SYNCWORD,
 CP_TXCTRL,
 CP_RXCTRL0,
 CP_RXCTRL1,
 CP_FSCTRL,
 CP_SECCTRL0,
 CP_SECCTRL1,
 CP_BATTMON,
 CP_IOCFG0,
 CP_IOCFG1
} ;

// STATUS Bit Posititions
#define CC2420_XOSC16M_STABLE	6
#define CC2420_TX_UNDERFLOW	5
#define CC2420_ENC_BUSY		4
#define CC2420_TX_ACTIVE	3
#define CC2420_LOCK   		2
#define CC2420_RSSI_VALID	1

#endif /* _CC2420CONST_H */
