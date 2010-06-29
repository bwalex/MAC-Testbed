/* ex: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */
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
 * Authors:	Thanos Stathopoulos
 * Date created: 1/21/2003
 *
 * This interface is for reporting SMAC-related values to upper layers
 * (e.g SMACSceiver)
 *
 */

interface MACReport
{
	command uint32_t getCrcErrors();
	command uint32_t getRetx();	
	command uint32_t getLenErrors();
	command uint32_t getCtrlPkts();
	command uint8_t  getNumNeighb();
	command uint8_t  getNumSched();
	command uint8_t  getMacState();
	command uint8_t  getRadioState(); 
	command uint8_t  getTxState();
	command uint16_t getNav();
	command uint16_t getNeighbNav();
    command uint16_t getMacTxErrors();
    command uint16_t getSleepErrors();

}
