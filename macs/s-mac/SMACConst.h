/*
 * Copyright (c) 2003 the University of Southern California.
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
 * Authors:     Wei Ye
 * Date created: 11/2003
 *
 * S-MAC constants that can be used by applications
 */

#ifndef SMAC_CONST
#define SMAC_CONST

/* User-adjustable S-MAC parameters
 * Do not directly change this file
 * Change default values in each application's config.h file
 *--------------------------------
 * SMAC_MAX_NUM_NEIGHB: maximum number of neighbors.
 * SMAC_MAX_NUM_SCHED: maximum number of different schedules.
 * SMAC_DUTY_CYCLE: duty cycle in percentage. It controls the length of sleep 
 *   interval.
 * SMAC_RETRY_LIMIT: maximum number of RTS retries for sending a single message.
 * SMAC_EXTEND_LIMIT: maximum number of times to extend Tx time when ACK
 *   timeout happens.
 * The following two macros define the period to search for new neighbors that
 * are potentially on a different schedules. The period is expressed as the
 * number of SYNC_PERIODs (10s). Therefore, 30 means every 30 SYNC_PERIODs, the
 * node will keep listening for an entire SYNC_PERIOD. Max value: 255.
 * SMAC_SRCH_NBR_SHORT_PERIOD: if I have no neighbor, search more aggressively
 * SMAC_SRCH_NBR_LONG_PERIOD: used when I already have neighbors
 */
#ifndef SMAC_MAX_NUM_NEIGHB
#define SMAC_MAX_NUM_NEIGHB 20
#endif

#ifndef SMAC_MAX_NUM_SCHED
#define SMAC_MAX_NUM_SCHED 4
#endif

#ifndef SMAC_DUTY_CYCLE
#define SMAC_DUTY_CYCLE 10
#endif

#ifdef SMAC_RETRY_LIMIT
#warning SMAC_RETRY_LIMIT is no longer in use.
#endif

#ifdef SMAC_EXTEND_LIMIT
#warning SMAC_EXTEND_LIMIT is no longer in use.
#endif

#ifndef SMAC_RTS_RETRY_LIMIT
#define SMAC_RTS_RETRY_LIMIT 7
#endif

#ifdef SMAC_DATA_RETX_LIMIT
#if (SMAC_DATA_RETX_LIMIT > 7)
#error Maximum allowed SMAC_DATA_RETX_LIMIT is 7
#endif
#else
#define SMAC_DATA_RETX_LIMIT 3
#endif

#ifndef SMAC_MAX_TX_MSG_TIME
#ifdef SMAC_NO_SLEEP_CYCLE
#define SMAC_MAX_TX_MSG_TIME 10000
#else
#define SMAC_MAX_TX_MSG_TIME 120000
#endif
#endif

#ifndef SMAC_SRCH_NBR_SHORT_PERIOD
#define SMAC_SRCH_NBR_SHORT_PERIOD 3
#endif

#ifndef SMAC_SRCH_NBR_LONG_PERIOD
#define SMAC_SRCH_NBR_LONG_PERIOD 30
#endif


/* S-MAC constants, not user-adjustable, but can be used by applications
 * ------------------------------------
 * SMAC_MAX_NUM_FRAGS: maximum number of fragments in a unicast message
 * SMAC_MAX_SYNC_SEQ_NO: maximun sequence number for SYNC packets
 * SMAC_MAX_BCAST_SEQ_NO: maximum sequence number for broadcast packets
 * SMAC_MAX_UCAST_SEQ_NO: maximum sequence number for unicast packets
 */
 
#define SMAC_MAX_NUM_FRAGS 8
#define SMAC_MAX_SYNC_SEQ_NO 255
#define SMAC_MAX_BCAST_SEQ_NO 255
#define SMAC_MAX_UCAST_SEQ_NO 31

#endif
