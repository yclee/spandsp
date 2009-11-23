/*
 * SpanDSP - a series of DSP components for telephony
 *
 * t38_terminal.c - T.38 termination, less the packet exchange part
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2005, 2006 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Id: t38_terminal.c,v 1.80 2007/11/30 12:20:34 steveu Exp $
 */

/*! \file */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif
#include <assert.h>
#include <tiffio.h>

#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/bit_operations.h"
#include "spandsp/queue.h"
#include "spandsp/power_meter.h"
#include "spandsp/complex.h"
#include "spandsp/tone_generate.h"
#include "spandsp/async.h"
#include "spandsp/hdlc.h"
#include "spandsp/fsk.h"
#include "spandsp/v29rx.h"
#include "spandsp/v29tx.h"
#include "spandsp/v27ter_rx.h"
#include "spandsp/v27ter_tx.h"
#include "spandsp/v17rx.h"
#include "spandsp/v17tx.h"
#include "spandsp/t4.h"
#include "spandsp/t30_fcf.h"
#include "spandsp/t35.h"
#include "spandsp/t30.h"
#include "spandsp/t38_core.h"
#include "spandsp/t38_terminal.h"

/* Settings suitable for paced transmission over a UDP transport */
#define MS_PER_TX_CHUNK                 30
#define INDICATOR_TX_COUNT              3
#define DATA_TX_COUNT                   1
#define DATA_END_TX_COUNT               3

/* Settings suitable for unpaced transmission over a TCP transport */
#define MAX_OCTETS_PER_UNPACED_CHUNK    300

/* Backstop timeout if reception of packets stops in the middle of a burst */
#define MID_RX_TIMEOUT                  15000

enum
{
    T38_TIMED_STEP_NONE = 0,
    T38_TIMED_STEP_NON_ECM_MODEM,
    T38_TIMED_STEP_NON_ECM_MODEM_2,
    T38_TIMED_STEP_NON_ECM_MODEM_3,
    T38_TIMED_STEP_NON_ECM_MODEM_4,
    T38_TIMED_STEP_NON_ECM_MODEM_5,
    T38_TIMED_STEP_HDLC_MODEM,
    T38_TIMED_STEP_HDLC_MODEM_2,
    T38_TIMED_STEP_HDLC_MODEM_3,
    T38_TIMED_STEP_HDLC_MODEM_4,
    T38_TIMED_STEP_CED,
    T38_TIMED_STEP_CED_2,
    T38_TIMED_STEP_CNG,
    T38_TIMED_STEP_CNG_2,
    T38_TIMED_STEP_PAUSE
};

static int process_rx_missing(t38_core_state_t *t, void *user_data, int rx_seq_no, int expected_seq_no)
{
    t38_terminal_state_t *s;
    
    s = (t38_terminal_state_t *) user_data;
    s->missing_data = TRUE;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_indicator(t38_core_state_t *t, void *user_data, int indicator)
{
    t38_terminal_state_t *s;
    
    s = (t38_terminal_state_t *) user_data;
    if (t->current_rx_indicator == indicator)
    {
        /* This is probably due to the far end repeating itself. Ignore it. Its harmless */
        return 0;
    }
    /* In termination mode we don't care very much about indicators telling us training
       is starting. We only care about V.21 preamble starting, for timeout control, and
       the actual data. */
    switch (indicator)
    {
    case T38_IND_NO_SIGNAL:
        if (t->current_rx_indicator == T38_IND_V21_PREAMBLE
            &&
            (s->current_rx_type == T30_MODEM_V21  ||  s->current_rx_type == T30_MODEM_CNG))
        {
            t30_hdlc_accept(&(s->t30_state), NULL, PUTBIT_CARRIER_DOWN, TRUE);
        }
        s->timeout_rx_samples = 0;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_ABSENT);
        break;
    case T38_IND_CNG:
        break;
    case T38_IND_CED:
        break;
    case T38_IND_V21_PREAMBLE:
        /* Some people pop these preamble indicators between HDLC frames, so we need to be tolerant of that. */
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_PRESENT);
        break;
    case T38_IND_V27TER_2400_TRAINING:
    case T38_IND_V27TER_4800_TRAINING:
    case T38_IND_V29_7200_TRAINING:
    case T38_IND_V29_9600_TRAINING:
    case T38_IND_V17_7200_SHORT_TRAINING:
    case T38_IND_V17_7200_LONG_TRAINING:
    case T38_IND_V17_9600_SHORT_TRAINING:
    case T38_IND_V17_9600_LONG_TRAINING:
    case T38_IND_V17_12000_SHORT_TRAINING:
    case T38_IND_V17_12000_LONG_TRAINING:
    case T38_IND_V17_14400_SHORT_TRAINING:
    case T38_IND_V17_14400_LONG_TRAINING:
    case T38_IND_V33_12000_TRAINING:
    case T38_IND_V33_14400_TRAINING:
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_PRESENT);
        break;
    case T38_IND_V8_ANSAM:
    case T38_IND_V8_SIGNAL:
    case T38_IND_V34_CNTL_CHANNEL_1200:
    case T38_IND_V34_PRI_CHANNEL:
    case T38_IND_V34_CC_RETRAIN:
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_PRESENT);
        break;
    default:
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_ABSENT);
        break;
    }
    s->rx_len = 0;
    s->missing_data = FALSE;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_data(t38_core_state_t *t, void *user_data, int data_type, int field_type, const uint8_t *buf, int len)
{
    t38_terminal_state_t *s;
    uint8_t buf2[len];
    
    s = (t38_terminal_state_t *) user_data;
#if 0
    /* In termination mode we don't care very much what the data type is. */
    switch (data_type)
    {
    case T38_DATA_V21:
    case T38_DATA_V27TER_2400:
    case T38_DATA_V27TER_4800:
    case T38_DATA_V29_7200:
    case T38_DATA_V29_9600:
    case T38_DATA_V17_7200:
    case T38_DATA_V17_9600:
    case T38_DATA_V17_12000:
    case T38_DATA_V17_14400:
    case T38_DATA_V8:
    case T38_DATA_V34_PRI_RATE:
    case T38_DATA_V34_CC_1200:
    case T38_DATA_V34_PRI_CH:
    case T38_DATA_V33_12000:
    case T38_DATA_V33_14400:
    default:
        break;
    }
#endif

    switch (field_type)
    {
    case T38_FIELD_HDLC_DATA:
        if (s->timeout_rx_samples == 0)
        {
            /* HDLC can just start without any signal indicator on some platforms, even when
               there is zero packet lost. Nasty, but true. Its a good idea to be tolerant of
               loss, though, so accepting a sudden start of HDLC data is the right thing to do. */
            s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
            t30_front_end_status(&(s->t30_state), T30_FRONT_END_SIGNAL_PRESENT);
            /* All real HDLC messages in the FAX world start with 0xFF. If this one is not starting
               with 0xFF it would appear some octets must have been missed before this one. */
            if (buf[0] != 0xFF)
                s->missing_data = TRUE;
        }
        if (s->rx_len + len <= T38_MAX_HDLC_LEN)
        {
            bit_reverse(s->rx_buf + s->rx_len, buf, len);
            s->rx_len += len;
        }
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        break;
    case T38_FIELD_HDLC_FCS_OK:
        if (len > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "There is data in a T38_FIELD_HDLC_FCS_OK!\n");
            /* The sender has incorrectly included data in this message. It is unclear what we should do
               with it, to maximise tolerance of buggy implementations. */
        }
        /* Some T.38 implementations send multiple T38_FIELD_HDLC_FCS_OK messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Type %s - CRC OK (%s)\n", (s->rx_len >= 3)  ?  t30_frametype(s->rx_buf[2])  :  "???", (s->missing_data)  ?  "missing octets"  :  "clean");
            t30_hdlc_accept(&(s->t30_state), s->rx_buf, s->rx_len, !s->missing_data);
        }
        s->rx_len = 0;
        s->missing_data = FALSE;
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        break;
    case T38_FIELD_HDLC_FCS_BAD:
        if (len > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "There is data in a T38_FIELD_HDLC_FCS_BAD!\n");
            /* The sender has incorrectly included data in this message. We can safely ignore it, as the
               bad FCS means we will throw away the whole message, anyway. */
        }
        /* Some T.38 implementations send multiple T38_FIELD_HDLC_FCS_BAD messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Type %s - CRC bad (%s)\n", (s->rx_len >= 3)  ?  t30_frametype(s->rx_buf[2])  :  "???", (s->missing_data)  ?  "missing octets"  :  "clean");
            t30_hdlc_accept(&(s->t30_state), s->rx_buf, s->rx_len, FALSE);
        }
        s->rx_len = 0;
        s->missing_data = FALSE;
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        break;
    case T38_FIELD_HDLC_FCS_OK_SIG_END:
        if (len > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "There is data in a T38_FIELD_HDLC_FCS_OK_SIG_END!\n");
            /* The sender has incorrectly included data in this message. It is unclear what we should do
               with it, to maximise tolerance of buggy implementations. */
        }
        /* Some T.38 implementations send multiple T38_FIELD_HDLC_FCS_OK_SIG_END messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Type %s - CRC OK, sig end (%s)\n", (s->rx_len >= 3)  ?  t30_frametype(s->rx_buf[2])  :  "???", (s->missing_data)  ?  "missing octets"  :  "clean");
            t30_hdlc_accept(&(s->t30_state), s->rx_buf, s->rx_len, !s->missing_data);
            t30_hdlc_accept(&(s->t30_state), NULL, PUTBIT_CARRIER_DOWN, TRUE);
        }
        s->rx_len = 0;
        s->missing_data = FALSE;
        s->timeout_rx_samples = 0;
        break;
    case T38_FIELD_HDLC_FCS_BAD_SIG_END:
        if (len > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "There is data in a T38_FIELD_HDLC_FCS_BAD_SIG_END!\n");
            /* The sender has incorrectly included data in this message. We can safely ignore it, as the
               bad FCS means we will throw away the whole message, anyway. */
        }
        /* Some T.38 implementations send multiple T38_FIELD_HDLC_FCS_BAD_SIG_END messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Type %s - CRC bad, sig end (%s)\n", (s->rx_len >= 3)  ?  t30_frametype(s->rx_buf[2])  :  "???", (s->missing_data)  ?  "missing octets"  :  "clean");
            t30_hdlc_accept(&(s->t30_state), s->rx_buf, s->rx_len, FALSE);
            t30_hdlc_accept(&(s->t30_state), NULL, PUTBIT_CARRIER_DOWN, TRUE);
        }
        s->rx_len = 0;
        s->missing_data = FALSE;
        s->timeout_rx_samples = 0;
        break;
    case T38_FIELD_HDLC_SIG_END:
        if (len > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "There is data in a T38_FIELD_HDLC_SIG_END!\n");
            /* The sender has incorrectly included data in this message, but there seems nothing meaningful
               it could be. There could not be an FCS good/bad report beyond this. */
        }
        /* Some T.38 implementations send multiple T38_FIELD_HDLC_SIG_END messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            /* WORKAROUND: At least some Mediatrix boxes have a bug, where they can send this message at the
                           end of non-ECM data. We need to tolerate this. We use the generic receive complete
                           indication, rather than the specific HDLC carrier down. */
            /* This message is expected under 2 circumstances. One is as an alternative to T38_FIELD_HDLC_FCS_OK_SIG_END - 
               i.e. they send T38_FIELD_HDLC_FCS_OK, and then T38_FIELD_HDLC_SIG_END when the carrier actually drops.
               The other is because the HDLC signal drops unexpectedly - i.e. not just after a final frame. */
            s->rx_len = 0;
            s->missing_data = FALSE;
            s->timeout_rx_samples = 0;
            t30_front_end_status(&(s->t30_state), T30_FRONT_END_RECEIVE_COMPLETE);
        }
        break;
    case T38_FIELD_T4_NON_ECM_DATA:
        if (!s->rx_signal_present)
        {
            t30_non_ecm_put_bit(&(s->t30_state), PUTBIT_TRAINING_SUCCEEDED);
            s->rx_signal_present = TRUE;
        }
        bit_reverse(buf2, buf, len);
        t30_non_ecm_put_chunk(&(s->t30_state), buf2, len);
        s->timeout_rx_samples = s->samples + ms_to_samples(MID_RX_TIMEOUT);
        break;
    case T38_FIELD_T4_NON_ECM_SIG_END:
        /* Some T.38 implementations send multiple T38_FIELD_T4_NON_ECM_SIG_END messages, in IFP packets with
           incrementing sequence numbers, which are actually repeats. They get through to this point because
           of the incrementing sequence numbers. We need to filter them here in a context sensitive manner. */
        if (t->current_rx_data_type != data_type  ||  t->current_rx_field_type != field_type)
        {
            if (len > 0)
            {
                if (!s->rx_signal_present)
                {
                    t30_non_ecm_put_bit(&(s->t30_state), PUTBIT_TRAINING_SUCCEEDED);
                    s->rx_signal_present = TRUE;
                }
                bit_reverse(buf2, buf, len);
                t30_non_ecm_put_chunk(&(s->t30_state), buf2, len);
            }
            /* WORKAROUND: At least some Mediatrix boxes have a bug, where they can send HDLC signal end where
                           they should send non-ECM signal end. It is possible they also do the opposite.
                           We need to tolerate this, so we use the generic receive complete
                           indication, rather than the specific non-ECM carrier down. */
            t30_front_end_status(&(s->t30_state), T30_FRONT_END_RECEIVE_COMPLETE);
        }
        s->rx_signal_present = FALSE;
        s->timeout_rx_samples = 0;
        break;
    case T38_FIELD_CM_MESSAGE:
    case T38_FIELD_JM_MESSAGE:
    case T38_FIELD_CI_MESSAGE:
    case T38_FIELD_V34RATE:
    default:
        break;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void send_hdlc(void *user_data, const uint8_t *msg, int len)
{
    t38_terminal_state_t *s;

    s = (t38_terminal_state_t *) user_data;
    if (len <= 0)
    {
        s->tx_len = -1;
    }
    else
    {
        bit_reverse(s->tx_buf, msg, len);
        s->tx_len = len;
        s->tx_ptr = 0;
    }
}
/*- End of function --------------------------------------------------------*/

static void set_octets_per_data_packet(t38_terminal_state_t *s, int bit_rate)
{
    if (s->ms_per_tx_chunk == 0)
        s->octets_per_data_packet = MAX_OCTETS_PER_UNPACED_CHUNK;
    else
        s->octets_per_data_packet = s->ms_per_tx_chunk*bit_rate/(8*1000);
}
/*- End of function --------------------------------------------------------*/

int t38_terminal_send_timeout(t38_terminal_state_t *s, int samples)
{
    int len;
    int i;
    int previous;
    uint8_t buf[MAX_OCTETS_PER_UNPACED_CHUNK + 50];
    t38_data_field_t data_fields[2];
    /* Training times for all the modem options, with and without TEP, and with and without HDLC preamble.
       Note that the preamble for V.21 is 1s+-15%, and for the other modems is 200ms+100ms. */
    static const struct
    {
        int without_tep;
        int with_tep;
        int without_tep_with_flags;
        int with_tep_with_flags;
    } training_time[] =
    {
        {   0,    0,    0,    0},   /* T38_IND_NO_SIGNAL */
        {   0,    0,    0,    0},   /* T38_IND_CNG */
        {   0,    0,    0,    0},   /* T38_IND_CED */
        {   0,    0, 1000, 1000},   /* T38_IND_V21_PREAMBLE */ /* TODO: 850 should be OK for this, but it causes trouble with some ATAs. Why? */
        { 943, 1158, 1143, 1158},   /* T38_IND_V27TER_2400_TRAINING */
        { 708,  923,  908, 1123},   /* T38_IND_V27TER_4800_TRAINING */
        { 234,  454,  434,  654},   /* T38_IND_V29_7200_TRAINING */
        { 234,  454,  434,  654},   /* T38_IND_V29_9600_TRAINING */
        { 142,  367,  342,  567},   /* T38_IND_V17_7200_SHORT_TRAINING */
        {1393, 1618, 1593, 1818},   /* T38_IND_V17_7200_LONG_TRAINING */
        { 142,  367,  342,  567},   /* T38_IND_V17_9600_SHORT_TRAINING */
        {1393, 1618, 1593, 1818},   /* T38_IND_V17_9600_LONG_TRAINING */
        { 142,  367,  342,  367},   /* T38_IND_V17_12000_SHORT_TRAINING */
        {1393, 1618, 1593, 1818},   /* T38_IND_V17_12000_LONG_TRAINING */
        { 142,  367,  342,  567},   /* T38_IND_V17_14400_SHORT_TRAINING */
        {1393, 1618, 1593, 1818},   /* T38_IND_V17_14400_LONG_TRAINING */
        {   0,    0,    0,    0},   /* T38_IND_V8_ANSAM */
        {   0,    0,    0,    0},   /* T38_IND_V8_SIGNAL */
        {   0,    0,    0,    0},   /* T38_IND_V34_CNTL_CHANNEL_1200 */
        {   0,    0,    0,    0},   /* T38_IND_V34_PRI_CHANNEL */
        {   0,    0,    0,    0},   /* T38_IND_V34_CC_RETRAIN */
        {   0,    0,    0,    0},   /* T38_IND_V33_12000_TRAINING */
        {   0,    0,    0,    0}    /* T38_IND_V33_14400_TRAINING */
    };

    if (s->current_rx_type == T30_MODEM_DONE  ||  s->current_tx_type == T30_MODEM_DONE)
        return TRUE;

    s->samples += samples;
    t30_timer_update(&s->t30_state, samples);
    if (s->timeout_rx_samples  &&  s->samples > s->timeout_rx_samples)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Timeout mid-receive\n");
        s->timeout_rx_samples = 0;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_RECEIVE_COMPLETE);
    }
    if (s->timed_step == T38_TIMED_STEP_NONE)
        return FALSE;
    if (s->samples < s->next_tx_samples)
        return FALSE;
    /* Its time to send something */
    switch (s->timed_step)
    {
    case T38_TIMED_STEP_NON_ECM_MODEM:
        /* Create a 75ms silence */
        if (s->t38.current_tx_indicator != T38_IND_NO_SIGNAL)
            t38_core_send_indicator(&s->t38, T38_IND_NO_SIGNAL, s->indicator_tx_count);
        s->timed_step = T38_TIMED_STEP_NON_ECM_MODEM_2;
        s->next_tx_samples += ms_to_samples(75);
        break;
    case T38_TIMED_STEP_NON_ECM_MODEM_2:
        /* Switch on a fast modem, and give the training time to complete */
        t38_core_send_indicator(&s->t38, s->next_tx_indicator, s->indicator_tx_count);
        s->timed_step = T38_TIMED_STEP_NON_ECM_MODEM_3;
        s->next_tx_samples += ms_to_samples((s->use_tep)  ?  training_time[s->next_tx_indicator].with_tep  :  training_time[s->next_tx_indicator].without_tep);
        break;
    case T38_TIMED_STEP_NON_ECM_MODEM_3:
        /* Send a chunk of non-ECM image data */
        /* T.38 says it is OK to send the last of the non-ECM data in the signal end message.
           However, I think the early versions of T.38 said the signal end message should not
           contain data. Hopefully, following the current spec will not cause compatibility
           issues. */
        len = t30_non_ecm_get_chunk(&s->t30_state, buf, s->octets_per_data_packet);
        bit_reverse(buf, buf, len);
        if (len < s->octets_per_data_packet)
        {
            /* That's the end of the image data. Do a little padding now */
            memset(buf + len, 0, s->octets_per_data_packet - len);
            s->trailer_bytes = 3*s->octets_per_data_packet + len;
            len = s->octets_per_data_packet;
            s->timed_step = T38_TIMED_STEP_NON_ECM_MODEM_4;
        }
        t38_core_send_data(&s->t38, s->current_tx_data_type, T38_FIELD_T4_NON_ECM_DATA, buf, len, DATA_TX_COUNT);
        s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T38_TIMED_STEP_NON_ECM_MODEM_4:
        /* This pads the end of the data with some zeros. If we just stop abruptly
           at the end of the EOLs, some ATAs fail to clean up properly before
           shutting down their transmit modem, and the last few rows of the image
           get corrupted. Simply delaying the no-signal message does not help for
           all implentations. It often appears to be ignored. */
        len = s->octets_per_data_packet;
        s->trailer_bytes -= len;
        if (s->trailer_bytes <= 0)
        {
            len += s->trailer_bytes;
            memset(buf, 0, len);
            t38_core_send_data(&s->t38, s->current_tx_data_type, T38_FIELD_T4_NON_ECM_SIG_END, buf, len, s->data_end_tx_count);
            s->timed_step = T38_TIMED_STEP_NON_ECM_MODEM_5;
            s->next_tx_samples += ms_to_samples(60);
            break;
        }
        memset(buf, 0, len);
        t38_core_send_data(&s->t38, s->current_tx_data_type, T38_FIELD_T4_NON_ECM_DATA, buf, len, DATA_TX_COUNT);
        s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T38_TIMED_STEP_NON_ECM_MODEM_5:
        /* This should not be needed, since the message above indicates the end of the signal, but it
           seems like it can improve compatibility with quirky implementations. */
        t38_core_send_indicator(&s->t38, T38_IND_NO_SIGNAL, s->indicator_tx_count);
        s->timed_step = T38_TIMED_STEP_NONE;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SEND_COMPLETE);
        break;
    case T38_TIMED_STEP_HDLC_MODEM:
        /* Send HDLC preambling */
        t38_core_send_indicator(&s->t38, s->next_tx_indicator, s->indicator_tx_count);
        s->next_tx_samples += ms_to_samples((s->use_tep)  ?  training_time[s->next_tx_indicator].with_tep_with_flags  :  training_time[s->next_tx_indicator].without_tep_with_flags);
        s->timed_step = T38_TIMED_STEP_HDLC_MODEM_2;
        break;
    case T38_TIMED_STEP_HDLC_MODEM_2:
        /* Send a chunk of HDLC data */
        i = s->tx_len - s->tx_ptr;
        if (s->octets_per_data_packet >= i)
        {
            /* The last part of the HDLC frame */
            if (s->merge_tx_fields)
            {
                /* Copy the data, as we might be about to refill the buffer it is in */
                memcpy(buf, &s->tx_buf[s->tx_ptr], i);
                data_fields[0].field_type = T38_FIELD_HDLC_DATA;
                data_fields[0].field = buf;
                data_fields[0].field_len = i;

                /* Now see about the next HDLC frame. This will tell us whether to send FCS_OK or FCS_OK_SIG_END */
                previous = s->current_tx_data_type;
                s->tx_ptr = 0;
                s->tx_len = 0;
                t30_front_end_status(&(s->t30_state), T30_FRONT_END_SEND_STEP_COMPLETE);
                /* The above step should have got the next HDLC step ready - either another frame, or an instruction to stop transmission. */
                if (s->tx_len < 0)
                {
                    data_fields[1].field_type = T38_FIELD_HDLC_FCS_OK_SIG_END;
                    s->timed_step = T38_TIMED_STEP_HDLC_MODEM_4;
                }
                else
                {
                    data_fields[1].field_type = T38_FIELD_HDLC_FCS_OK;
                    s->timed_step = T38_TIMED_STEP_HDLC_MODEM_2;
                }
                data_fields[1].field = NULL;
                data_fields[1].field_len = 0;
                t38_core_send_data_multi_field(&s->t38, s->current_tx_data_type, data_fields, 2, DATA_TX_COUNT);
            }
            else
            {
                t38_core_send_data(&s->t38, s->current_tx_data_type, T38_FIELD_HDLC_DATA, &s->tx_buf[s->tx_ptr], i, DATA_TX_COUNT);
                s->timed_step = T38_TIMED_STEP_HDLC_MODEM_3;
            }
            s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
            break;
        }
        t38_core_send_data(&s->t38, s->current_tx_data_type, T38_FIELD_HDLC_DATA, &s->tx_buf[s->tx_ptr], s->octets_per_data_packet, DATA_TX_COUNT);
        s->tx_ptr += s->octets_per_data_packet;
        s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T38_TIMED_STEP_HDLC_MODEM_3:
        /* End of HDLC frame */
        previous = s->current_tx_data_type;
        s->tx_ptr = 0;
        s->tx_len = 0;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SEND_STEP_COMPLETE);
        /* The above step should have got the next HDLC step ready - either another frame, or an instruction to stop transmission. */
        if (s->tx_len < 0)
        {
            t38_core_send_data(&s->t38, previous, T38_FIELD_HDLC_FCS_OK_SIG_END, NULL, 0, s->data_end_tx_count);
            s->timed_step = T38_TIMED_STEP_HDLC_MODEM_4;
            s->next_tx_samples += ms_to_samples(100);
            break;
        }
        t38_core_send_data(&s->t38, previous, T38_FIELD_HDLC_FCS_OK, NULL, 0, DATA_TX_COUNT);
        if (s->tx_len)
            s->timed_step = T38_TIMED_STEP_HDLC_MODEM_2;
        s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T38_TIMED_STEP_HDLC_MODEM_4:
        /* Note that some boxes do not like us sending a T38_FIELD_HDLC_SIG_END at this point.
           A T38_IND_NO_SIGNAL should always be OK. */
        t38_core_send_indicator(&s->t38, T38_IND_NO_SIGNAL, s->indicator_tx_count);
        s->tx_len = 0;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SEND_STEP_COMPLETE);
        /* The above step might have started a whole new HDLC sequence */
        if (s->tx_len)
        {
            s->timed_step = T38_TIMED_STEP_HDLC_MODEM;
            s->next_tx_samples += ms_to_samples(s->ms_per_tx_chunk);
        }
        break;
    case T38_TIMED_STEP_CED:
        /* It seems common practice to start with a no signal indicator, though
           this is not a specified requirement. Since we should be sending 200ms
           of silence, starting the delay with a no signal indication makes sense.
           We do need a 200ms delay, as that is a specification requirement. */
        s->timed_step = T38_TIMED_STEP_CED_2;
        s->next_tx_samples = s->samples + ms_to_samples(200);
        t38_core_send_indicator(&s->t38, T38_IND_NO_SIGNAL, s->indicator_tx_count);
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T38_TIMED_STEP_CED_2:
        /* Initial 200ms delay over. Send the CED indicator */
        s->next_tx_samples = s->samples + ms_to_samples(3000);
        s->timed_step = T38_TIMED_STEP_PAUSE;
        t38_core_send_indicator(&s->t38, T38_IND_CED, s->indicator_tx_count);
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T38_TIMED_STEP_CNG:
        /* It seems common practice to start with a no signal indicator, though
           this is not a specified requirement. Since we should be sending 200ms
           of silence, starting the delay with a no signal indication makes sense.
           We do need a 200ms delay, as that is a specification requirement. */
        s->timed_step = T38_TIMED_STEP_CNG_2;
        s->next_tx_samples = s->samples + ms_to_samples(200);
        t38_core_send_indicator(&s->t38, T38_IND_NO_SIGNAL, s->indicator_tx_count);
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T38_TIMED_STEP_CNG_2:
        /* Initial short delay over. Send the CNG indicator */
        s->timed_step = T38_TIMED_STEP_NONE;
        t38_core_send_indicator(&s->t38, T38_IND_CNG, s->indicator_tx_count);
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T38_TIMED_STEP_PAUSE:
        /* End of timed pause */
        s->timed_step = T38_TIMED_STEP_NONE;
        t30_front_end_status(&(s->t30_state), T30_FRONT_END_SEND_STEP_COMPLETE);
        break;
    }
    return FALSE;
}
/*- End of function --------------------------------------------------------*/

static void set_rx_type(void *user_data, int type, int short_train, int use_hdlc)
{
    t38_terminal_state_t *s;

    s = (t38_terminal_state_t *) user_data;
    span_log(&s->logging, SPAN_LOG_FLOW, "Set rx type %d\n", type);
    s->current_rx_type = type;
}
/*- End of function --------------------------------------------------------*/

static void set_tx_type(void *user_data, int type, int short_train, int use_hdlc)
{
    t38_terminal_state_t *s;

    s = (t38_terminal_state_t *) user_data;
    span_log(&s->logging, SPAN_LOG_FLOW, "Set tx type %d\n", type);
    if (s->current_tx_type == type)
        return;

    switch (type)
    {
    case T30_MODEM_NONE:
        s->timed_step = T38_TIMED_STEP_NONE;
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T30_MODEM_PAUSE:
        s->next_tx_samples = s->samples + ms_to_samples(short_train);
        s->timed_step = T38_TIMED_STEP_PAUSE;
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T30_MODEM_CED:
        /* A 200ms initial delay is specified. Delay this amount before the CED indicator is sent. */
        s->next_tx_samples = s->samples;
        s->timed_step = T38_TIMED_STEP_CED;
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T30_MODEM_CNG:
        /* Allow a short initial delay, so the chances of the other end actually being ready to receive
           the CNG indicator are improved. */
        s->next_tx_samples = s->samples;
        s->timed_step = T38_TIMED_STEP_CNG;
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    case T30_MODEM_V21:
        if (s->current_tx_type > T30_MODEM_V21)
        {
            /* Pause before switching from phase C, as per T.30. If we omit this, the receiver
               might not see the carrier fall between the high speed and low speed sections. */
            s->next_tx_samples = s->samples + ms_to_samples(75);
        }
        else
        {
            s->next_tx_samples = s->samples;
        }
        set_octets_per_data_packet(s, 300);
        s->next_tx_indicator = T38_IND_V21_PREAMBLE;
        s->current_tx_data_type = T38_DATA_V21;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        break;
    case T30_MODEM_V27TER_2400:
        set_octets_per_data_packet(s, 2400);
        s->next_tx_indicator = T38_IND_V27TER_2400_TRAINING;
        s->current_tx_data_type = T38_DATA_V27TER_2400;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V27TER_4800:
        set_octets_per_data_packet(s, 4800);
        s->next_tx_indicator = T38_IND_V27TER_4800_TRAINING;
        s->current_tx_data_type = T38_DATA_V27TER_4800;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V29_7200:
        set_octets_per_data_packet(s, 7200);
        s->next_tx_indicator = T38_IND_V29_7200_TRAINING;
        s->current_tx_data_type = T38_DATA_V29_7200;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V29_9600:
        set_octets_per_data_packet(s, 9600);
        s->next_tx_indicator = T38_IND_V29_9600_TRAINING;
        s->current_tx_data_type = T38_DATA_V29_9600;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V17_7200:
        set_octets_per_data_packet(s, 7200);
        s->next_tx_indicator = (short_train)  ?  T38_IND_V17_7200_SHORT_TRAINING  :  T38_IND_V17_7200_LONG_TRAINING;
        s->current_tx_data_type = T38_DATA_V17_7200;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V17_9600:
        set_octets_per_data_packet(s, 9600);
        s->next_tx_indicator = (short_train)  ?  T38_IND_V17_9600_SHORT_TRAINING  :  T38_IND_V17_9600_LONG_TRAINING;
        s->current_tx_data_type = T38_DATA_V17_9600;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V17_12000:
        set_octets_per_data_packet(s, 12000);
        s->next_tx_indicator = (short_train)  ?  T38_IND_V17_12000_SHORT_TRAINING  :  T38_IND_V17_12000_LONG_TRAINING;
        s->current_tx_data_type = T38_DATA_V17_12000;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_V17_14400:
        set_octets_per_data_packet(s, 14400);
        s->next_tx_indicator = (short_train)  ?  T38_IND_V17_14400_SHORT_TRAINING  :  T38_IND_V17_14400_LONG_TRAINING;
        s->current_tx_data_type = T38_DATA_V17_14400;
        s->timed_step = (use_hdlc)  ?  T38_TIMED_STEP_HDLC_MODEM  :  T38_TIMED_STEP_NON_ECM_MODEM;
        s->next_tx_samples = s->samples + ms_to_samples(s->ms_per_tx_chunk);
        break;
    case T30_MODEM_DONE:
        span_log(&s->logging, SPAN_LOG_FLOW, "FAX exchange complete\n");
        s->timed_step = T38_TIMED_STEP_NONE;
        s->current_tx_data_type = T38_DATA_NONE;
        break;
    }
    s->current_tx_type = type;
}
/*- End of function --------------------------------------------------------*/

void t38_terminal_set_config(t38_terminal_state_t *s, int without_pacing)
{
    if (without_pacing)
    {
        /* Continuous streaming mode, as used for TPKT over TCP transport */
        s->indicator_tx_count = 0;
        s->data_end_tx_count = 1;
        s->ms_per_tx_chunk = 0;
    }
    else
    {
        /* Paced streaming mode, as used for UDP transports */
        s->indicator_tx_count = INDICATOR_TX_COUNT;
        s->data_end_tx_count = DATA_END_TX_COUNT;
        s->ms_per_tx_chunk = MS_PER_TX_CHUNK;
    }
}
/*- End of function --------------------------------------------------------*/

void t38_terminal_set_tep_mode(t38_terminal_state_t *s, int use_tep)
{
    s->use_tep = use_tep;
}
/*- End of function --------------------------------------------------------*/

t38_terminal_state_t *t38_terminal_init(t38_terminal_state_t *s,
                                        int calling_party,
                                        t38_tx_packet_handler_t *tx_packet_handler,
                                        void *tx_packet_user_data)
{
    if (tx_packet_handler == NULL)
        return NULL;

    if (s == NULL)
    {
        if ((s = (t38_terminal_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));
    span_log_init(&s->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&s->logging, "T.38T");
    s->rx_signal_present = FALSE;

    s->timed_step = T38_TIMED_STEP_NONE;
    s->tx_ptr = 0;

    t38_core_init(&s->t38,
                  process_rx_indicator,
                  process_rx_data,
                  process_rx_missing,
                  (void *) s,
                  tx_packet_handler,
                  tx_packet_user_data);
    s->t38.fastest_image_data_rate = 14400;
    t38_terminal_set_config(s, FALSE);

    s->timed_step = T38_TIMED_STEP_NONE;
    s->current_tx_data_type = T38_DATA_NONE;
    s->next_tx_samples = 0;
    s->merge_tx_fields = FALSE;

    t30_init(&(s->t30_state),
             calling_party,
             set_rx_type,
             (void *) s,
             set_tx_type,
             (void *) s,
             send_hdlc,
             (void *) s);
    /* TODO: This needs to go where the operation starts up. The variable will not
             have been set at this stage. */
    if (s->t38.fill_bit_removal)
        t30_set_min_non_ecm_row_bits(&(s->t30_state), 0);
    t30_set_supported_modems(&(s->t30_state),
                             T30_SUPPORT_V27TER | T30_SUPPORT_V29 | T30_SUPPORT_V17 | T30_SUPPORT_IAF);
    t30_set_iaf_mode(&(s->t30_state), T30_IAF_MODE_T37 | T30_IAF_MODE_T38);
    t30_restart(&s->t30_state);
    return s;
}
/*- End of function --------------------------------------------------------*/

int t38_terminal_release(t38_terminal_state_t *s)
{
    t30_release(&s->t30_state);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t38_terminal_free(t38_terminal_state_t *s)
{
    t30_release(&s->t30_state);
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
