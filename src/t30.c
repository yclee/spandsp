/*
 * SpanDSP - a series of DSP components for telephony
 *
 * t30.c - ITU T.30 FAX transfer processing
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2003, 2004, 2005, 2006, 2007 Steve Underwood
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
 * $Id: t30.c,v 1.214 2007/11/30 12:20:34 steveu Exp $
 */

/*! \file */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif
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
#include "spandsp/t4.h"

#include "spandsp/t30_fcf.h"
#include "spandsp/t35.h"
#include "spandsp/t30.h"

#define MAX_MESSAGE_TRIES   3

#define ms_to_samples(t)    (((t)*SAMPLE_RATE)/1000)

typedef struct
{
    int val;
    const char *str;
} value_string_t;

/* T.30 defines the following call phases:
   Phase A: Call set-up.
       Exchange of CNG, CED and the called terminal identification.
   Phase B: Pre-message procedure for identifying and selecting the required facilities.
       Capabilities negotiation, and training, up the the confirmation to receive.
   Phase C: Message transmission (includes phasing and synchronization where appropriate).
       Transfer of the message at high speed.
   Phase D: Post-message procedure, including end-of-message and confirmation and multi-document procedures.
       End of message and acknowledgement.
   Phase E: Call release
       Final call disconnect. */
enum
{
    T30_PHASE_IDLE = 0,     /* Freshly initialised */
    T30_PHASE_A_CED,        /* Doing the CED (answer) sequence */
    T30_PHASE_A_CNG,        /* Doing the CNG (caller) sequence */
    T30_PHASE_B_RX,         /* Receiving pre-message control messages */
    T30_PHASE_B_TX,         /* Transmitting pre-message control messages */
    T30_PHASE_C_NON_ECM_RX, /* Receiving a document message in non-ECM mode */
    T30_PHASE_C_NON_ECM_TX, /* Transmitting a document message in non-ECM mode */
    T30_PHASE_C_ECM_RX,     /* Receiving a document message in ECM (HDLC) mode */
    T30_PHASE_C_ECM_TX,     /* Transmitting a document message in ECM (HDLC) mode */
    T30_PHASE_D_RX,         /* Receiving post-message control messages */
    T30_PHASE_D_TX,         /* Transmitting post-message control messages */
    T30_PHASE_E,            /* In phase E */
    T30_PHASE_CALL_FINISHED /* Call completely finished */
};

static const char *phase_names[] =
{
    "T30_PHASE_IDLE",
    "T30_PHASE_A_CED",
    "T30_PHASE_A_CNG",
    "T30_PHASE_B_RX",
    "T30_PHASE_B_TX",
    "T30_PHASE_C_NON_ECM_RX",
    "T30_PHASE_C_NON_ECM_TX",
    "T30_PHASE_C_ECM_RX",
    "T30_PHASE_C_ECM_TX",
    "T30_PHASE_D_RX",
    "T30_PHASE_D_TX",
    "T30_PHASE_E",
    "T30_PHASE_CALL_FINISHED"
};

/* These state names are modelled after places in the T.30 flow charts. */
enum
{
    T30_STATE_ANSWERING = 1,
    T30_STATE_B,
    T30_STATE_C,
    T30_STATE_D,
    T30_STATE_D_TCF,
    T30_STATE_D_POST_TCF,
    T30_STATE_F_TCF,
    T30_STATE_F_CFR,
    T30_STATE_F_FTT,
    T30_STATE_F_DOC_NON_ECM,
    T30_STATE_F_POST_DOC_NON_ECM,
    T30_STATE_F_DOC_ECM,
    T30_STATE_F_POST_DOC_ECM,
    T30_STATE_F_POST_RCP_MCF,
    T30_STATE_F_POST_RCP_PPR,
    T30_STATE_F_POST_RCP_RNR,
    T30_STATE_R,
    T30_STATE_T,
    T30_STATE_I,
    T30_STATE_II,
    T30_STATE_II_Q,
    T30_STATE_III_Q_MCF,
    T30_STATE_III_Q_RTP,
    T30_STATE_III_Q_RTN,
    T30_STATE_IV,
    T30_STATE_IV_PPS_NULL,
    T30_STATE_IV_PPS_Q,
    T30_STATE_IV_PPS_RNR,
    T30_STATE_IV_CTC,
    T30_STATE_IV_EOR,
    T30_STATE_IV_EOR_RNR,
    T30_STATE_CALL_FINISHED
};

enum
{
    T30_MIN_SCAN_20MS = 0,
    T30_MIN_SCAN_5MS = 1,
    T30_MIN_SCAN_10MS = 2,
    T30_MIN_SCAN_40MS = 4,
    T30_MIN_SCAN_0MS = 7,
};

enum
{
    T30_MODE_SEND_DOC = 1,
    T30_MODE_RECEIVE_DOC
};

enum
{
    T30_COPY_QUALITY_GOOD = 0,
    T30_COPY_QUALITY_POOR,
    T30_COPY_QUALITY_BAD
};

enum
{
    DISBIT1 = 0x01,
    DISBIT2 = 0x02,
    DISBIT3 = 0x04,
    DISBIT4 = 0x08,
    DISBIT5 = 0x10,
    DISBIT6 = 0x20,
    DISBIT7 = 0x40,
    DISBIT8 = 0x80
};

/* All timers specified in milliseconds */

/* Time-out T0 defines the amount of time an automatic calling terminal waits for the called terminal
to answer the call.
T0 begins after the dialling of the number is completed and is reset:
a)       when T0 times out; or
b)       when timer T1 is started; or
c)       if the terminal is capable of detecting any condition which indicates that the call will not be
         successful, when such a condition is detected.
The recommended value of T0 is 60+-5s; however, when it is anticipated that a long call set-up
time may be encountered, an alternative value of up to 120s may be used.
NOTE - National regulations may require the use of other values for T0. */
#define DEFAULT_TIMER_T0                60000

/* Time-out T1 defines the amount of time two terminals will continue to attempt to identify each
other. T1 is 35+-5s, begins upon entering phase B, and is reset upon detecting a valid signal or
when T1 times out.
For operating methods 3 and 4 (see 3.1), the calling terminal starts time-out T1 upon reception of
the V.21 modulation scheme.
For operating method 4 bis a (see 3.1), the calling terminal starts time-out T1 upon starting
transmission using the V.21 modulation scheme. */
#define DEFAULT_TIMER_T1                35000

/* Time-out T2 makes use of the tight control between commands and responses to detect the loss of
command/response synchronization. T2 is 6+-1s and begins when initiating a command search
(e.g., the first entrance into the "command received" subroutine, reference flow diagram in 5.2).
T2 is reset when an HDLC flag is received or when T2 times out. */
#define DEFAULT_TIMER_T2                7000

/* Time-out T3 defines the amount of time a terminal will attempt to alert the local operator in
response to a procedural interrupt. Failing to achieve operator intervention, the terminal will
discontinue this attempt and shall issue other commands or responses. T3 is 10+-5s, begins on the
first detection of a procedural interrupt command/response signal (i.e., PIN/PIP or PRI-Q) and is
reset when T3 times out or when the operator initiates a line request. */
#define DEFAULT_TIMER_T3                15000

/* NOTE - For manual FAX units, the value of timer T4 may be either 3.0s +-15% or 4.5s +-15%.
If the value of 4.5s is used, then after detection of a valid response to the first DIS, it may
be reduced to 3.0s +-15%. T4 = 3.0s +-15% for automatic units. */
#define DEFAULT_TIMER_T4                3450

/* Time-out T5 is defined for the optional T.4 error correction mode. Time-out T5 defines the amount
of time waiting for clearance of the busy condition of the receiving terminal. T5 is 60+-5s and
begins on the first detection of the RNR response. T5 is reset when T5 times out or the MCF or PIP
response is received or when the ERR or PIN response is received in the flow control process after
transmitting the EOR command. If the timer T5 has expired, the DCN command is transmitted for
call release. */
#define DEFAULT_TIMER_T5                65000

#define DEFAULT_TIMER_T6                5000

#define DEFAULT_TIMER_T7                6000

#define DEFAULT_TIMER_T8                10000

/* Final time we allow for things to flush through the system, before we disconnect, in milliseconds.
   200ms should be fine for a PSTN call. For a T.38 call something longer is desirable. */
#define FINAL_FLUSH_TIME                1000

/* Start points in the fallback table for different capabilities */
#define T30_V17_FALLBACK_START          0
#define T30_V29_FALLBACK_START          3
#define T30_V27TER_FALLBACK_START       6

static const struct
{
    int bit_rate;
    int modem_type;
    int which;
    uint8_t dcs_code;
} fallback_sequence[] =
{
    {14400, T30_MODEM_V17_14400,    T30_SUPPORT_V17,    DISBIT6},
    {12000, T30_MODEM_V17_12000,    T30_SUPPORT_V17,    (DISBIT6 | DISBIT4)},
    { 9600, T30_MODEM_V17_9600,     T30_SUPPORT_V17,    (DISBIT6 | DISBIT3)},
    { 9600, T30_MODEM_V29_9600,     T30_SUPPORT_V29,    DISBIT3},
    { 7200, T30_MODEM_V17_7200,     T30_SUPPORT_V17,    (DISBIT6 | DISBIT4 | DISBIT3)},
    { 7200, T30_MODEM_V29_7200,     T30_SUPPORT_V29,    (DISBIT4 | DISBIT3)},
    { 4800, T30_MODEM_V27TER_4800,  T30_SUPPORT_V27TER, DISBIT4},
    { 2400, T30_MODEM_V27TER_2400,  T30_SUPPORT_V27TER, 0},
    {    0, 0, 0, 0}
};

static void queue_phase(t30_state_t *s, int phase);
static void set_phase(t30_state_t *s, int phase);
static void set_state(t30_state_t *s, int state);
static void send_simple_frame(t30_state_t *s, int type);
static void send_frame(t30_state_t *s, const uint8_t *fr, int frlen);
static void send_dcn(t30_state_t *s);
static void repeat_last_command(t30_state_t *s);
static void disconnect(t30_state_t *s);
static void decode_20digit_msg(t30_state_t *s, char *msg, const uint8_t *pkt, int len);
static void decode_url_msg(t30_state_t *s, char *msg, const uint8_t *pkt, int len);

#define set_dis_dtc_bit(s,bit) s->dis_dtc_frame[3 + ((bit - 1)/8)] |= (1 << ((bit - 1)%8))
#define set_dis_dtc_bits(s,val,bit) s->dis_dtc_frame[3 + ((bit - 1)/8)] |= ((val) << ((bit - 1)%8))
#define clr_dis_dtc_bit(s,bit) s->dis_dtc_frame[3 + ((bit - 1)/8)] &= ~(1 << ((bit - 1)%8))

#define test_bit(s,bit) (s[3 + ((bit - 1)/8)] & (1 << ((bit - 1)%8)))

#define set_dcs_bit(s,bit) s->dcs_frame[3 + ((bit - 1)/8)] |= (1 << ((bit - 1)%8))
#define set_dcs_bits(s,val,bit) s->dcs_frame[3 + ((bit - 1)/8)] |= ((val) << ((bit - 1)%8))
#define clr_dcs_bit(s,bit) s->dcs_frame[3 + ((bit - 1)/8)] &= ~(1 << ((bit - 1)%8))

static void rx_start_page(t30_state_t *s)
{
    int i;

    t4_rx_set_image_width(&(s->t4), s->image_width);
    t4_rx_set_sub_address(&(s->t4), s->far_sub_address);
    t4_rx_set_dcs(&(s->t4), s->rx_dcs_string);
    t4_rx_set_far_ident(&(s->t4), s->far_ident);
    t4_rx_set_vendor(&(s->t4), s->vendor);
    t4_rx_set_model(&(s->t4), s->model);

    t4_rx_set_rx_encoding(&(s->t4), s->line_encoding);
    t4_rx_set_x_resolution(&(s->t4), s->x_resolution);
    t4_rx_set_y_resolution(&(s->t4), s->y_resolution);

    t4_rx_start_page(&(s->t4));
    /* Clear the buffer */
    for (i = 0;  i < 256;  i++)
        s->ecm_len[i] = -1;
    s->ecm_page++;
    s->ecm_block = 0;
    s->ecm_frames = -1;
    s->ecm_frames_this_burst = 0;
}
/*- End of function --------------------------------------------------------*/

static int copy_quality(t30_state_t *s)
{
    t4_stats_t stats;

    /* There is no specification for judging copy quality. However, we need to classify
       it at three levels, to control what we do next: OK; tolerable, but retrain;
       intolerable, so retrain. */
    t4_get_transfer_statistics(&(s->t4), &stats);
    span_log(&s->logging, SPAN_LOG_FLOW, "Page no = %d\n", stats.pages_transferred + 1);
    span_log(&s->logging, SPAN_LOG_FLOW, "Image size = %d x %d pixels\n", stats.width, stats.length);
    span_log(&s->logging, SPAN_LOG_FLOW, "Image resolution = %d/m x %d/m\n", stats.x_resolution, stats.y_resolution);
    span_log(&s->logging, SPAN_LOG_FLOW, "Bad rows = %d\n", stats.bad_rows);
    span_log(&s->logging, SPAN_LOG_FLOW, "Longest bad row run = %d\n", stats.longest_bad_row_run);
    if (stats.bad_rows*50 < stats.length)
        return T30_COPY_QUALITY_GOOD;
    if (stats.bad_rows*20 < stats.length)
        return T30_COPY_QUALITY_POOR;
    return T30_COPY_QUALITY_BAD;
}
/*- End of function --------------------------------------------------------*/

const char *t30_completion_code_to_str(int result)
{
    switch (result)
    {
    case T30_ERR_OK:
        return "OK";
    case T30_ERR_CEDTONE:
        return "The CED tone exceeded 5s";
    case T30_ERR_T0_EXPIRED:
        return "Timed out waiting for initial communication";
    case T30_ERR_T1_EXPIRED:
        return "Timed out waiting for the first message";
    case T30_ERR_T3_EXPIRED:
        return "Timed out waiting for procedural interrupt";
    case T30_ERR_HDLC_CARRIER:
        return "The HDLC carrier did not stop in a timely manner";
    case T30_ERR_CANNOT_TRAIN:
        return "Failed to train with any of the compatible modems";
    case T30_ERR_OPERINTFAIL:
        return "Operator intervention failed";
    case T30_ERR_INCOMPATIBLE:
        return "Far end is not compatible";
    case T30_ERR_RX_INCAPABLE:
        return "Far end is not able to receive";
    case T30_ERR_TX_INCAPABLE:
        return "Far end is not able to transmit";
    case T30_ERR_NORESSUPPORT:
        return "Far end cannot receive at the resolution of the image";
    case T30_ERR_NOSIZESUPPORT:
        return "Far end cannot receive at the size of image";
    case T30_ERR_UNEXPECTED:
        return "Unexpected message received";
    case T30_ERR_FILEERROR:
        return "TIFF/F file cannot be opened";
    case T30_ERR_NOPAGE:
        return "TIFF/F page not found";
    case T30_ERR_BADTIFF:
        return "TIFF/F format is not compatible";
    case T30_ERR_BADDCSTX:
        return "Received bad response to DCS or training";
    case T30_ERR_BADPGTX:
        return "Received a DCN from remote after sending a page";
    case T30_ERR_ECMPHDTX:
        return "Invalid ECM response received from receiver";
    case T30_ERR_T5_EXPIRED:
        return "Timed out waiting for receiver ready (ECM mode)";
    case T30_ERR_GOTDCNTX:
        return "Received a DCN while waiting for a DIS";
    case T30_ERR_INVALRSPTX:
        return "Invalid response after sending a page";
    case T30_ERR_NODISTX:
        return "Received other than DIS while waiting for DIS";
    case T30_ERR_PHBDEADTX:
        return "Received no response to DCS or TCF";
    case T30_ERR_PHDDEADTX:
        return "No response after sending a page";
    case T30_ERR_ECMPHDRX:
        return "Invalid ECM response received from transmitter";
    case T30_ERR_GOTDCSRX:
        return "DCS received while waiting for DTC";
    case T30_ERR_INVALCMDRX:
        return "Unexpected command after page received";
    case T30_ERR_NOCARRIERRX:
        return "Carrier lost during fax receive";
    case T30_ERR_NOEOLRX:
        return "Timed out while waiting for EOL (end Of line)";
    case T30_ERR_NOFAXRX:
        return "Timed out while waiting for first line";
    case T30_ERR_T2EXPDCNRX:
        return "Timer T2 expired while waiting for DCN";
    case T30_ERR_T2EXPDRX:
        return "Timer T2 expired while waiting for phase D";
    case T30_ERR_T2EXPFAXRX:
        return "Timer T2 expired while waiting for fax page";
    case T30_ERR_T2EXPMPSRX:
        return "Timer T2 expired while waiting for next fax page";
    case T30_ERR_T2EXPRRRX:
        return "Timer T2 expired while waiting for RR command";
    case T30_ERR_T2EXPRX:
        return "Timer T2 expired while waiting for NSS, DCS or MCF";
    case T30_ERR_DCNWHYRX:
        return "Unexpected DCN while waiting for DCS or DIS";
    case T30_ERR_DCNDATARX:
        return "Unexpected DCN while waiting for image data";
    case T30_ERR_DCNFAXRX:
        return "Unexpected DCN while waiting for EOM, EOP or MPS";
    case T30_ERR_DCNPHDRX:
        return "Unexpected DCN after EOM or MPS sequence";
    case T30_ERR_DCNRRDRX:
        return "Unexpected DCN after RR/RNR sequence";
    case T30_ERR_DCNNORTNRX:
        return "Unexpected DCN after requested retransmission";
    case T30_ERR_BADPAGE:
        return "TIFF/F page number tag missing";
    case T30_ERR_BADTAG:
        return "Incorrect values for TIFF/F tags";
    case T30_ERR_BADTIFFHDR:
        return "Bad TIFF/F header - incorrect values in fields";
    case T30_ERR_NODATA:
        return "Data requested is not available (NSF, DIS, DCS)";
    case T30_ERR_NOMEM:
        return "Cannot allocate memory for more pages";
    case T30_ERR_NOPOLL:
        return "Poll not accepted";
    case T30_ERR_RETRYDCN:
        return "Disconnected after permitted retries";
    case T30_ERR_CALLDROPPED:
        return "The call dropped prematurely";
    }
    return "???";
}
/*- End of function --------------------------------------------------------*/

void t30_non_ecm_put_bit(void *user_data, int bit)
{
    t30_state_t *s;
    int was_trained;

    s = (t30_state_t *) user_data;
    if (bit < 0)
    {
        /* Special conditions */
        switch (bit)
        {
        case PUTBIT_TRAINING_FAILED:
            span_log(&s->logging, SPAN_LOG_FLOW, "Non-ECM carrier training failed in state %d\n", s->state);
            s->rx_trained = FALSE;
            /* Cancel the timer, since we have actually seen something, and wait until the carrier drops
               before proceeding. */
            // TODO: this is not a complete answer to handling failures to train
            s->timer_t2_t4 = 0;
            break;
        case PUTBIT_TRAINING_SUCCEEDED:
            /* The modem is now trained */
            span_log(&s->logging, SPAN_LOG_FLOW, "Non-ECM carrier trained in state %d\n", s->state);
            /* In case we are in trainability test mode... */
            s->training_current_zeros = 0;
            s->training_most_zeros = 0;
            s->rx_signal_present = TRUE;
            s->rx_trained = TRUE;
            s->timer_t2_t4 = 0;
            break;
        case PUTBIT_CARRIER_UP:
            span_log(&s->logging, SPAN_LOG_FLOW, "Non-ECM carrier up in state %d\n", s->state);
            break;
        case PUTBIT_CARRIER_DOWN:
            span_log(&s->logging, SPAN_LOG_FLOW, "Non-ECM carrier down in state %d\n", s->state);
            was_trained = s->rx_trained;
            s->rx_signal_present = FALSE;
            s->rx_trained = FALSE;
            switch (s->state)
            {
            case T30_STATE_F_TCF:
                /* Only respond if we managed to actually sync up with the source. We don't
                   want to respond just because we saw a click. These often occur just
                   before the real signal, with many modems. Presumably this is due to switching
                   within the far end modem. We also want to avoid the possibility of responding
                   to the tail end of any slow modem signal. If there was a genuine data signal
                   which we failed to train on it should not matter. If things are that bad, we
                   do not stand much chance of good quality communications. */
                if (was_trained)
                {
                    /* Although T.30 says the training test should be 1.5s of all 0's, some FAX
                       machines send a burst of all 1's before the all 0's. Tolerate this. */
                    if (s->training_current_zeros > s->training_most_zeros)
                        s->training_most_zeros = s->training_current_zeros;
                    if (s->training_most_zeros < fallback_sequence[s->current_fallback].bit_rate)
                    {
                        span_log(&s->logging, SPAN_LOG_FLOW, "Trainability test failed - longest run of zeros was %d\n", s->training_most_zeros);
                        set_phase(s, T30_PHASE_B_TX);
                        set_state(s, T30_STATE_F_FTT);
                        send_simple_frame(s, T30_FTT);
                    }
                    else
                    {
                        /* The training went OK */
                        s->short_train = TRUE;
                        s->in_message = TRUE;
                        rx_start_page(s);
                        set_phase(s, T30_PHASE_B_TX);
                        set_state(s, T30_STATE_F_CFR);
                        send_simple_frame(s, T30_CFR);
                    }
                }
                break;
            case T30_STATE_F_POST_DOC_NON_ECM:
                /* Page ended cleanly */
                if (s->current_status == T30_ERR_NOCARRIERRX)
                    s->current_status = T30_ERR_OK;
                break;
            default:
                /* We should be receiving a document right now, but it did not end cleanly. */
                if (was_trained)
                {
                    span_log(&s->logging, SPAN_LOG_WARNING, "Page did not end cleanly\n");
                    /* We trained OK, so we should have some kind of received page, even though
                       it did not end cleanly. */
                    set_state(s, T30_STATE_F_POST_DOC_NON_ECM);
                    set_phase(s, T30_PHASE_D_RX);
                    s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
                    s->timer_is_t4 = FALSE;
                    if (s->current_status == T30_ERR_NOCARRIERRX)
                        s->current_status = T30_ERR_OK;
                }
                else
                {
                    span_log(&s->logging, SPAN_LOG_WARNING, "Non-ECM carrier not found\n");
                    s->current_status = T30_ERR_NOCARRIERRX;
                }
                break;
            }
            if (s->next_phase != T30_PHASE_IDLE)
            {
                set_phase(s, s->next_phase);
                s->next_phase = T30_PHASE_IDLE;
            }
            break;
        default:
            span_log(&s->logging, SPAN_LOG_WARNING, "Unexpected non-ECM special bit - %d!\n", bit);
            break;
        }
        return;
    }
    switch (s->state)
    {
    case T30_STATE_F_TCF:
        /* Trainability test */
        if (bit)
        {
            if (s->training_current_zeros > s->training_most_zeros)
                s->training_most_zeros = s->training_current_zeros;
            s->training_current_zeros = 0;
        }
        else
        {
            s->training_current_zeros++;
        }
        break;
    case T30_STATE_F_DOC_NON_ECM:
        /* Document transfer */
        if (t4_rx_put_bit(&(s->t4), bit))
        {
            /* That is the end of the document */
            set_state(s, T30_STATE_F_POST_DOC_NON_ECM);
            queue_phase(s, T30_PHASE_D_RX);
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
        }
        break;
    }
}
/*- End of function --------------------------------------------------------*/

void t30_non_ecm_put_byte(void *user_data, int byte)
{
    t30_state_t *s;

    s = (t30_state_t *) user_data;
    switch (s->state)
    {
    case T30_STATE_F_TCF:
        /* Trainability test */
        /* This makes counting zeros fast, but approximate. That really doesn't matter */
        if (byte)
        {
            if (s->training_current_zeros > s->training_most_zeros)
                s->training_most_zeros = s->training_current_zeros;
            s->training_current_zeros = 0;
        }
        else
        {
            s->training_current_zeros += 8;
        }
        break;
    case T30_STATE_F_DOC_NON_ECM:
        /* Document transfer */
        if (t4_rx_put_byte(&(s->t4), byte))
        {
            /* That is the end of the document */
            set_state(s, T30_STATE_F_POST_DOC_NON_ECM);
            queue_phase(s, T30_PHASE_D_RX);
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
        }
        break;
    }
}
/*- End of function --------------------------------------------------------*/

void t30_non_ecm_put_chunk(void *user_data, const uint8_t buf[], int len)
{
    t30_state_t *s;
    int i;

    s = (t30_state_t *) user_data;
    switch (s->state)
    {
    case T30_STATE_F_TCF:
        /* Trainability test */
        /* This makes counting zeros fast, but approximate. That really doesn't matter */
        for (i = 0;  i < len;  i++)
        {
            if (buf[i])
            {
                if (s->training_current_zeros > s->training_most_zeros)
                    s->training_most_zeros = s->training_current_zeros;
                s->training_current_zeros = 0;
            }
            else
            {
                s->training_current_zeros += 8;
            }
        }
        break;
    case T30_STATE_F_DOC_NON_ECM:
        /* Document transfer */
        if (t4_rx_put_chunk(&(s->t4), buf, len))
        {
            /* That is the end of the document */
            set_state(s, T30_STATE_F_POST_DOC_NON_ECM);
            queue_phase(s, T30_PHASE_D_RX);
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
        }
        break;
    }
}
/*- End of function --------------------------------------------------------*/

int t30_non_ecm_get_bit(void *user_data)
{
    int bit;
    t30_state_t *s;

    s = (t30_state_t *) user_data;
    switch (s->state)
    {
    case T30_STATE_D_TCF:
        /* Trainability test. */
        bit = 0;
        if (s->training_test_bits-- < 0)
        {
            /* Finished sending training test. */
            bit = PUTBIT_END_OF_DATA;
        }
        break;
    case T30_STATE_I:
        /* Transferring real data. */
        bit = t4_tx_get_bit(&(s->t4));
        break;
    case T30_STATE_D_POST_TCF:
    case T30_STATE_II_Q:
        /* We should be padding out a block of samples if we are here */
        bit = 0;
        break;
    default:
        span_log(&s->logging, SPAN_LOG_WARNING, "t30_non_ecm_get_bit in bad state %d\n", s->state);
        bit = PUTBIT_END_OF_DATA;
        break;
    }
    return bit;
}
/*- End of function --------------------------------------------------------*/

int t30_non_ecm_get_byte(void *user_data)
{
    int byte;
    t30_state_t *s;

    s = (t30_state_t *) user_data;
    switch (s->state)
    {
    case T30_STATE_D_TCF:
        /* Trainability test. */
        byte = 0;
        if ((s->training_test_bits -= 8) < 0)
        {
            /* Finished sending training test. */
            byte = 0x100;
        }
        break;
    case T30_STATE_I:
        /* Transferring real data. */
        byte = t4_tx_get_byte(&(s->t4));
        break;
    case T30_STATE_D_POST_TCF:
    case T30_STATE_II_Q:
        /* We should be padding out a block of samples if we are here */
        byte = 0;
        break;
    default:
        span_log(&s->logging, SPAN_LOG_WARNING, "t30_non_ecm_get_byte in bad state %d\n", s->state);
        byte = 0x100;
        break;
    }
    return byte;
}
/*- End of function --------------------------------------------------------*/

int t30_non_ecm_get_chunk(void *user_data, uint8_t buf[], int max_len)
{
    int len;
    t30_state_t *s;

    s = (t30_state_t *) user_data;
    switch (s->state)
    {
    case T30_STATE_D_TCF:
        /* Trainability test. */
        for (len = 0;  len < max_len;  len++)
        {
            buf[len] = 0;
            if ((s->training_test_bits -= 8) < 0)
                break;
        }
        break;
    case T30_STATE_I:
        /* Transferring real data. */
        len = t4_tx_get_chunk(&(s->t4), buf, max_len);
        break;
    case T30_STATE_D_POST_TCF:
    case T30_STATE_II_Q:
        /* We should be padding out a block of samples if we are here */
        len = 0;
        break;
    default:
        span_log(&s->logging, SPAN_LOG_WARNING, "t30_non_ecm_get_chunk in bad state %d\n", s->state);
        len = 0;
        break;
    }
    return len;
}
/*- End of function --------------------------------------------------------*/

static int check_next_tx_step(t30_state_t *s)
{
    int more;

    if (t4_tx_more_pages(&(s->t4)) == 0)
        return (s->local_interrupt_pending)  ?  T30_PRI_MPS  :  T30_MPS;
    /* Call a user handler, if one is set, to check if another document is to be sent.
       If so, we send an EOM, rather than an EOP. Then we will renegotiate, and the new
       document will begin. */
    if (s->document_handler)
        more = s->document_handler(s, s->document_user_data, 0);
    else
        more = FALSE;
    if (more)
        return (s->local_interrupt_pending)  ?  T30_PRI_EOM  :  T30_EOM;
    return (s->local_interrupt_pending)  ?  T30_PRI_EOP  :  T30_EOP;
}
/*- End of function --------------------------------------------------------*/

static int get_partial_ecm_page(t30_state_t *s)
{
    int i;
    int len;

    s->ppr_count = 0;
    /* Fill our partial page buffer with a partial page. Use the negotiated preferred frame size
       as the basis for the size of the frames produced. */
    /* We fill the buffer with complete HDLC frames, ready to send out. */
    /* The frames are all marked as not being final frames. When sent, the are followed by a partial
       page signal, which is marked as the final frame. */
    for (i = 3;  i < 32 + 3;  i++)
        s->ecm_frame_map[i] = 0xFF;
    for (i = 0;  i < 256;  i++)
    {
        s->ecm_len[i] = -1;
        s->ecm_data[i][0] = 0xFF;
        s->ecm_data[i][1] = 0x03;
        s->ecm_data[i][2] = T4_FCD;
        /* These frames contain a frame sequence number within the partial page (one octet) followed
           by some image data. */
        s->ecm_data[i][3] = (uint8_t) i;
        if ((len = t4_tx_get_chunk(&(s->t4), &s->ecm_data[i][4], s->octets_per_ecm_frame)) < s->octets_per_ecm_frame)
        {
            /* The image is not big enough to fill the entire buffer */
            /* We need to pad to a full frame, as most receivers expect that. */
            if (len > 0)
            {
                memset(&s->ecm_data[i][4 + len], 0, s->octets_per_ecm_frame - len);
                s->ecm_len[i++] = (int16_t) (s->octets_per_ecm_frame + 4);
            }
            s->ecm_frames = i;
            span_log(&s->logging, SPAN_LOG_FLOW, "Partial page buffer contains %d frames (%d per frame)\n", i, s->octets_per_ecm_frame);
            s->ecm_at_page_end = TRUE;
            return i;
        }
        s->ecm_len[i] = (int16_t) 4 + len;
    }
    /* We filled the entire buffer */
    s->ecm_frames = 256;
    span_log(&s->logging, SPAN_LOG_FLOW, "Partial page buffer full (%d per frame)\n", s->octets_per_ecm_frame);
    s->ecm_at_page_end = ((t4_tx_check_bit(&(s->t4)) & 2) != 0);
    return 256;
}
/*- End of function --------------------------------------------------------*/

static int t30_ecm_commit_partial_page(t30_state_t *s)
{
    int i;

    span_log(&s->logging, SPAN_LOG_FLOW, "Commiting partial page - %d frames\n", s->ecm_frames);
    for (i = 0;  i < s->ecm_frames;  i++)
    {
        if (t4_rx_put_chunk(&(s->t4), s->ecm_data[i], s->ecm_len[i]))
        {
            /* This is the end of the document */
            /* Clear the buffer */
            for (i = 0;  i < 256;  i++)
                s->ecm_len[i] = -1;
            s->ecm_frames = -1;
            return -1;
        }
    }
    /* Clear the buffer */
    for (i = 0;  i < 256;  i++)
        s->ecm_len[i] = -1;
    s->ecm_frames = -1;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int send_next_ecm_frame(t30_state_t *s)
{
    int i;
    uint8_t frame[3];

    if (s->ecm_current_frame < s->ecm_frames)
    {
        /* Search for the next frame, within the current partial page, which has
           not been tagged as transferred OK. */
        for (i = s->ecm_current_frame;  i < s->ecm_frames;  i++)
        {
            if (s->ecm_len[i] >= 0)
            {
                send_frame(s, s->ecm_data[i], s->ecm_len[i]);
                s->ecm_current_frame = i + 1;
                s->ecm_frames_this_burst++;
                return 0;
            }
        }
        s->ecm_current_frame = s->ecm_frames;
    }
    if (s->ecm_current_frame <= s->ecm_frames + 3)
    {
        /* We have sent all the FCD frames. Send some RCP frames. Three seems to be
           a popular number, to minimise the risk of a bit error stopping the receiving
           end from recognising the RCP. */
        s->ecm_current_frame++;
        /* The RCP frame is an odd man out, as its a simple 1 byte control
           frame, but is specified to not have the final bit set. It doesn't
           seem to have the DIS received bit set, either. */
        frame[0] = 0xFF;
        frame[1] = 0x03;
        frame[2] = T4_RCP;
        send_frame(s, frame, 3);
        /* In case we are just after a CTC/CTR exchange, which kicked us back to long training */
        s->short_train = TRUE;
        return 0;
    }
    return -1;
}
/*- End of function --------------------------------------------------------*/

static int send_first_ecm_frame(t30_state_t *s)
{
    s->ecm_current_frame = 0;
    s->ecm_frames_this_burst = 0;
    return send_next_ecm_frame(s);
}
/*- End of function --------------------------------------------------------*/

static void print_frame(t30_state_t *s, const char *io, const uint8_t *fr, int frlen)
{
    span_log(&s->logging,
             SPAN_LOG_FLOW,
             "%s %s with%s final frame tag\n",
             io,
             t30_frametype(fr[2]),
             (fr[1] & 0x10)  ?  ""  :  "out");
    span_log_buf(&s->logging, SPAN_LOG_FLOW, io, fr, frlen);
}
/*- End of function --------------------------------------------------------*/

static void send_frame(t30_state_t *s, const uint8_t *fr, int frlen)
{
    print_frame(s, "Tx: ", fr, frlen);

    if (s->send_hdlc_handler)
        s->send_hdlc_handler(s->send_hdlc_user_data, fr, frlen);
}
/*- End of function --------------------------------------------------------*/

static void send_simple_frame(t30_state_t *s, int type)
{
    uint8_t frame[3];

    /* The simple command/response frames are always final frames */
    frame[0] = 0xFF;
    frame[1] = 0x13;
    frame[2] = (uint8_t) (type | s->dis_received);
    send_frame(s, frame, 3);
}
/*- End of function --------------------------------------------------------*/

static void send_20digit_msg_frame(t30_state_t *s, int cmd, char *msg)
{
    size_t len;
    int p;
    uint8_t frame[23];

    len = strlen(msg);
    p = 0;
    frame[p++] = 0xFF;
    frame[p++] = 0x03;
    frame[p++] = (uint8_t) (cmd | s->dis_received);
    while (len > 0)
        frame[p++] = msg[--len];
    while (p < 23)
        frame[p++] = ' ';
    send_frame(s, frame, 23);
}
/*- End of function --------------------------------------------------------*/

static int send_ident_frame(t30_state_t *s, uint8_t cmd)
{
    /* Only send if there is an ident to send. */
    if (s->local_ident[0])
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Sending ident '%s'\n", s->local_ident);
        /* 'cmd' should be T30_TSI, T30_CIG or T30_CSI */
        send_20digit_msg_frame(s, cmd, s->local_ident);
        return TRUE;
    }
    return FALSE;
}
/*- End of function --------------------------------------------------------*/

static int send_pw_frame(t30_state_t *s)
{
    /* Only send if there is a password to send. */
    if (s->local_password[0])
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Sending password '%s'\n", s->local_password);
        send_20digit_msg_frame(s, T30_PWD, s->local_password);
        return TRUE;
    }
    return FALSE;
}
/*- End of function --------------------------------------------------------*/

static int send_sub_frame(t30_state_t *s)
{
    /* Only send if there is a sub-address to send. */
    if (s->local_sub_address[0])
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Sending sub address '%s'\n", s->local_sub_address);
        send_20digit_msg_frame(s, T30_SUB, s->local_sub_address);
        return TRUE;
    }
    return FALSE;
}
/*- End of function --------------------------------------------------------*/

static int send_nsf_frame(t30_state_t *s)
{
    int p;
    uint8_t frame[T30_MAX_LOCAL_NSF_LEN + 3];

    /* Only send if there is an NSF message to send. */
    if (s->local_nsf_len)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Sending user supplied NSF - %d octets\n", s->local_nsf_len);
        p = 0;
        frame[p++] = 0xFF;
        frame[p++] = 0x03;
        frame[p++] = (uint8_t) (T30_NSF | s->dis_received);
        for (  ;  p < s->local_nsf_len + 3;  p++)
            frame[p] = s->local_nsf[p - 3];
        send_frame(s, frame, s->local_nsf_len + 3);
        return TRUE;
    }
    return FALSE;
}
/*- End of function --------------------------------------------------------*/

static int send_pps_frame(t30_state_t *s)
{
    uint8_t frame[7];
    
    frame[0] = 0xFF;
    frame[1] = 0x13;
    frame[2] = (uint8_t) (T30_PPS | s->dis_received);
    frame[3] = (s->ecm_at_page_end)  ?  ((uint8_t) (s->next_tx_step | s->dis_received))  :  T30_NULL;
    frame[4] = (uint8_t) (s->ecm_page & 0xFF);
    frame[5] = (uint8_t) (s->ecm_block & 0xFF);
    frame[6] = (uint8_t) ((s->ecm_frames_this_burst == 0)  ?  0  :  (s->ecm_frames_this_burst - 1));
    span_log(&s->logging, SPAN_LOG_FLOW, "Sending PPS + %s\n", t30_frametype(frame[3]));
    send_frame(s, frame, 7);
    return frame[3] & 0xFE;
}
/*- End of function --------------------------------------------------------*/

static int set_dis_or_dtc(t30_state_t *s)
{
    /* Whether we use a DIS or a DTC is determined by whether we have received a DIS.
       We just need to edit the prebuilt message. */
    s->dis_dtc_frame[2] = (uint8_t) (T30_DIS | s->dis_received);
    /* If we have a file name to receive into, then we are receive capable */
    if (s->rx_file[0])
        set_dis_dtc_bit(s, 10);
    else
        clr_dis_dtc_bit(s, 10);
    /* If we have a file name to transmit, then we are ready to transmit (polling) */
    if (s->tx_file[0])
        set_dis_dtc_bit(s, 9);
    else
        clr_dis_dtc_bit(s, 9);
    //t30_decode_dis_dtc_dcs(s, s->dis_dtc_frame, s->dis_dtc_len);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int build_dis_or_dtc(t30_state_t *s)
{
    int i;

    /* Build a skeleton for the DIS and DTC messages. This will be edited for
       the dynamically changing capabilities (e.g. can receive) just before
       it is sent. It might also be edited if the application changes our
       capabilities (e.g. disabling fine mode). Right now we set up all the
       unchanging stuff about what we are capable of doing. */
    s->dis_dtc_frame[0] = 0xFF;
    s->dis_dtc_frame[1] = 0x13;
    s->dis_dtc_frame[2] = (uint8_t) (T30_DIS | s->dis_received);
    for (i = 3;  i < 19;  i++)
        s->dis_dtc_frame[i] = 0x00;

    /* Always say 256 octets per ECM frame preferred, as 64 is never used in the
       real world. */
    if ((s->iaf & T30_IAF_MODE_T37))
        set_dis_dtc_bit(s, 1);
    if ((s->iaf & T30_IAF_MODE_T38))
        set_dis_dtc_bit(s, 3);
    /* No 3G mobile  */
    /* No V.8 */
    /* 256 octets preferred - don't bother making this optional, as everything uses 256 */
    /* Ready to transmit a fax (polling) will be determined separately, and this message edited. */
    /* Ready to receive a fax will be determined separately, and this message edited. */
    /* With no modems set we are actually selecting V.27ter fallback at 2400bps */
    if ((s->supported_modems & T30_SUPPORT_V27TER))
        set_dis_dtc_bit(s, 12);
    if ((s->supported_modems & T30_SUPPORT_V29))
        set_dis_dtc_bit(s, 11);
    /* V.17 is only valid when combined with V.29 and V.27ter, so if we enable V.17 we force the others too. */
    if ((s->supported_modems & T30_SUPPORT_V17))
        s->dis_dtc_frame[4] |= (DISBIT6 | DISBIT4 | DISBIT3);
    if ((s->supported_resolutions & T30_SUPPORT_FINE_RESOLUTION))
        set_dis_dtc_bit(s, 15);
    if ((s->supported_compressions & T30_SUPPORT_T4_2D_COMPRESSION))
        set_dis_dtc_bit(s, 16);
    /* 215mm wide is always supported */
    if ((s->supported_image_sizes & T30_SUPPORT_303MM_WIDTH))
        set_dis_dtc_bit(s, 18);
    else if ((s->supported_image_sizes & T30_SUPPORT_255MM_WIDTH))
        set_dis_dtc_bit(s, 17);
    /* A4 is always supported. */
    if ((s->supported_image_sizes & T30_SUPPORT_UNLIMITED_LENGTH))
        set_dis_dtc_bit(s, 20);
    else if ((s->supported_image_sizes & T30_SUPPORT_B4_LENGTH))
        set_dis_dtc_bit(s, 19);
    /* No scan-line padding required. */
    set_dis_dtc_bits(s, 7, 21);
    if ((s->supported_compressions & T30_SUPPORT_NO_COMPRESSION))
        set_dis_dtc_bit(s, 26);
    if (s->ecm_allowed)
    {
        /* ECM allowed */
        set_dis_dtc_bit(s, 27);
        /* Only offer the option of fancy compression schemes, if we are
           also offering the ECM option needed to support them. */
        if ((s->supported_compressions & T30_SUPPORT_T6_COMPRESSION))
            set_dis_dtc_bit(s, 31);
        if ((s->supported_compressions & T30_SUPPORT_T43_COMPRESSION))
            set_dis_dtc_bit(s, 36);
        if ((s->supported_compressions & T30_SUPPORT_T85_COMPRESSION))
            set_dis_dtc_bit(s, 78);
        /* No T.85 optional. */
        if ((s->supported_compressions & T30_SUPPORT_T45_COMPRESSION))
            set_dis_dtc_bit(s, 116);
    }
    if (s->support_fnv)
        set_dis_dtc_bit(s, 33);
    if ((s->supported_polling_features & T30_SUPPORT_SEP))
        set_dis_dtc_bit(s, 34);
    if ((s->supported_polling_features & T30_SUPPORT_PSA))
        set_dis_dtc_bit(s, 35);
    /* No plane interleave */
    /* No G.726 */
    /* No extended voice coding */
    if ((s->supported_resolutions & T30_SUPPORT_SUPERFINE_RESOLUTION))
        set_dis_dtc_bit(s, 41);
    if ((s->supported_resolutions & T30_SUPPORT_300_300_RESOLUTION))
        set_dis_dtc_bit(s, 42);
    if ((s->supported_resolutions & (T30_SUPPORT_400_400_RESOLUTION | T30_SUPPORT_R16_RESOLUTION)))
        set_dis_dtc_bit(s, 43);
    /* Metric */ 
    set_dis_dtc_bit(s, 45);
    /* Superfine minimum scan line time pattern follows fine */
    /* No selective polling */
    /* No sub-addressing */
    /* No password */
    /* No data file (polling) */
    /* No BFT */
    /* No DTM */
    /* No EDI */
    /* No BTM */
    /* No mixed mode (polling) */
    /* No character mode */
    /* No mixed mode */
    /* No mode 26 */
    /* No digital network capable */
    /* No JPEG */
    /* No full colour */
    /* No 12bits/pel */
    /* No sub-sampling */
    if ((s->supported_image_sizes & T30_SUPPORT_US_LETTER_LENGTH))
        set_dis_dtc_bit(s, 76);
    if ((s->supported_image_sizes & T30_SUPPORT_US_LEGAL_LENGTH))
        set_dis_dtc_bit(s, 77);
    if ((s->supported_resolutions & T30_SUPPORT_600_600_RESOLUTION))
        set_dis_dtc_bit(s, 105);
    if ((s->supported_resolutions & T30_SUPPORT_1200_1200_RESOLUTION))
        set_dis_dtc_bit(s, 106);
    if ((s->supported_resolutions & T30_SUPPORT_300_600_RESOLUTION))
        set_dis_dtc_bit(s, 107);
    if ((s->supported_resolutions & T30_SUPPORT_400_800_RESOLUTION))
        set_dis_dtc_bit(s, 108);
    if ((s->supported_resolutions & T30_SUPPORT_600_1200_RESOLUTION))
        set_dis_dtc_bit(s, 109);
    if ((s->iaf & T30_IAF_MODE_FLOW_CONTROL))
        set_dis_dtc_bit(s, 121);
    if ((s->iaf & T30_IAF_MODE_CONTINUOUS_FLOW))
        set_dis_dtc_bit(s, 123);
    s->dis_dtc_len = 19;
    //t30_decode_dis_dtc_dcs(s, s->dis_dtc_frame, s->dis_dtc_len);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int prune_dis_dtc(t30_state_t *s)
{
    int i;

    /* Find the last octet that is really needed, set the extension bits, and trim the message length */
    for (i = 18;  i > 4;  i--)
    {
        /* Strip the top bit */
        s->dis_dtc_frame[i] &= (DISBIT1 | DISBIT2 | DISBIT3 | DISBIT4 | DISBIT5 | DISBIT6 | DISBIT7);
        /* Check if there is some real message content here */
        if (s->dis_dtc_frame[i])
            break;
    }
    s->dis_dtc_len = i + 1;
    /* Fill in any required extension bits */
    for (i--;  i > 4;  i--)
        s->dis_dtc_frame[i] |= DISBIT8;
    t30_decode_dis_dtc_dcs(s, s->dis_dtc_frame, s->dis_dtc_len);
    return s->dis_dtc_len;
}
/*- End of function --------------------------------------------------------*/

static int build_dcs(t30_state_t *s, const uint8_t *msg, int len)
{
    uint8_t dis_dtc_frame[T30_MAX_DIS_DTC_DCS_LEN];
    int i;
    int bad;
    
    if (len < 6)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Short DIS/DTC frame\n");
        s->current_status = T30_ERR_INCOMPATIBLE;
        return -1;
    }

    /* Make a local copy of the message, padded to the maximum possible length with zeros. This allows
       us to simply pick out the bits, without worrying about whether they were set from the remote side. */
    if (len > T30_MAX_DIS_DTC_DCS_LEN)
    {
        memcpy(dis_dtc_frame, msg, T30_MAX_DIS_DTC_DCS_LEN);
    }
    else
    {
        memcpy(dis_dtc_frame, msg, len);
        if (len < T30_MAX_DIS_DTC_DCS_LEN)
            memset(dis_dtc_frame + len, 0, T30_MAX_DIS_DTC_DCS_LEN - len);
    }
    
    /* Make a DCS frame based on local issues and a received DIS frame. Negotiate the result
       based on what both parties can do. */
    s->dcs_frame[0] = 0xFF;
    s->dcs_frame[1] = 0x13;
    s->dcs_frame[2] = (uint8_t) (T30_DCS | s->dis_received);
    for (i = 3;  i < 19;  i++)
        s->dcs_frame[i] = 0x00;
    /* Set to required modem rate */
    s->dcs_frame[4] |= fallback_sequence[s->current_fallback].dcs_code;

    /* Select the compression to use. */
    switch (s->line_encoding)
    {
    case T4_COMPRESSION_ITU_T6:
        set_dcs_bit(s, 31);
        set_dcs_bits(s, T30_MIN_SCAN_0MS, 21);
        break;
    case T4_COMPRESSION_ITU_T4_2D:
        set_dcs_bit(s, 16);
        set_dcs_bits(s, s->min_scan_time_code & 0x7, 21);
        break;
    case T4_COMPRESSION_ITU_T4_1D:
        set_dcs_bits(s, s->min_scan_time_code & 0x7, 21);
        break;
    default:
        set_dcs_bits(s, T30_MIN_SCAN_0MS, 21);
        break;
    }
    /* We have a file to send, so tell the far end to go into receive mode. */
    set_dcs_bit(s, 10);
    /* Set the Y resolution bits */
    bad = T30_ERR_OK;
    switch (s->y_resolution)
    {
    case T4_Y_RESOLUTION_1200:
        switch (s->x_resolution)
        {
        case T4_X_RESOLUTION_600:
            if (!(s->supported_resolutions & T30_SUPPORT_600_1200_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 109);
            break;
        case T4_X_RESOLUTION_1200:
            if (!(s->supported_resolutions & T30_SUPPORT_1200_1200_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 106);
            break;
        default:
            bad = T30_ERR_NORESSUPPORT;
            break;
        }
        break;
    case T4_Y_RESOLUTION_800:
        switch (s->x_resolution)
        {
        case T4_X_RESOLUTION_R16:
            if (!(s->supported_resolutions & T30_SUPPORT_400_800_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 108);
            break;
        default:
            bad = T30_ERR_NORESSUPPORT;
            break;
        }
        break;
    case T4_Y_RESOLUTION_600:
        switch (s->x_resolution)
        {
        case T4_X_RESOLUTION_300:
            if (!(s->supported_resolutions & T30_SUPPORT_300_600_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 107);
            break;
        case T4_X_RESOLUTION_600:
            if (!(s->supported_resolutions & T30_SUPPORT_600_600_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 105);
            break;
        default:
            bad = T30_ERR_NORESSUPPORT;
            break;
        }
        break;
    case T4_Y_RESOLUTION_SUPERFINE:
        if (!(s->supported_resolutions & T30_SUPPORT_SUPERFINE_RESOLUTION))
        {
            bad = T30_ERR_NORESSUPPORT;
        }
        else
        {
            switch (s->x_resolution)
            {
            case T4_X_RESOLUTION_R8:
                set_dcs_bit(s, 41);
                break;
            case T4_X_RESOLUTION_R16:
                set_dcs_bit(s, 43);
                break;
            default:
                bad = T30_ERR_NORESSUPPORT;
                break;
            }
        }
        break;
    case T4_Y_RESOLUTION_300:
        switch (s->x_resolution)
        {
        case T4_X_RESOLUTION_300:
            if (!(s->supported_resolutions & T30_SUPPORT_300_300_RESOLUTION))
                bad = T30_ERR_NORESSUPPORT;
            else
                set_dcs_bit(s, 42);
            break;
        default:
            bad = T30_ERR_NORESSUPPORT;
            break;
        }
        break;
    case T4_Y_RESOLUTION_FINE:
        if (!(s->supported_resolutions & T30_SUPPORT_FINE_RESOLUTION))
        {
            bad = T30_ERR_NORESSUPPORT;
        }
        else
        {
            switch (s->x_resolution)
            {
            case T4_X_RESOLUTION_R8:
                set_dcs_bit(s, 15);
                break;
            default:
                bad = T30_ERR_NORESSUPPORT;
                break;
            }
        }
        break;
    default:
    case T4_Y_RESOLUTION_STANDARD:
        switch (s->x_resolution)
        {
        case T4_X_RESOLUTION_R8:
            /* No bits to set for this */
            break;
        default:
            bad = T30_ERR_NORESSUPPORT;
            break;
        }
        break;
    }
    if (bad != T30_ERR_OK)
    {
        s->current_status = bad;
        span_log(&s->logging, SPAN_LOG_FLOW, "Image resolution (%d x %d) not acceptable\n", s->x_resolution, s->y_resolution);
        return -1;
    }
    /* Deal with the image width. The X resolution will fall in line with any valid width. */
    /* Low (R4) res widths are not supported in recent versions of T.30 */
    bad = T30_ERR_OK;
    switch (s->image_width)
    {
    case T4_WIDTH_R8_A4:
    case T4_WIDTH_300_A4:
    case T4_WIDTH_R16_A4:
    case T4_WIDTH_600_A4:
    case T4_WIDTH_1200_A4:
        /* No width related bits need to be set. */
        break;
    case T4_WIDTH_R8_B4:
    case T4_WIDTH_300_B4:
    case T4_WIDTH_R16_B4:
    case T4_WIDTH_600_B4:
    case T4_WIDTH_1200_B4:
        if ((s->dis_dtc_frame[5] & (DISBIT2 | DISBIT1)) < 1)
            bad = T30_ERR_NOSIZESUPPORT;
        else if (!(s->supported_image_sizes & T30_SUPPORT_255MM_WIDTH))
            bad = T30_ERR_BADTIFF;
        else
            set_dcs_bit(s, 17);
        break;
    case T4_WIDTH_R8_A3:
    case T4_WIDTH_300_A3:
    case T4_WIDTH_R16_A3:
    case T4_WIDTH_600_A3:
    case T4_WIDTH_1200_A3:
        if ((s->dis_dtc_frame[5] & (DISBIT2 | DISBIT1)) < 2)
            bad = T30_ERR_NOSIZESUPPORT;
        else if (!(s->supported_image_sizes & T30_SUPPORT_303MM_WIDTH))    
            bad = T30_ERR_BADTIFF;
        else
            set_dcs_bit(s, 18);
        break;
    default:
        /* T.30 does not support this width */
        bad = T30_ERR_BADTIFF;
        break;
    }
    if (bad != T30_ERR_OK)
    {
        s->current_status = bad;
        span_log(&s->logging, SPAN_LOG_FLOW, "Image width (%d pixels) not a valid FAX image width\n", s->image_width);
        return -1;
    }
    switch (s->image_width)
    {
    case T4_WIDTH_R8_A4:
    case T4_WIDTH_R8_B4:
    case T4_WIDTH_R8_A3:
        /* These are always OK */
        break;
    case T4_WIDTH_300_A4:
    case T4_WIDTH_300_B4:
    case T4_WIDTH_300_A3:
        if (!test_bit(dis_dtc_frame, 42)  &&  !test_bit(dis_dtc_frame, 107))
            bad = T30_ERR_NOSIZESUPPORT;
        break;
    case T4_WIDTH_R16_A4:
    case T4_WIDTH_R16_B4:
    case T4_WIDTH_R16_A3:
        if (!test_bit(dis_dtc_frame, 43))
            bad = T30_ERR_NOSIZESUPPORT;
        break;
    case T4_WIDTH_600_A4:
    case T4_WIDTH_600_B4:
    case T4_WIDTH_600_A3:
        if (!test_bit(dis_dtc_frame, 105)  &&  !test_bit(dis_dtc_frame, 109))
            bad = T30_ERR_NOSIZESUPPORT;
        break;
    case T4_WIDTH_1200_A4:
    case T4_WIDTH_1200_B4:
    case T4_WIDTH_1200_A3:
        if (!test_bit(dis_dtc_frame, 106))
            bad = T30_ERR_NOSIZESUPPORT;
        break;
    default:
        /* T.30 does not support this width */
        bad = T30_ERR_BADTIFF;
        break;
    }
    if (bad != T30_ERR_OK)
    {
        s->current_status = bad;
        span_log(&s->logging, SPAN_LOG_FLOW, "Image width (%d pixels) not a valid FAX image width\n", s->image_width);
        return -1;
    }
    /* Deal with the image length */
    /* If the other end supports unlimited length, then use that. Otherwise, if the other end supports
       B4 use that, as its longer than the default A4 length. */
    if (test_bit(dis_dtc_frame, 20))
        set_dcs_bit(s, 20);
    else if (test_bit(dis_dtc_frame, 19))
        set_dcs_bit(s, 19);

    if (s->error_correcting_mode)
        set_dcs_bit(s, 27);

    if ((s->iaf & T30_IAF_MODE_FLOW_CONTROL)  &&  test_bit(dis_dtc_frame, 121))
        set_dcs_bit(s, 121);
    if ((s->iaf & T30_IAF_MODE_CONTINUOUS_FLOW)  &&  test_bit(dis_dtc_frame, 123))
        set_dcs_bit(s, 123);
    s->dcs_len = 19;
    //t30_decode_dis_dtc_dcs(s, s->dcs_frame, s->dcs_len);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int prune_dcs(t30_state_t *s)
{
    int i;

    /* Find the last octet that is really needed, set the extension bits, and trim the message length */
    for (i = 18;  i > 4;  i--)
    {
        /* Strip the top bit */
        s->dcs_frame[i] &= (DISBIT1 | DISBIT2 | DISBIT3 | DISBIT4 | DISBIT5 | DISBIT6 | DISBIT7);
        /* Check if there is some real message content here */
        if (s->dcs_frame[i])
            break;
    }
    s->dcs_len = i + 1;
    /* Fill in any required extension bits */
    for (i--  ;  i > 4;  i--)
        s->dcs_frame[i] |= DISBIT8;
    t30_decode_dis_dtc_dcs(s, s->dcs_frame, s->dcs_len);
    return s->dcs_len;
}
/*- End of function --------------------------------------------------------*/

static int step_fallback_entry(t30_state_t *s)
{
    while (fallback_sequence[++s->current_fallback].which)
    {
        if ((fallback_sequence[s->current_fallback].which & s->current_permitted_modems))
            return s->current_fallback;
    }
    return -1;
}
/*- End of function --------------------------------------------------------*/

static int find_fallback_entry(int dcs_code)
{
    int i;

    /* The table is short, and not searched often, so a brain-dead linear scan seems OK */
    for (i = 0;  fallback_sequence[i].bit_rate;  i++)
    {
        if (fallback_sequence[i].dcs_code == dcs_code)
            break;
    }
    if (fallback_sequence[i].bit_rate == 0)
        return -1;
    return i;
}
/*- End of function --------------------------------------------------------*/

static void send_dcn(t30_state_t *s)
{
    queue_phase(s, T30_PHASE_D_TX);
    set_state(s, T30_STATE_C);
    send_simple_frame(s, T30_DCN);
}
/*- End of function --------------------------------------------------------*/

static void send_dis_or_dtc_sequence(t30_state_t *s)
{
    prune_dis_dtc(s);
    set_state(s, T30_STATE_R);
    if (send_nsf_frame(s))
    {
        s->step = 0;
        return;
    }
    if (send_ident_frame(s, T30_CSI))
    {
        s->step = 1;
        return;
    }
    set_dis_or_dtc(s);
    send_frame(s, s->dis_dtc_frame, s->dis_dtc_len);
    s->step = 2;
}
/*- End of function --------------------------------------------------------*/

static void send_dcs_sequence(t30_state_t *s)
{
    /* Schedule training after the messages */
    prune_dcs(s);
    set_state(s, T30_STATE_D);
    if (send_pw_frame(s))
    {
        s->step = 0;
        return;
    }
    if (send_sub_frame(s))
    {
        s->step = 1;
        return;
    }
    if (send_ident_frame(s, T30_TSI))
    {
        s->step = 2;
        return;
    }
    send_frame(s, s->dcs_frame, s->dcs_len);
    s->step = 3;
}
/*- End of function --------------------------------------------------------*/

static void disconnect(t30_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Disconnecting\n");
    /* Make sure any FAX in progress is tidied up. If the tidying up has
       already happened, repeating it here is harmless. */
    t4_rx_end(&(s->t4));
    t4_tx_end(&(s->t4));
    s->timer_t0_t1 = 0;
    s->timer_t2_t4 = 0;
    s->timer_t3 = 0;
    s->timer_t5 = 0;
    set_phase(s, T30_PHASE_E);
    set_state(s, T30_STATE_B);
}
/*- End of function --------------------------------------------------------*/

static int set_min_scan_time_code(t30_state_t *s, const uint8_t *msg, int len)
{
    /* Translation between the codes for the minimum scan times the other end needs,
       and the codes for what we say will be used. We need 0 minimum. */
    static const uint8_t translate_min_scan_time[3][8] =
    {
        {T30_MIN_SCAN_20MS, T30_MIN_SCAN_5MS, T30_MIN_SCAN_10MS, T30_MIN_SCAN_20MS, T30_MIN_SCAN_40MS, T30_MIN_SCAN_40MS, T30_MIN_SCAN_10MS, T30_MIN_SCAN_0MS}, /* normal */
        {T30_MIN_SCAN_20MS, T30_MIN_SCAN_5MS, T30_MIN_SCAN_10MS, T30_MIN_SCAN_10MS, T30_MIN_SCAN_40MS, T30_MIN_SCAN_20MS, T30_MIN_SCAN_5MS,  T30_MIN_SCAN_0MS}, /* fine */
        {T30_MIN_SCAN_10MS, T30_MIN_SCAN_5MS, T30_MIN_SCAN_5MS,  T30_MIN_SCAN_5MS,  T30_MIN_SCAN_20MS, T30_MIN_SCAN_10MS, T30_MIN_SCAN_5MS,  T30_MIN_SCAN_0MS}  /* superfine, when half fine time is selected */
    };
    int min_bits_field;

    /* Set the minimum scan time bits */
    if ((s->iaf & T30_IAF_MODE_NO_FILL_BITS)  ||  len <= 5)
        min_bits_field = T30_MIN_SCAN_0MS;
    else
        min_bits_field = (msg[5] >> 4) & 7;
    switch (s->y_resolution)
    {
    case T4_Y_RESOLUTION_SUPERFINE:
        if (len > 8  &&  (msg[8] & DISBIT1))
        {
            s->min_scan_time_code = translate_min_scan_time[((msg[8] & DISBIT6))  ?  2  :  1][min_bits_field];
            break;
        }
        s->current_status = T30_ERR_NORESSUPPORT;
        span_log(&s->logging, SPAN_LOG_FLOW, "Remote FAX does not support super-fine resolution.\n");
        return -1;
    case T4_Y_RESOLUTION_FINE:
        if (len > 4  &&  (msg[4] & DISBIT7))
        {
            s->min_scan_time_code = translate_min_scan_time[1][min_bits_field];
            break;
        }
        s->current_status = T30_ERR_NORESSUPPORT;
        span_log(&s->logging, SPAN_LOG_FLOW, "Remote FAX does not support fine resolution.\n");
        return -1;
    default:
    case T4_Y_RESOLUTION_STANDARD:
        s->min_scan_time_code = translate_min_scan_time[0][min_bits_field];
        break;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int start_sending_document(t30_state_t *s, const uint8_t *msg, int len)
{
    /* Translation between the codes for the minimum scan time we will use, and milliseconds. */
    static const int min_scan_times[8] =
    {
        20, 5, 10, 0, 40, 0, 0, 0
    };
    int min_row_bits;

    if (s->tx_file[0] == '\0')
    {
        /* There is nothing to send */
        span_log(&s->logging, SPAN_LOG_FLOW, "No document to send\n");
        return -1;
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "Start sending document\n");
    if (t4_tx_init(&(s->t4), s->tx_file, s->tx_start_page, s->tx_stop_page) == NULL)
    {
        span_log(&s->logging, SPAN_LOG_WARNING, "Cannot open source TIFF file '%s'\n", s->tx_file);
        s->current_status = T30_ERR_FILEERROR;
        return -1;
    }
    t4_tx_get_pages_in_file(&(s->t4));
    t4_tx_set_tx_encoding(&(s->t4), s->line_encoding);
    t4_tx_set_local_ident(&(s->t4), s->local_ident);
    t4_tx_set_header_info(&(s->t4), s->header_info);

    s->x_resolution = t4_tx_get_x_resolution(&(s->t4));
    s->y_resolution = t4_tx_get_y_resolution(&(s->t4));
    /* The minimum scan time to be used can't be evaluated until we know the Y resolution, and
       must be evaluated before the minimum scan row bits can be evaluated. */
    if (set_min_scan_time_code(s, msg, len))
    {
        t4_tx_end(&(s->t4));
        return -1;
    }
    min_row_bits = fallback_sequence[s->current_fallback].bit_rate*min_scan_times[s->min_scan_time_code]/1000;
    span_log(&s->logging, SPAN_LOG_FLOW, "Minimum bits per row will be %d\n", min_row_bits);
    t4_tx_set_min_row_bits(&(s->t4), (s->forced_min_non_ecm_row_bits >= 0)  ?  s->forced_min_non_ecm_row_bits  :  min_row_bits);

    if (t4_tx_start_page(&(s->t4)))
    {
        t4_tx_end(&(s->t4));
        return -1;
    }
    s->image_width = t4_tx_get_image_width(&(s->t4));
    s->ecm_page = 0;
    s->ecm_block = 0;
    if (s->error_correcting_mode)
    {
        if (get_partial_ecm_page(s) == 0)
            span_log(&s->logging, SPAN_LOG_WARNING, "No image data to send\n");
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int restart_sending_document(t30_state_t *s)
{
    t4_tx_restart_page(&(s->t4));
    s->retries = 0;
    s->ecm_block = 0;
    send_dcs_sequence(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int start_receiving_document(t30_state_t *s)
{
    if (s->rx_file[0] == '\0')
    {
        /* There is nothing to receive to */
        span_log(&s->logging, SPAN_LOG_FLOW, "No document to receive\n");
        return -1;
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "Start receiving document\n");
    queue_phase(s, T30_PHASE_B_TX);
    s->dis_received = FALSE;
    s->ecm_page = 0;
    s->ecm_block = 0;
    send_dis_or_dtc_sequence(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void unexpected_frame(t30_state_t *s, const uint8_t *msg, int len)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected %s received in state %d\n", t30_frametype(msg[2]), s->state);
    switch (s->state)
    {
    case T30_STATE_F_DOC_ECM:
    case T30_STATE_F_DOC_NON_ECM:
        s->current_status = T30_ERR_INVALCMDRX;
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void unexpected_non_final_frame(t30_state_t *s, const uint8_t *msg, int len)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected %s frame in state %d\n", t30_frametype(msg[2]), s->state);
    s->current_status = T30_ERR_UNEXPECTED;
}
/*- End of function --------------------------------------------------------*/

static void unexpected_final_frame(t30_state_t *s, const uint8_t *msg, int len)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected %s received in state %d\n", t30_frametype(msg[2]), s->state);
    s->current_status = T30_ERR_UNEXPECTED;
    send_dcn(s);
}
/*- End of function --------------------------------------------------------*/

static void unexpected_frame_length(t30_state_t *s, const uint8_t *msg, int len)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected %s frame length - %d\n", t30_frametype(msg[0]), len);
    s->current_status = T30_ERR_UNEXPECTED;
    send_dcn(s);
}
/*- End of function --------------------------------------------------------*/

static int process_rx_dis_dtc(t30_state_t *s, const uint8_t *msg, int len)
{
    uint8_t dis_dtc_frame[T30_MAX_DIS_DTC_DCS_LEN];
    
    t30_decode_dis_dtc_dcs(s, msg, len);

    if (len < 6)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Short DIS/DTC frame\n");
        return -1;
    }

    if (msg[2] == T30_DIS)
        s->dis_received = TRUE;
    /* Make a local copy of the message, padded to the maximum possible length with zeros. This allows
       us to simply pick out the bits, without worrying about whether they were set from the remote side. */
    if (len > T30_MAX_DIS_DTC_DCS_LEN)
    {
        memcpy(dis_dtc_frame, msg, T30_MAX_DIS_DTC_DCS_LEN);
    }
    else
    {
        memcpy(dis_dtc_frame, msg, len);
        if (len < T30_MAX_DIS_DTC_DCS_LEN)
            memset(dis_dtc_frame + len, 0, T30_MAX_DIS_DTC_DCS_LEN - len);
    }
    s->error_correcting_mode = (s->ecm_allowed  &&  (dis_dtc_frame[6] & DISBIT3) != 0);
    /* 256 octets per ECM frame */
    s->octets_per_ecm_frame = 256;
    /* Select the compression to use. */
    if (s->error_correcting_mode  &&  (s->supported_compressions & T30_SUPPORT_T6_COMPRESSION)  &&  (dis_dtc_frame[6] & DISBIT7))
    {
        s->line_encoding = T4_COMPRESSION_ITU_T6;
    }
    else if ((s->supported_compressions & T30_SUPPORT_T4_2D_COMPRESSION)  &&  (dis_dtc_frame[4] & DISBIT8))
    {
        s->line_encoding = T4_COMPRESSION_ITU_T4_2D;
    }
    else
    {
        s->line_encoding = T4_COMPRESSION_ITU_T4_1D;
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "Selected compression %d\n", s->line_encoding);
    switch (dis_dtc_frame[4] & (DISBIT6 | DISBIT5 | DISBIT4 | DISBIT3))
    {
    case (DISBIT6 | DISBIT4 | DISBIT3):
        if ((s->supported_modems & T30_SUPPORT_V17))
        {
            s->current_permitted_modems = T30_SUPPORT_V17 | T30_SUPPORT_V29 | T30_SUPPORT_V27TER;
            s->current_fallback = T30_V17_FALLBACK_START;
            break;
        }
        /* Fall through */
    case (DISBIT4 | DISBIT3):
        if ((s->supported_modems & T30_SUPPORT_V29))
        {
            s->current_permitted_modems = T30_SUPPORT_V29 | T30_SUPPORT_V27TER;
            s->current_fallback = T30_V29_FALLBACK_START;
            break;
        }
        /* Fall through */
    case DISBIT4:
        s->current_permitted_modems = T30_SUPPORT_V27TER;
        s->current_fallback = T30_V27TER_FALLBACK_START;
        break;
    case 0:
        s->current_permitted_modems = T30_SUPPORT_V27TER;
        s->current_fallback = T30_V27TER_FALLBACK_START + 1;
        break;
    case DISBIT3:
        if ((s->supported_modems & T30_SUPPORT_V29))
        {
            /* TODO: this doesn't allow for skipping the V.27ter modes */
            s->current_permitted_modems = T30_SUPPORT_V29;
            s->current_fallback = T30_V29_FALLBACK_START;
            break;
        }
        /* Fall through */
    default:
        span_log(&s->logging, SPAN_LOG_FLOW, "Remote does not support a compatible modem\n");
        /* We cannot talk to this machine! */
        s->current_status = T30_ERR_INCOMPATIBLE;
        return -1;
    }
    if (s->phase_b_handler)
        s->phase_b_handler(s, s->phase_d_user_data, msg[2]);
    queue_phase(s, T30_PHASE_B_TX);
    /* Try to send something */
    if (s->tx_file[0])
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Trying to send file '%s'\n", s->tx_file);
        if ((msg[4] & DISBIT2) == 0)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "%s far end cannot receive\n", t30_frametype(msg[2]));
            s->current_status = T30_ERR_RX_INCAPABLE;
            send_dcn(s);
        }
        if (start_sending_document(s, msg, len))
        {
            send_dcn(s);
            return -1;
        }
        if (build_dcs(s, msg, len))
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "The far end is incompatible\n", s->tx_file);
            send_dcn(s);
            return -1;
        }
        s->retries = 0;
        send_dcs_sequence(s);
        return 0;
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "%s nothing to send\n", t30_frametype(msg[2]));
    /* ... then try to receive something */
    if (s->rx_file[0])
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Trying to receive file '%s'\n", s->rx_file);
        if ((msg[4] & DISBIT1) == 0)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "%s far end cannot transmit\n", t30_frametype(msg[2]));
            s->current_status = T30_ERR_TX_INCAPABLE;
            send_dcn(s);
            return -1;
        }
        if (start_receiving_document(s))
        {
            send_dcn(s);
            return -1;
        }
        s->dis_received = TRUE;
        if (set_dis_or_dtc(s))
        {
            s->current_status = T30_ERR_INCOMPATIBLE;
            send_dcn(s);
            return -1;
        }
        s->retries = 0;
        send_dis_or_dtc_sequence(s);
        return 0;
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "%s nothing to receive\n", t30_frametype(msg[2]));
    /* There is nothing to do, or nothing we are able to do. */
    send_dcn(s);
    return -1;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_dcs(t30_state_t *s, const uint8_t *msg, int len)
{
    static const int widths[6][4] =
    {
        {  T4_WIDTH_R4_A4,   T4_WIDTH_R4_B4,   T4_WIDTH_R4_A3, -1}, /* R4 resolution - no longer used in recent versions of T.30 */
        {  T4_WIDTH_R8_A4,   T4_WIDTH_R8_B4,   T4_WIDTH_R8_A3, -1}, /* R8 resolution */
        { T4_WIDTH_300_A4,  T4_WIDTH_300_B4,  T4_WIDTH_300_A3, -1}, /* 300/inch resolution */
        { T4_WIDTH_R16_A4,  T4_WIDTH_R16_B4,  T4_WIDTH_R16_A3, -1}, /* R16 resolution */
        { T4_WIDTH_600_A4,  T4_WIDTH_600_B4,  T4_WIDTH_600_A3, -1}, /* 600/inch resolution */
        {T4_WIDTH_1200_A4, T4_WIDTH_1200_B4, T4_WIDTH_1200_A3, -1}  /* 1200/inch resolution */
    };
    uint8_t dcs_frame[T30_MAX_DIS_DTC_DCS_LEN];
    int i;

    t30_decode_dis_dtc_dcs(s, msg, len);

    /* Check DCS frame from remote */
    if (len < 6)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Short DCS frame\n");
        return -1;
    }

    /* Make an ASCII string format copy of the message, for logging in the
       received file. This string does not include the frame header octets. */
    sprintf(s->rx_dcs_string, "%02X", bit_reverse8(msg[3]));
    for (i = 4;  i < len;  i++)
        sprintf(s->rx_dcs_string + 3*i - 10, " %02X", bit_reverse8(msg[i]));
    /* Make a local copy of the message, padded to the maximum possible length with zeros. This allows
       us to simply pick out the bits, without worrying about whether they were set from the remote side. */
    if (len > T30_MAX_DIS_DTC_DCS_LEN)
    {
        memcpy(dcs_frame, msg, T30_MAX_DIS_DTC_DCS_LEN);
    }
    else
    {
        memcpy(dcs_frame, msg, len);
        if (len < T30_MAX_DIS_DTC_DCS_LEN)
            memset(dcs_frame + len, 0, T30_MAX_DIS_DTC_DCS_LEN - len);
    }

    s->octets_per_ecm_frame = test_bit(dcs_frame, 28)  ?  256  :  64;

    if (test_bit(dcs_frame, 106))
        s->x_resolution = T4_X_RESOLUTION_1200;
    else if (test_bit(dcs_frame, 105)  ||  test_bit(dcs_frame, 109))
        s->x_resolution = T4_X_RESOLUTION_600;
    else if (test_bit(dcs_frame, 43)  ||  test_bit(dcs_frame, 108))
        s->x_resolution = T4_X_RESOLUTION_R16;
    else if (test_bit(dcs_frame, 42)  ||  test_bit(dcs_frame, 107))
        s->x_resolution = T4_X_RESOLUTION_300;
    else
        s->x_resolution = T4_X_RESOLUTION_R8;

    if (test_bit(dcs_frame, 106)  ||  test_bit(dcs_frame, 109))
        s->y_resolution = T4_Y_RESOLUTION_1200;
    else if (test_bit(dcs_frame, 108))
        s->y_resolution = T4_Y_RESOLUTION_800;
    else if (test_bit(dcs_frame, 105)  ||  test_bit(dcs_frame, 107))
        s->y_resolution = T4_Y_RESOLUTION_600;
    else if (test_bit(dcs_frame, 41)  ||  test_bit(dcs_frame, 43))
        s->y_resolution = T4_Y_RESOLUTION_SUPERFINE;
    else if (test_bit(dcs_frame, 42))
        s->y_resolution = T4_Y_RESOLUTION_300;
    else if (test_bit(dcs_frame, 15))
        s->y_resolution = T4_Y_RESOLUTION_FINE;
    else
        s->y_resolution = T4_Y_RESOLUTION_STANDARD;

    if (s->x_resolution == T4_X_RESOLUTION_1200)
        i = 5;
    else if (s->x_resolution == T4_X_RESOLUTION_600)
        i = 4;
    else if (s->x_resolution == T4_X_RESOLUTION_R16)
        i = 3;
    else if (s->x_resolution == T4_X_RESOLUTION_300)
        i = 2;
    else if (s->x_resolution == T4_X_RESOLUTION_R4)
        i = 0;
    else
        i = 1;

    s->image_width = widths[i][dcs_frame[5] & (DISBIT2 | DISBIT1)];

    /* Check which compression we will use. */
    if (test_bit(dcs_frame, 31))
        s->line_encoding = T4_COMPRESSION_ITU_T6;
    else if (test_bit(dcs_frame, 16))
        s->line_encoding = T4_COMPRESSION_ITU_T4_2D;
    else
        s->line_encoding = T4_COMPRESSION_ITU_T4_1D;
    span_log(&s->logging, SPAN_LOG_FLOW, "Selected compression %d\n", s->line_encoding);
    if (!(dcs_frame[4] & DISBIT2))
        span_log(&s->logging, SPAN_LOG_FLOW, "Remote cannot receive\n");

    if ((s->current_fallback = find_fallback_entry(dcs_frame[4] & (DISBIT6 | DISBIT5 | DISBIT4 | DISBIT3))) < 0)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Remote asked for a modem standard we do not support\n");
        return -1;
    }
    s->error_correcting_mode = (test_bit(dcs_frame, 27) != 0);

    if (s->phase_b_handler)
        s->phase_b_handler(s, s->phase_d_user_data, T30_DCS);
    /* Start document reception */
    span_log(&s->logging,
             SPAN_LOG_FLOW, 
             "Get document at %dbps, modem %d\n",
             fallback_sequence[s->current_fallback].bit_rate,
             fallback_sequence[s->current_fallback].modem_type);
    if (s->rx_file[0] == '\0')
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "No document to receive\n");
        s->current_status = T30_ERR_FILEERROR;
        send_dcn(s);
        return -1;
    }
    if (!s->in_message  &&  t4_rx_init(&(s->t4), s->rx_file, s->output_encoding) == NULL)
    {
        span_log(&s->logging, SPAN_LOG_WARNING, "Cannot open target TIFF file '%s'\n", s->rx_file);
        s->current_status = T30_ERR_FILEERROR;
        send_dcn(s);
        return -1;
    }
    if (!(s->iaf & T30_IAF_MODE_NO_TCF))
    {
        set_state(s, T30_STATE_F_TCF);
        set_phase(s, T30_PHASE_C_NON_ECM_RX);
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int send_deferred_pps_response(t30_state_t *s)
{
    queue_phase(s, T30_PHASE_D_TX);
    if (s->ecm_first_bad_frame >= s->ecm_frames)
    {
        /* Everything was OK. We can accept the data and move on. */
        switch (s->last_pps_fcf2)
        {
        case T30_NULL:
            /* We can confirm this partial page. */
            t30_ecm_commit_partial_page(s);
            break;
        default:
            /* We can confirm the whole page. */
            s->next_rx_step = s->last_pps_fcf2;
            t30_ecm_commit_partial_page(s);
            t4_rx_end_page(&(s->t4));
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, s->last_pps_fcf2);
            rx_start_page(s);
            break;
        }
        set_state(s, T30_STATE_F_POST_RCP_MCF);
        send_simple_frame(s, T30_MCF);
    }
    else
    {
        /* We need to send the PPR frame we have created, to try to fill in the missing/bad data. */
        set_state(s, T30_STATE_F_POST_RCP_PPR);
        s->ecm_frame_map[0] = 0xFF;
        s->ecm_frame_map[1] = 0x13;
        s->ecm_frame_map[2] = (uint8_t) (T30_PPR | s->dis_received);
        send_frame(s, s->ecm_frame_map, 3 + 32);
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_pps(t30_state_t *s, const uint8_t *msg, int len)
{
    int page;
    int block;
    int frames;
    int i;
    int j;
    int frame_no;

    if (len < 7)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Bad PPS message length %d.\n", len);
        return -1;
    }
    s->last_pps_fcf2 = msg[3] & 0xFE;
    page = msg[4];
    block = msg[5];
    /* The frames count is not well specified in T.30. In practice it seems it might be the
       number of frames in the current block, or it might be the number of frames in the
       current burst of transmission. For a burst of resent frames this would make it smaller
       than the actual size of the block. If we only accept the number when it exceeds
       previous values, we should get the real number of frames in the block. */
    frames = msg[6] + 1;
    if (s->ecm_frames < 0)
    {
        /* First time. Take the number and believe in it. */
        s->ecm_frames = frames;
    }
    else
    {
        /* If things have gone wrong, the far end might try to send us zero FCD
           frames. It can't represent zero in the block count field, so it might
           put zero there, or it might simplistically insert (blocks - 1), and put
           0xFF there. Beware of this. */
        if (frames == 0xFF)
        {
            /* This is probably zero, erroneously rolled over to the maximum count */
            frames = 0;
        }
    }
    span_log(&s->logging, SPAN_LOG_FLOW, "Received PPS + %s\n", t30_frametype(msg[3]));
    /* Build a bit map of which frames we now have stored OK */
    frame_no = 0;
    s->ecm_first_bad_frame = 256;
    for (i = 3;  i < 3 + 32;  i++)
    {
        s->ecm_frame_map[i] = 0;
        for (j = 0;  j < 8;  j++)
        {
            if (s->ecm_len[frame_no] < 0)
            {   
                s->ecm_frame_map[i] |= (1 << j);
                if (frame_no < s->ecm_first_bad_frame)
                    s->ecm_first_bad_frame = frame_no;
            }
            frame_no++;
        }
    }
    /* Are there any bad frames, or does our scan represent things being OK? */
    switch (s->last_pps_fcf2)
    {
    case T30_NULL:
    case T30_EOP:
    case T30_EOM:
    case T30_MPS:
    case T30_PRI_EOP:
    case T30_PRI_EOM:
    case T30_PRI_MPS:
        if (s->receiver_not_ready_count > 0)
        {
            queue_phase(s, T30_PHASE_D_TX);
            s->receiver_not_ready_count--;
            set_state(s, T30_STATE_F_POST_RCP_RNR);
            send_simple_frame(s, T30_RNR);
        }
        else
        {
            send_deferred_pps_response(s);
        }
        break;
    default:
        unexpected_final_frame(s, msg, len);
        break;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void process_rx_ppr(t30_state_t *s, const uint8_t *msg, int len)
{
    int i;
    int j;
    int frame_no;
    int mask;
    uint8_t frame[4];

    if (++s->ppr_count >= 4)
    {
        /* Continue to correct? */
        /* TODO: Decide if we should continue */
        /* Continue only if we have been making progress */
        if (1)
        {
            set_state(s, T30_STATE_IV_CTC);
            send_simple_frame(s, T30_CTC);
        }
        else
        {
            set_state(s, T30_STATE_IV_EOR);
            frame[0] = 0xFF;
            frame[1] = 0x13;
            frame[2] = (uint8_t) (T30_EOR | s->dis_received);
            frame[3] = (s->ecm_at_page_end)  ?  ((uint8_t) (s->next_tx_step | s->dis_received))  :  T30_NULL;
            span_log(&s->logging, SPAN_LOG_FLOW, "Sending EOR + %s\n", t30_frametype(frame[3]));
            send_frame(s, frame, 4);
        }
    }
    else
    {
        if (len != 3 + 32)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Bad length for PPR bits - %d\n", len);
            /* TODO: probably should send DCN */
            return;
        }
        /* Check which frames are OK, and mark them as OK. */
        for (i = 0;  i < 32;  i++)
        {
            if (msg[i + 3] == 0)
            {
                /* A chunk of 8 frames is OK */
                s->ecm_frame_map[i + 3] = 0;
                for (j = 0;  j < 8;  j++)
                    s->ecm_len[(i << 3) + j] = -1;
            }
            else
            {
                /* We need to sift through a chunk of 8 frames to find the good and bad */
                mask = 1;
                for (j = 0;  j < 8;  j++)
                {
                    frame_no = (i << 3) + j;
                    /* Tick off the frames they are not complaining about as OK */
                    if ((msg[i + 3] & mask) == 0)
                    {
                        s->ecm_len[frame_no] = -1;
                    }
                    else
                    {
                        if (frame_no < s->ecm_frames)
                            span_log(&s->logging, SPAN_LOG_FLOW, "Frame %d to be resent\n", frame_no);
#if 0
                        /* Diagnostic: See if the other end is complaining about something we didn't even send this time. */
                        if (s->ecm_len[frame_no] < 0)
                            span_log(&s->logging, SPAN_LOG_FLOW, "PPR contains complaint about frame %d, which was not sent\n", frame_no);
#endif
                    }
                    mask <<= 1;
                }
            }
        }
        /* Initiate resending of the remainder of the frames. */
        set_state(s, T30_STATE_IV);
        queue_phase(s, T30_PHASE_C_ECM_TX);
        send_first_ecm_frame(s);
    }
}
/*- End of function --------------------------------------------------------*/

static void process_rx_fcd(t30_state_t *s, const uint8_t *msg, int len)
{
    int frame_no;

    /* Facsimile coded data */
    switch (s->state)
    {
    case T30_STATE_F_DOC_ECM:
        if (len <= 4 + 256)
        {
            frame_no = msg[3];
            /* Just store the actual image data, and record its length */
            span_log(&s->logging, SPAN_LOG_FLOW, "Storing ECM frame %d, length %d\n", frame_no, len - 4);
            memcpy(&s->ecm_data[frame_no][0], &msg[4], len - 4);
            s->ecm_len[frame_no] = (int16_t) (len - 4);
            /* In case we are just after a CTC/CTR exchange, which kicked us back to long training */
            s->short_train = TRUE;
        }
        else
        {
            unexpected_frame_length(s, msg, len);
        }
        break;
    default:
        unexpected_non_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_rx_rcp(t30_state_t *s, const uint8_t *msg, int len)
{
    /* Return to control for partial page. These might come through with or without the final frame tag,
       so we have this routine to deal with the "no final frame tag" case. */
    switch (s->state)
    {
    case T30_STATE_F_DOC_ECM:
        set_state(s, T30_STATE_F_POST_DOC_ECM);
        queue_phase(s, T30_PHASE_D_RX);
        break;
    case T30_STATE_F_POST_DOC_ECM:
        /* Just ignore this */
        break;
    default:
        unexpected_non_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_rx_fnv(t30_state_t *s, const uint8_t *msg, int len)
{
    logging_state_t *log;
    const char *x;

    /* Field not valid */
    /* TODO: analyse the message, as per 5.3.6.2.13 */
    if (!span_log_test(&s->logging, SPAN_LOG_FLOW))
        return;
    log = &s->logging;

    if ((msg[3] & 0x01))
        span_log(log, SPAN_LOG_FLOW, "  Incorrect password (PWD).\n");
    if ((msg[3] & 0x02))
        span_log(log, SPAN_LOG_FLOW, "  Selective polling reference (SEP) not known.\n");
    if ((msg[3] & 0x04))
        span_log(log, SPAN_LOG_FLOW, "  Subaddress (SUB) not known.\n");
    if ((msg[3] & 0x08))
        span_log(log, SPAN_LOG_FLOW, "  Sender identity (SID) not known.\n");
    if ((msg[3] & 0x10))
        span_log(log, SPAN_LOG_FLOW, "  Secure fax error.\n");
    if ((msg[3] & 0x20))
        span_log(log, SPAN_LOG_FLOW, "  Transmitting subscriber identity (TSI) not accepted.\n");
    if ((msg[3] & 0x40))
        span_log(log, SPAN_LOG_FLOW, "  Polled subaddress (PSA) not known.\n");
    if (len > 4  &&  (msg[3] & DISBIT8))
    {
        if ((msg[4] & 0x01))
            span_log(log, SPAN_LOG_FLOW, "  BFT negotiations request not accepted.\n");
        if ((msg[4] & 0x02))
            span_log(log, SPAN_LOG_FLOW, "  Internet routing address (IRA) not known.\n");
        if ((msg[4] & 0x04))
            span_log(log, SPAN_LOG_FLOW, "  Internet selective polling address (ISP) not known.\n");
    }
    if (len > 5)
    {
        span_log(log, SPAN_LOG_FLOW, "  FNV sequence number %d.\n", msg[5]);
    }
    if (len > 6)
    {
        switch (msg[6])
        {
        case 0x83:
            x = "Incorrect password (PWD)";
            break;
        case 0x85:
            x = "Selective polling reference (SEP) not known";
            break;
        case 0x43:
        case 0xC3:
            x = "Subaddress (SUB) not known";
            break;
        case 0x45:
        case 0xC5:
            x = "Sender identity (SID) not known";
            break;
        case 0x10:
            x = "Secure fax error";
            break;
        case 0x42:
        case 0xC2:
            x = "Transmitting subscriber identity (TSI) not accepted";
            break;
        case 0x86:
            x = "Polled subaddress (PSA) not known";
            break;
        default:
            x = "???";
            break;
        }
        span_log(log, SPAN_LOG_FLOW, "  FNV diagnostic info type %s.\n", x);
    }
    if (len > 7)
    {
        span_log(log, SPAN_LOG_FLOW, "  FNV length %d.\n", msg[7]);
    }
    /* We've decoded it, but we don't yet know how to deal with it, so treat it as unexpected */
    unexpected_final_frame(s, msg, len);
}
/*- End of function --------------------------------------------------------*/

static void process_state_answering(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_DIS:
        /* TODO: This is a fudge to allow for starting up in T.38, where the other end has
           seen DIS by analogue modem means, and has immediately sent DIS/DTC. We might have
           missed useful info, like TSI, but just accept things and carry on form now. */
        span_log(&s->logging, SPAN_LOG_FLOW, "DIS/DTC before DIS\n");
        process_rx_dis_dtc(s, msg, len);
        break;
    case T30_DCS:
        /* TODO: This is a fudge to allow for starting up in T.38, where the other end has
           seen DIS by analogue modem means, and has immediately sent DCS. We might have
           missed useful info, like TSI, but just accept things and carry on form now. */
        span_log(&s->logging, SPAN_LOG_FLOW, "DCS before DIS\n");
        /* (TSI) DCS */
        /* (PWD) (SUB) (TSI) DCS */
        process_rx_dcs(s, msg, len);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_GOTDCNTX;
        disconnect(s);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_b(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_DCN:
        /* Just ignore any DCN's which appear at this stage. */
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_c(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_DCN:
        /* Just ignore any DCN's which appear at this stage. */
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_d(t30_state_t *s, const uint8_t *msg, int len)
{
    /* We should be sending the DCS sequence right now */
    switch (msg[2] & 0xFE)
    {
    case T30_DCN:
        s->current_status = T30_ERR_BADDCSTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_d_tcf(t30_state_t *s, const uint8_t *msg, int len)
{
    /* We should be sending the TCF data right now */
    switch (msg[2] & 0xFE)
    {
    case T30_DCN:
        s->current_status = T30_ERR_BADDCSTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_d_post_tcf(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CFR:
        /* Trainability test succeeded. Send the document. */
        span_log(&s->logging, SPAN_LOG_FLOW, "Trainability test succeeded\n");
        s->retries = 0;
        s->short_train = TRUE;
        if (s->error_correcting_mode)
        {
            set_state(s, T30_STATE_IV);
            queue_phase(s, T30_PHASE_C_ECM_TX);
            send_first_ecm_frame(s);
        }
        else
        {
            set_state(s, T30_STATE_I);
            queue_phase(s, T30_PHASE_C_NON_ECM_TX);
        }
        break;
    case T30_FTT:
        /* Trainability test failed. Try again. */
        span_log(&s->logging, SPAN_LOG_FLOW, "Trainability test failed\n");
        s->retries = 0;
        s->short_train = FALSE;
        if (step_fallback_entry(s) < 0)
        {
            /* We have fallen back as far as we can go. Give up. */
            s->current_fallback = 0;
            s->current_status = T30_ERR_CANNOT_TRAIN;
            send_dcn(s);
            break;
        }
        queue_phase(s, T30_PHASE_B_TX);
        send_dcs_sequence(s);
        break;
    case T30_DIS:
        /* It appears they didn't see what we sent - retry the TCF */
        if (++s->retries > MAX_MESSAGE_TRIES)
        {
            s->current_status = T30_ERR_RETRYDCN;
            send_dcn(s);
            break;
        }
        queue_phase(s, T30_PHASE_B_TX);
        /* TODO: should be reassess the new DIS message, and possibly adjust the DCS we use? */
        send_dcs_sequence(s);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_BADDCSTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_tcf(t30_state_t *s, const uint8_t *msg, int len)
{
    /* We should be receiving TCF right now, not HDLC messages */
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_cfr(t30_state_t *s, const uint8_t *msg, int len)
{
    /* We're waiting for a response to the CFR we sent */
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_ftt(t30_state_t *s, const uint8_t *msg, int len)
{
    /* We're waiting for a response to the FTT we sent */
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_doc_non_ecm(t30_state_t *s, const uint8_t *msg, int len)
{
    /* If we are getting HDLC messages, and we have not moved to the _POST_DOC_NON_ECM
       state, it looks like we didn't see the image data carrier properly. */
    switch (msg[2] & 0xFE)
    {
    case T30_DIS:
        process_rx_dis_dtc(s, msg, len);
        break;
    case T30_DCS:
        /* (TSI) DCS */
        /* (PWD) (SUB) (TSI) DCS */
        process_rx_dcs(s, msg, len);
        break;
    case T30_MPS:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, T30_MPS);
        s->next_rx_step = T30_MPS;
        queue_phase(s, T30_PHASE_D_TX);
        set_state(s, T30_STATE_III_Q_RTN);
        send_simple_frame(s, T30_RTN);
        break;
    case T30_PRI_MPS:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_MPS);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_MPS;
        set_state(s, T30_STATE_III_Q_RTN);
        break;
    case T30_EOM:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, T30_EOM);
        s->next_rx_step = T30_EOM;
        /* Return to phase B */
        queue_phase(s, T30_PHASE_B_TX);
        set_state(s, T30_STATE_III_Q_RTN);
        send_simple_frame(s, T30_RTN);
        break;
    case T30_PRI_EOM:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_EOM);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_EOM;
        set_state(s, T30_STATE_III_Q_RTN);
        break;
    case T30_EOP:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, T30_EOP);
        s->next_rx_step = T30_EOP;
        queue_phase(s, T30_PHASE_D_TX);
        set_state(s, T30_STATE_III_Q_RTN);
        send_simple_frame(s, T30_RTN);
        break;
    case T30_PRI_EOP:
        /* Treat this as a bad quality page. */
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_EOP);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_EOP;
        set_state(s, T30_STATE_III_Q_RTN);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNDATARX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_post_doc_non_ecm(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_MPS:
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, msg[2] & 0xFE);
        s->next_rx_step = T30_MPS;
        queue_phase(s, T30_PHASE_D_TX);
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_MCF);
            send_simple_frame(s, T30_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_RTP);
            send_simple_frame(s, T30_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_RTN);
            send_simple_frame(s, T30_RTN);
            break;
        }
        break;
    case T30_PRI_MPS:
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_MPS);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_MPS;
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            set_state(s, T30_STATE_III_Q_RTN);
            break;
        }
        break;
    case T30_EOM:
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, msg[2] & 0xFE);
        s->next_rx_step = T30_EOM;
        /* Return to phase B */
        queue_phase(s, T30_PHASE_B_TX);
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_MCF);
            send_simple_frame(s, T30_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_RTP);
            send_simple_frame(s, T30_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            rx_start_page(s);
            set_state(s, T30_STATE_III_Q_RTN);
            send_simple_frame(s, T30_RTN);
            break;
        }
        break;
    case T30_PRI_EOM:
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_EOM);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_EOM;
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            set_state(s, T30_STATE_III_Q_RTN);
            break;
        }
        break;
    case T30_EOP:
        if (s->phase_d_handler)
            s->phase_d_handler(s, s->phase_d_user_data, msg[2] & 0xFE);
        s->next_rx_step = T30_EOP;
        queue_phase(s, T30_PHASE_D_TX);
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_MCF);
            send_simple_frame(s, T30_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_RTP);
            send_simple_frame(s, T30_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            set_state(s, T30_STATE_III_Q_RTN);
            send_simple_frame(s, T30_RTN);
            break;
        }
        break;
    case T30_PRI_EOP:
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PRI_EOP);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        s->next_rx_step = T30_PRI_EOP;
        switch (copy_quality(s))
        {
        case T30_COPY_QUALITY_GOOD:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_MCF);
            break;
        case T30_COPY_QUALITY_POOR:
            t4_rx_end_page(&(s->t4));
            t4_rx_end(&(s->t4));
            s->in_message = FALSE;
            set_state(s, T30_STATE_III_Q_RTP);
            break;
        case T30_COPY_QUALITY_BAD:
            set_state(s, T30_STATE_III_Q_RTN);
            break;
        }
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNFAXRX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_doc_ecm(t30_state_t *s, const uint8_t *msg, int len)
{
    int fcf2;
    
    /* This actually handles 2 states - _DOC_ECM and _POST_DOC_ECM - as they are very similar */
    switch (msg[2] & 0xFE)
    {
    case T30_DIS:
        process_rx_dis_dtc(s, msg, len);
        break;
    case T30_DCS:
        /* (TSI) DCS */
        /* (PWD) (SUB) (TSI) DCS */
        process_rx_dcs(s, msg, len);
        break;
    case T4_RCP:
        if (s->state == T30_STATE_F_DOC_ECM)
        {
            /* Return to control for partial page */
            set_state(s, T30_STATE_F_POST_DOC_ECM);
            queue_phase(s, T30_PHASE_D_RX);
        }
        else
        {
            /* Ignore extra RCP frames. The source will usually send several to maximise the chance of
               one getting through OK. */
        }
        break;
    case T30_EOR:
        if (len != 4)
        {
            unexpected_frame_length(s, msg, len);
            break;
        }
        fcf2 = msg[3] & 0xFE;
        span_log(&s->logging, SPAN_LOG_FLOW, "Received EOR + %s\n", t30_frametype(msg[3]));
        switch (fcf2)
        {
        case T30_NULL:
            break;
        case T30_PRI_EOM:
        case T30_PRI_MPS:
        case T30_PRI_EOP:
            /* TODO: Alert operator */
            /* Fall through */
        case T30_EOM:
        case T30_MPS:
        case T30_EOP:
            s->next_rx_step = fcf2;
            send_simple_frame(s, T30_ERR);
            break;
        default:
            unexpected_final_frame(s, msg, len);
            break;
        }
        break;
    case T30_PPS:
        process_rx_pps(s, msg, len);
        break;
    case T30_CTC:
        send_simple_frame(s, T30_CTR);
        /* T.30 says we change back to long training here */
        s->short_train = FALSE;
        break;
    case T30_RR:
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNDATARX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_post_rcp_mcf(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_post_rcp_ppr(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_f_post_rcp_rnr(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_RR:
        if (s->receiver_not_ready_count > 0)
        {
            s->receiver_not_ready_count--;
            queue_phase(s, T30_PHASE_D_TX);
            set_state(s, T30_STATE_F_POST_RCP_RNR);
            send_simple_frame(s, T30_RNR);
        }
        else
        {
            send_deferred_pps_response(s);
        }
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_r(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_DIS:
        process_rx_dis_dtc(s, msg, len);
        break;
    case T30_DCS:
        /* (TSI) DCS */
        /* (PWD) (SUB) (TSI) DCS */
        process_rx_dcs(s, msg, len);
        break;
    case T30_DCN:
        /* Received a DCN while waiting for a DIS */
        s->current_status = T30_ERR_GOTDCNTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_t(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_DIS:
        process_rx_dis_dtc(s, msg, len);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNWHYRX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        s->current_status = T30_ERR_NODISTX;
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_i(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_ii(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_ii_q(t30_state_t *s, const uint8_t *msg, int len)
{
    t4_stats_t stats;

    switch (msg[2] & 0xFE)
    {
    case T30_MCF:
        switch (s->next_tx_step)
        {
        case T30_MPS:
        case T30_PRI_MPS:
            s->retries = 0;
            t4_tx_end_page(&(s->t4));
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
            if (t4_tx_start_page(&(s->t4)))
            {
                /* TODO: recover */
                break;
            }
            set_state(s, T30_STATE_I);
            queue_phase(s, T30_PHASE_C_NON_ECM_TX);
            break;
        case T30_EOM:
        case T30_PRI_EOM:
            s->retries = 0;
            t4_tx_end_page(&(s->t4));
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
            t4_tx_end(&(s->t4));
            set_state(s, T30_STATE_R);
            if (span_log_test(&s->logging, SPAN_LOG_FLOW))
            {
                t4_get_transfer_statistics(&(s->t4), &stats);
                span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
            }
            break;
        case T30_EOP:
        case T30_PRI_EOP:
            s->retries = 0;
            t4_tx_end_page(&(s->t4));
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
            t4_tx_end(&(s->t4));
            send_dcn(s);
            if (span_log_test(&s->logging, SPAN_LOG_FLOW))
            {
                t4_get_transfer_statistics(&(s->t4), &stats);
                span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
            }
            break;
        }
        break;
    case T30_RTP:
        switch (s->next_tx_step)
        {
        case T30_MPS:
        case T30_PRI_MPS:
            s->retries = 0;
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_RTP);
            /* Send fresh training, and then the next page */
            queue_phase(s, T30_PHASE_B_TX);
            restart_sending_document(s);
            break;
        case T30_EOM:
        case T30_PRI_EOM:
            s->retries = 0;
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_RTP);
            /* TODO: should go back to T, and resend */
            set_state(s, T30_STATE_R);
            break;
        case T30_EOP:
        case T30_PRI_EOP:
            s->retries = 0;
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_RTN);
            s->current_status = T30_ERR_INVALRSPTX;
            send_dcn(s);
            break;
        }
        break;
    case T30_RTN:
        switch (s->next_tx_step)
        {
        case T30_MPS:
        case T30_PRI_MPS:
            s->retries = 0;
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_RTN);
            /* Send fresh training, and then repeat the last page */
            queue_phase(s, T30_PHASE_B_TX);
            restart_sending_document(s);
            break;
        case T30_EOM:
        case T30_PRI_EOM:
        case T30_EOP:
        case T30_PRI_EOP:
            s->retries = 0;
            if (s->phase_d_handler)
                s->phase_d_handler(s, s->phase_d_user_data, T30_RTN);
            s->current_status = T30_ERR_INVALRSPTX;
            send_dcn(s);
            break;
        }
        break;
    case T30_PIP:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIP);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_PIN:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIN);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_DCN:
        switch (s->next_tx_step)
        {
        case T30_MPS:
        case T30_PRI_MPS:
        case T30_EOM:
        case T30_PRI_EOM:
            /* Unexpected DCN after EOM or MPS sequence */
            s->current_status = T30_ERR_DCNPHDRX;
            break;
        default:
            s->current_status = T30_ERR_BADPGTX;
            break;
        }
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iii_q_mcf(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_MPS:
    case T30_EOM:
    case T30_EOP:
        /* Looks like they didn't see our signal. Repeat it */
        send_simple_frame(s, T30_MCF);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iii_q_rtp(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_MPS:
    case T30_EOM:
    case T30_EOP:
        /* Looks like they didn't see our signal. Repeat it */
        send_simple_frame(s, T30_RTP);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iii_q_rtn(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_MPS:
    case T30_EOM:
    case T30_EOP:
        /* Looks like they didn't see our signal. Repeat it */
        send_simple_frame(s, T30_RTN);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNNORTNRX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_pps_null(t30_state_t *s, const uint8_t *msg, int len)
{
    t4_stats_t stats;

    switch (msg[2] & 0xFE)
    {
    case T30_MCF:
        s->retries = 0;
        s->timer_t5 = 0;
        /* Is there more of the current page to get, or do we move on? */
        span_log(&s->logging, SPAN_LOG_FLOW, "Is there more to send? - %d %d\n", s->ecm_frames, s->ecm_len[255]);
        if (!s->ecm_at_page_end  &&  get_partial_ecm_page(s) > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "Additional image data to send\n");
            s->ecm_block++;
            set_state(s, T30_STATE_IV);
            queue_phase(s, T30_PHASE_C_ECM_TX);
            send_first_ecm_frame(s);
        }
        else
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Moving on to the next page\n");
            switch (s->next_tx_step)
            {
            case T30_MPS:
            case T30_PRI_MPS:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                if (t4_tx_start_page(&(s->t4)))
                {
                    /* TODO: recover */
                    break;
                }
                s->ecm_page++;
                s->ecm_block = 0;
                if (get_partial_ecm_page(s) > 0)
                {
                    set_state(s, T30_STATE_IV);
                    queue_phase(s, T30_PHASE_C_ECM_TX);
                    send_first_ecm_frame(s);
                }
                break;
            case T30_EOM:
            case T30_PRI_EOM:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                set_state(s, T30_STATE_R);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            case T30_EOP:
            case T30_PRI_EOP:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                send_dcn(s);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            }
        }
        break;
    case T30_PPR:
        process_rx_ppr(s, msg, len);
        break;
    case T30_RNR:
        if (s->timer_t5 == 0)
            s->timer_t5 = ms_to_samples(DEFAULT_TIMER_T5);
        set_state(s, T30_STATE_IV_PPS_RNR);
        queue_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RR);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_BADPGTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        s->current_status = T30_ERR_ECMPHDTX;
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_pps_q(t30_state_t *s, const uint8_t *msg, int len)
{
    t4_stats_t stats;

    switch (msg[2] & 0xFE)
    {
    case T30_MCF:
        s->retries = 0;
        s->timer_t5 = 0;
        /* Is there more of the current page to get, or do we move on? */
        span_log(&s->logging, SPAN_LOG_FLOW, "Is there more to send? - %d %d\n", s->ecm_frames, s->ecm_len[255]);
        if (!s->ecm_at_page_end  &&  get_partial_ecm_page(s) > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "Additional image data to send\n");
            s->ecm_block++;
            set_state(s, T30_STATE_IV);
            queue_phase(s, T30_PHASE_C_ECM_TX);
            send_first_ecm_frame(s);
        }
        else
        {

            span_log(&s->logging, SPAN_LOG_FLOW, "Moving on to the next page\n");
            switch (s->next_tx_step)
            {
            case T30_MPS:
            case T30_PRI_MPS:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                if (t4_tx_start_page(&(s->t4)))
                {
                    /* TODO: recover */
                    break;
                }
                s->ecm_page++;
                s->ecm_block = 0;
                if (get_partial_ecm_page(s) > 0)
                {
                    set_state(s, T30_STATE_IV);
                    queue_phase(s, T30_PHASE_C_ECM_TX);
                    send_first_ecm_frame(s);
                }
                break;
            case T30_EOM:
            case T30_PRI_EOM:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                set_state(s, T30_STATE_R);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            case T30_EOP:
            case T30_PRI_EOP:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                send_dcn(s);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            }
        }
        break;
    case T30_RNR:
        if (s->timer_t5 == 0)
            s->timer_t5 = ms_to_samples(DEFAULT_TIMER_T5);
        set_state(s, T30_STATE_IV_PPS_RNR);
        queue_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RR);
        break;
    case T30_PIP:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIP);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_PIN:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIN);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_PPR:
        process_rx_ppr(s, msg, len);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_BADPGTX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        s->current_status = T30_ERR_ECMPHDTX;
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_pps_rnr(t30_state_t *s, const uint8_t *msg, int len)
{
    t4_stats_t stats;

    switch (msg[2] & 0xFE)
    {
    case T30_MCF:
        s->retries = 0;
        s->timer_t5 = 0;
        /* Is there more of the current page to get, or do we move on? */
        span_log(&s->logging, SPAN_LOG_FLOW, "Is there more to send? - %d %d\n", s->ecm_frames, s->ecm_len[255]);
        if (!s->ecm_at_page_end  &&  get_partial_ecm_page(s) > 0)
        {
            span_log(&s->logging, SPAN_LOG_WARNING, "Additional image data to send\n");
            s->ecm_block++;
            set_state(s, T30_STATE_IV);
            queue_phase(s, T30_PHASE_C_ECM_TX);
            send_first_ecm_frame(s);
        }
        else
        {

            span_log(&s->logging, SPAN_LOG_FLOW, "Moving on to the next page\n");
            switch (s->next_tx_step)
            {
            case T30_MPS:
            case T30_PRI_MPS:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                if (t4_tx_start_page(&(s->t4)))
                {
                    /* TODO: recover */
                    break;
                }
                s->ecm_page++;
                s->ecm_block = 0;
                if (get_partial_ecm_page(s) > 0)
                {
                    set_state(s, T30_STATE_IV);
                    queue_phase(s, T30_PHASE_C_ECM_TX);
                    send_first_ecm_frame(s);
                }
                break;
            case T30_EOM:
            case T30_PRI_EOM:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                set_state(s, T30_STATE_R);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            case T30_EOP:
            case T30_PRI_EOP:
                s->retries = 0;
                t4_tx_end_page(&(s->t4));
                if (s->phase_d_handler)
                    s->phase_d_handler(s, s->phase_d_user_data, T30_MCF);
                t4_tx_end(&(s->t4));
                send_dcn(s);
                if (span_log_test(&s->logging, SPAN_LOG_FLOW))
                {
                    t4_get_transfer_statistics(&(s->t4), &stats);
                    span_log(&s->logging, SPAN_LOG_FLOW, "Success - delivered %d pages\n", stats.pages_transferred);
                }
                break;
            }
        }
        break;
    case T30_RNR:
        if (s->timer_t5 == 0)
            s->timer_t5 = ms_to_samples(DEFAULT_TIMER_T5);
        set_state(s, T30_STATE_IV_PPS_RNR);
        queue_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RR);
        break;
    case T30_PIP:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIP);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_PIN:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIN);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNRRDRX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_ctc(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_CTR:
        /* Valid response to a CTC received */
        /* T.30 says we change back to long training here */
        s->short_train = FALSE;
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_eor(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_RNR:
        if (s->timer_t5 == 0)
            s->timer_t5 = ms_to_samples(DEFAULT_TIMER_T5);
        set_state(s, T30_STATE_IV_EOR_RNR);
        queue_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RR);
        break;
    case T30_PIN:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIN);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_ERR:
        /* TODO: Continue with the next message if MPS or EOM? */
        s->timer_t5 = 0;
        send_dcn(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_iv_eor_rnr(t30_state_t *s, const uint8_t *msg, int len)
{
    switch (msg[2] & 0xFE)
    {
    case T30_RNR:
        if (s->timer_t5 == 0)
            s->timer_t5 = ms_to_samples(DEFAULT_TIMER_T5);
        set_state(s, T30_STATE_IV_EOR_RNR);
        queue_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RR);
        break;
    case T30_PIN:
        s->retries = 0;
        if (s->phase_d_handler)
        {
            s->phase_d_handler(s, s->phase_d_user_data, T30_PIN);
            s->timer_t3 = ms_to_samples(DEFAULT_TIMER_T3);
        }
        break;
    case T30_ERR:
        /* TODO: Continue with the next message if MPS or EOM? */
        s->timer_t5 = 0;
        send_dcn(s);
        break;
    case T30_DCN:
        s->current_status = T30_ERR_DCNRRDRX;
        disconnect(s);
        break;
    case T30_CRP:
        repeat_last_command(s);
        break;
    case T30_FNV:
        process_rx_fnv(s, msg, len);
        break;
    default:
        /* We don't know what to do with this. */
        unexpected_final_frame(s, msg, len);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void process_state_call_finished(t30_state_t *s, const uint8_t *msg, int len)
{
    /* Simply ignore anything which comes in when we have declared the call
       to have finished. */
}
/*- End of function --------------------------------------------------------*/

static void hdlc_accept_control_msg(t30_state_t *s, const uint8_t *msg, int len, int ok)
{
    char far_password[T30_MAX_IDENT_LEN];
    
    if ((msg[1] & 0x10) == 0)
    {
        /* This is not a final frame */
        /* It seems we should not restart the command or response timer when exchanging HDLC image
           data. If the modem looses sync in the middle of the image, we should just wait until
           the carrier goes away before proceeding. */
        if (s->phase != T30_PHASE_C_ECM_RX)
        {
            /* Restart the command or response timer, T2 or T4 */
            s->timer_t2_t4 = ms_to_samples((s->timer_is_t4)  ?  DEFAULT_TIMER_T4  :  DEFAULT_TIMER_T2);
        }
        /* The following handles all the message types we expect to get without
           a final frame tag. If we get one that T.30 says we should not expect
           in a particular context, its pretty harmless, so don't worry. */
        switch (msg[2] & 0xFE)
        {
        case T30_CSI:
            if (msg[2] == T30_CSI)
            {
                /* Called subscriber identification */
                /* OK in (NSF) (CSI) DIS */
                decode_20digit_msg(s, s->far_ident, &msg[2], len - 2);
            }
            else
            {
                /* CIG - Calling subscriber identification */
                /* OK in (NSC) (CIG) DTC */
                /* OK in (PWD) (SEP) (CIG) DTC */
                decode_20digit_msg(s, s->far_ident, &msg[2], len - 2);
            }
            break;
        case T30_NSF:
            if (msg[2] == T30_NSF)
            {
                /* Non-standard facilities */
                /* OK in (NSF) (CSI) DIS */
                t35_decode(&msg[3], len - 3, &s->country, &s->vendor, &s->model);
                if (s->country)
                    span_log(&s->logging, SPAN_LOG_FLOW, "The remote was made in '%s'\n", s->country);
                if (s->vendor)
                    span_log(&s->logging, SPAN_LOG_FLOW, "The remote was made by '%s'\n", s->vendor);
                if (s->model)
                    span_log(&s->logging, SPAN_LOG_FLOW, "The remote is a '%s'\n", s->model);
            }
            else
            {
                /* NSC - Non-standard facilities command */
                /* OK in (NSC) (CIG) DTC */
            }
            break;
        case T30_PWD:
            if (msg[2] == T30_PWD)
            {
                /* Password */
                /* OK in (PWD) (SUB) (TSI) DCS */
                /* OK in (PWD) (SEP) (CIG) DTC */
                decode_20digit_msg(s, far_password, &msg[2], len - 2);
                if (strcmp(s->far_password, far_password) == 0)
                    s->far_password_ok = TRUE;  
            }
            else
            {
                unexpected_frame(s, msg, len);
            }
            break;
        case T30_SEP:
            if (msg[2] == T30_SEP)
            {
                /* Selective polling */
                /* OK in (PWD) (SEP) (CIG) DTC */
                decode_20digit_msg(s, s->sep_address, &msg[2], len - 2);
            }
            else
            {
                unexpected_frame(s, msg, len);
            }
            break;
        case T30_PSA:
            if (msg[2] == T30_PSA)
            {
                /* Polled subaddress */
                decode_20digit_msg(s, s->psa_address, &msg[2], len - 2);
            }
            else
            {
                unexpected_frame(s, msg, len);
            }
            break;
        case T30_CIA:
            if (msg[2] == T30_CIA)
            {
                /* Calling subscriber internet address */
                decode_url_msg(s, NULL, &msg[2], len - 2);
            }
            else
            {
                unexpected_frame(s, msg, len);
            }
            break;
        case T30_ISP:
            if (msg[2] == T30_ISP)
            {
                /* Internet selective polling address */
                decode_url_msg(s, NULL, &msg[2], len - 2);
            }
            else
            {
                unexpected_frame(s, msg, len);
            }
            break;
        case T30_TSI:
            /* Transmitting subscriber identity */
            /* OK in (TSI) DCS */
            /* OK in (PWD) (SUB) (TSI) DCS */
            decode_20digit_msg(s, s->far_ident, &msg[2], len - 2);
            break;
        case T30_SUB:
            /* Subaddress */
            /* OK in (PWD) (SUB) (TSI) DCS */
            decode_20digit_msg(s, s->far_sub_address, &msg[2], len - 2);
            break;
        case T30_SID:
            /* Sender Identification */
            /* T.30 does not say where this is OK */
            decode_20digit_msg(s, NULL, &msg[2], len - 2);
            break;
        case T30_CSA:
            /* Calling subscriber internet address */
            decode_url_msg(s, NULL, &msg[2], len - 2);
            break;
        case T30_TSA:
            /* Transmitting subscriber internet address */
            decode_url_msg(s, NULL, &msg[2], len - 2);
            break;
        case T30_IRA:
            /* Internet routing address */
            decode_url_msg(s, NULL, &msg[2], len - 2);
            break;
        case T4_FCD:
            process_rx_fcd(s, msg, len);
            break;
        case T4_RCP:
            process_rx_rcp(s, msg, len);
            break;
        default:
            unexpected_non_final_frame(s, msg, len);
            break;
        }
    }
    else
    {
        /* This is a final frame */
        /* Once we have any successful message from the far end, we
           cancel timer T1 */
        s->timer_t0_t1 = 0;

        /* The following handles context sensitive message types, which should
           occur at the end of message sequences. They should, therefore have
           the final frame flag set. */
        span_log(&s->logging, SPAN_LOG_FLOW, "In state %d\n", s->state);

        switch (s->state)
        {
        case T30_STATE_ANSWERING:
            process_state_answering(s, msg, len);
            break;
        case T30_STATE_B:
            process_state_b(s, msg, len);
            break;
        case T30_STATE_C:
            process_state_c(s, msg, len);
            break;
        case T30_STATE_D:
            process_state_d(s, msg, len);
            break;
        case T30_STATE_D_TCF:
            process_state_d_tcf(s, msg, len);
            break;
        case T30_STATE_D_POST_TCF:
            process_state_d_post_tcf(s, msg, len);
            break;
        case T30_STATE_F_TCF:
            process_state_f_tcf(s, msg, len);
            break;
        case T30_STATE_F_CFR:
            process_state_f_cfr(s, msg, len);
            break;
        case T30_STATE_F_FTT:
            process_state_f_ftt(s, msg, len);
            break;
        case T30_STATE_F_DOC_NON_ECM:
            process_state_f_doc_non_ecm(s, msg, len);
            break;
        case T30_STATE_F_POST_DOC_NON_ECM:
            process_state_f_post_doc_non_ecm(s, msg, len);
            break;
        case T30_STATE_F_DOC_ECM:
        case T30_STATE_F_POST_DOC_ECM:
            process_state_f_doc_ecm(s, msg, len);
            break;
        case T30_STATE_F_POST_RCP_MCF:
            process_state_f_post_rcp_mcf(s, msg, len);
            break;
        case T30_STATE_F_POST_RCP_PPR:
            process_state_f_post_rcp_ppr(s, msg, len);
            break;
        case T30_STATE_F_POST_RCP_RNR:
            process_state_f_post_rcp_rnr(s, msg, len);
            break;
        case T30_STATE_R:
            process_state_r(s, msg, len);
            break;
        case T30_STATE_T:
            process_state_t(s, msg, len);
            break;
        case T30_STATE_I:
            process_state_i(s, msg, len);
            break;
        case T30_STATE_II:
            process_state_ii(s, msg, len);
            break;
        case T30_STATE_II_Q:
            process_state_ii_q(s, msg, len);
            break;
        case T30_STATE_III_Q_MCF:
            process_state_iii_q_mcf(s, msg, len);
            break;
        case T30_STATE_III_Q_RTP:
            process_state_iii_q_rtp(s, msg, len);
            break;
        case T30_STATE_III_Q_RTN:
            process_state_iii_q_rtn(s, msg, len);
            break;
        case T30_STATE_IV:
            process_state_iv(s, msg, len);
            break;
        case T30_STATE_IV_PPS_NULL:
            process_state_iv_pps_null(s, msg, len);
            break;
        case T30_STATE_IV_PPS_Q:
            process_state_iv_pps_q(s, msg, len);
            break;
        case T30_STATE_IV_PPS_RNR:
            process_state_iv_pps_rnr(s, msg, len);
            break;
        case T30_STATE_IV_CTC:
            process_state_iv_ctc(s, msg, len);
            break;
        case T30_STATE_IV_EOR:
            process_state_iv_eor(s, msg, len);
            break;
        case T30_STATE_IV_EOR_RNR:
            process_state_iv_eor_rnr(s, msg, len);
            break;
        case T30_STATE_CALL_FINISHED:
            process_state_call_finished(s, msg, len);
            break;
        default:
            /* We don't know what to do with this. */
            unexpected_final_frame(s, msg, len);
            break;
        }
    }
}
/*- End of function --------------------------------------------------------*/

void t30_hdlc_accept(void *user_data, const uint8_t *msg, int len, int ok)
{
    t30_state_t *s;

    s = (t30_state_t *) user_data;
    if (len < 0)
    {
        /* Special conditions */
        switch (len)
        {
        case PUTBIT_TRAINING_FAILED:
            span_log(&s->logging, SPAN_LOG_FLOW, "HDLC carrier training failed in state %d\n", s->state);
            s->rx_trained = FALSE;
            /* Cancel the timer, since we have actually seen something, and wait until the carrier drops
               before proceeding. */
            // TODO: this is not a complete answer to handling failures to train
            s->timer_t2_t4 = 0;
            break;
        case PUTBIT_TRAINING_SUCCEEDED:
            /* The modem is now trained */
            span_log(&s->logging, SPAN_LOG_FLOW, "HDLC carrier trained in state %d\n", s->state);
            s->rx_signal_present = TRUE;
            s->rx_trained = TRUE;
            break;
        case PUTBIT_CARRIER_UP:
            span_log(&s->logging, SPAN_LOG_FLOW, "HDLC carrier up in state %d\n", s->state);
            s->rx_signal_present = TRUE;
            break;
        case PUTBIT_CARRIER_DOWN:
            span_log(&s->logging, SPAN_LOG_FLOW, "HDLC carrier down in state %d\n", s->state);
            s->rx_signal_present = FALSE;
            s->rx_trained = FALSE;
            /* If a phase change has been queued to occur after the receive signal drops,
               its time to change. */
            if (s->next_phase != T30_PHASE_IDLE)
            {
                set_phase(s, s->next_phase);
                s->next_phase = T30_PHASE_IDLE;
            }
            break;
        case PUTBIT_FRAMING_OK:
            span_log(&s->logging, SPAN_LOG_FLOW, "HDLC framing OK in state %d\n", s->state);
            if (!s->far_end_detected  &&  s->timer_t0_t1 > 0)
            {
                s->timer_t0_t1 = ms_to_samples(DEFAULT_TIMER_T1);
                s->far_end_detected = TRUE;
                if (s->phase == T30_PHASE_A_CED  ||  s->phase == T30_PHASE_A_CNG)
                    set_phase(s, T30_PHASE_B_RX);
            }
            /* 5.4.3.1 Timer T2 is reset if flag is received */
            if (!s->timer_is_t4  &&  s->timer_t2_t4 > 0)
                s->timer_t2_t4 = 0;
            break;
        case PUTBIT_ABORT:
            /* Just ignore these */
            break;
        default:
            span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected HDLC special length - %d!\n", len);
            break;
        }
        return;
    }

    /* The spec. says a command or response is not valid if:
        - any of the frames, optional or mandatory, have an FCS error.
        - any single frame exceeds 3s +- 15% (i.e. no frame should exceed 2.55s)
        - the final frame is not tagged as a final frame
        - the final frame is not a recognised one.
       The first point seems benign. If we accept an optional frame, and a later
       frame is bad, having accepted the optional frame should be harmless.
       The 2.55s maximum seems to limit signalling frames to no more than 95 octets,
       including FCS, and flag octets (assuming the use of V.21).
    */
    if (!ok)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Bad CRC received\n");
        if (s->crp_enabled)
            send_simple_frame(s, T30_CRP);
        return;
    }

    /* Cancel the command or response timer */
    s->timer_t2_t4 = 0;
    if (len < 3)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Bad HDLC frame length - %d\n", len);
        return;
    }
    if (msg[0] != 0xFF  ||  !(msg[1] == 0x03  ||  msg[1] == 0x13))
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Bad HDLC frame header - %02x %02x\n", msg[0], msg[1]);
        return;
    }
    print_frame(s, "Rx: ", msg, len);

    switch (s->phase)
    {
    case T30_PHASE_A_CED:
    case T30_PHASE_A_CNG:
    case T30_PHASE_B_RX:
    case T30_PHASE_C_ECM_RX:
    case T30_PHASE_D_RX:
        break;
    default:
        span_log(&s->logging, SPAN_LOG_FLOW, "Unexpected HDLC frame received in phase %s, state %d\n", phase_names[s->phase], s->state);
        break;
    }
    hdlc_accept_control_msg(s, msg, len, ok);
}
/*- End of function --------------------------------------------------------*/

static void queue_phase(t30_state_t *s, int phase)
{
    if (s->rx_signal_present)
    {
        /* We need to wait for that signal to go away */
        s->next_phase = phase;
    }
    else
    {
        set_phase(s, phase);
        s->next_phase = T30_PHASE_IDLE;
    }
}
/*- End of function --------------------------------------------------------*/

static void set_phase(t30_state_t *s, int phase)
{
    if (phase != s->phase)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Changing from phase %s to %s\n", phase_names[s->phase], phase_names[phase]);
        /* We may be killing a receiver before it has declared the end of the
           signal. Force the signal present indicator to off, because the
           receiver will never be able to. */
        if (s->phase != T30_PHASE_A_CED  &&  s->phase != T30_PHASE_A_CNG)
            s->rx_signal_present = FALSE;
        s->rx_trained = FALSE;
        s->phase = phase;
        switch (phase)
        {
        case T30_PHASE_A_CED:
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_V21, FALSE, TRUE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_CED, FALSE, FALSE);
            break;
        case T30_PHASE_A_CNG:
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_V21, FALSE, TRUE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_CNG, FALSE, FALSE);
            break;
        case T30_PHASE_B_RX:
        case T30_PHASE_D_RX:
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_V21, FALSE, TRUE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            break;
        case T30_PHASE_B_TX:
        case T30_PHASE_D_TX:
            if (!s->far_end_detected  &&  s->timer_t0_t1 > 0)
            {
                s->timer_t0_t1 = ms_to_samples(DEFAULT_TIMER_T1);
                s->far_end_detected = TRUE;
            }
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_V21, FALSE, TRUE);
            break;
        case T30_PHASE_C_NON_ECM_RX:
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, fallback_sequence[s->current_fallback].modem_type, s->short_train, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            break;
        case T30_PHASE_C_NON_ECM_TX:
            /* Pause before switching from anything to phase C */
            /* Always prime the training count for 1.5s of data at the current rate. Its harmless if
               we prime it and are not doing TCF. */
            s->training_test_bits = (3*fallback_sequence[s->current_fallback].bit_rate)/2;
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, fallback_sequence[s->current_fallback].modem_type, s->short_train, FALSE);
            break;
        case T30_PHASE_C_ECM_RX:
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, fallback_sequence[s->current_fallback].modem_type, s->short_train, TRUE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            break;
        case T30_PHASE_C_ECM_TX:
            /* Pause before switching from anything to phase C */
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, fallback_sequence[s->current_fallback].modem_type, s->short_train, TRUE);
            break;
        case T30_PHASE_E:
            /* Send a little silence before ending things, to ensure the
               buffers are all flushed through, and the far end has seen
               the last message we sent. */
            s->training_current_zeros = 0;
            s->training_most_zeros = 0;
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_NONE, FALSE, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_PAUSE, FINAL_FLUSH_TIME, FALSE);
            break;
        case T30_PHASE_CALL_FINISHED:
            if (s->set_rx_type_handler)
                s->set_rx_type_handler(s->set_rx_type_user_data, T30_MODEM_DONE, FALSE, FALSE);
            if (s->set_tx_type_handler)
                s->set_tx_type_handler(s->set_tx_type_user_data, T30_MODEM_DONE, FALSE, FALSE);
            break;
        }
    }
}
/*- End of function --------------------------------------------------------*/

static void set_state(t30_state_t *s, int state)
{
    if (s->state != state)
    {
        span_log(&s->logging, SPAN_LOG_FLOW, "Changing from state %d to %d\n", s->state, state);
        s->state = state;
    }
    s->step = 0;
}
/*- End of function --------------------------------------------------------*/

void t30_front_end_status(void *user_data, int status)
{
    t30_state_t *s;
    
    s = (t30_state_t *) user_data;

    switch (status)
    {
    case T30_FRONT_END_SEND_STEP_COMPLETE:
    case T30_FRONT_END_SEND_COMPLETE:
        span_log(&s->logging, SPAN_LOG_FLOW, "Send complete in phase %s, state %d\n", phase_names[s->phase], s->state);
        /* We have finished sending our messages, so move on to the next operation. */
        switch (s->state)
        {
        case T30_STATE_ANSWERING:
            span_log(&s->logging, SPAN_LOG_FLOW, "Starting answer mode\n");
            set_phase(s, T30_PHASE_B_TX);
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T2);
            s->timer_is_t4 = FALSE;
            s->dis_received = FALSE;
            send_dis_or_dtc_sequence(s);
            break;
        case T30_STATE_R:
            switch (s->step)
            {
            case 0:
                s->step++;
                if (send_ident_frame(s, T30_CSI))
                    break;
                /* Fall through */
            case 1:
                s->step++;
                set_dis_or_dtc(s);
                send_frame(s, s->dis_dtc_frame, s->dis_dtc_len);
                break;
            case 2:
                s->step++;
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                break;        
            default:
                /* Wait for an acknowledgement. */
                set_phase(s, T30_PHASE_B_RX);
                s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T4);
                s->timer_is_t4 = TRUE;
                break;
            }
            break;
        case T30_STATE_F_CFR:
            if (s->step == 0)
            {
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                s->step++;
            }
            else
            {
                if (s->error_correcting_mode)
                {
                    set_state(s, T30_STATE_F_DOC_ECM);
                    set_phase(s, T30_PHASE_C_ECM_RX);
                }
                else
                {
                    set_state(s, T30_STATE_F_DOC_NON_ECM);
                    set_phase(s, T30_PHASE_C_NON_ECM_RX);
                }
                s->next_rx_step = T30_MPS;
            }
            break;
        case T30_STATE_F_FTT:
            if (s->step == 0)
            {
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                s->step++;
            }
            else
            {
                set_phase(s, T30_PHASE_B_RX);
                s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T4);
                s->timer_is_t4 = TRUE;
            }
            break;
        case T30_STATE_III_Q_MCF:
        case T30_STATE_III_Q_RTP:
        case T30_STATE_III_Q_RTN:
        case T30_STATE_F_POST_RCP_PPR:
        case T30_STATE_F_POST_RCP_MCF:
            if (s->step == 0)
            {
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                s->step++;
            }
            else
            {
                switch (s->next_rx_step)
                {
                case T30_MPS:
                case T30_PRI_MPS:
                    if (s->error_correcting_mode)
                    {
                        set_state(s, T30_STATE_F_DOC_ECM);
                        set_phase(s, T30_PHASE_C_ECM_RX);
                    }
                    else
                    {
                        set_state(s, T30_STATE_F_DOC_NON_ECM);
                        set_phase(s, T30_PHASE_C_NON_ECM_RX);
                    }
                    break;
                case T30_EOM:
                case T30_PRI_EOM:
                    /* TODO: */
                    disconnect(s);
                    break;
                case T30_EOP:
                case T30_PRI_EOP:
                    disconnect(s);
                    break;
                default:
                    span_log(&s->logging, SPAN_LOG_FLOW, "Unknown next rx step - %d\n", s->next_rx_step);
                    disconnect(s);
                    break;
                }
            }
            break;
        case T30_STATE_II_Q:
        case T30_STATE_IV_PPS_NULL:
        case T30_STATE_IV_PPS_Q:
        case T30_STATE_IV_PPS_RNR:
        case T30_STATE_IV_EOR_RNR:
        case T30_STATE_F_POST_RCP_RNR:
            if (s->step == 0)
            {
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                s->step++;
            }
            else
            {
                /* We have finished sending the post image message. Wait for an
                   acknowledgement. */
                set_phase(s, T30_PHASE_D_RX);
                s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T4);
                s->timer_is_t4 = TRUE;
            }
            break;
        case T30_STATE_B:
            /* We have now allowed time for the last message to flush
               through the system, so it is safe to report the end of the
               call. */
            if (s->phase_e_handler)
                s->phase_e_handler(s, s->phase_e_user_data, s->current_status);
            set_state(s, T30_STATE_CALL_FINISHED);
            set_phase(s, T30_PHASE_CALL_FINISHED);
            break;
        case T30_STATE_C:
            if (s->step == 0)
            {
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                s->step++;
            }
            else
            {
                /* We just sent the disconnect message. Now it is time to disconnect */
                disconnect(s);
            }
            break;
        case T30_STATE_D:
            switch (s->step)
            {
            case 0:
                s->step++;
                if (send_sub_frame(s))
                    break;
                /* Fall through */
            case 1:
                s->step++;
                if (send_ident_frame(s, T30_TSI))
                    break;
                /* Fall through */
            case 2:
                s->step++;
                send_frame(s, s->dcs_frame, s->dcs_len);
                break;
            case 3:
                s->step++;
                if (s->send_hdlc_handler)
                    s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                break;        
            default:
                if ((s->iaf & T30_IAF_MODE_NO_TCF))
                {
                    /* Skip the trainability test */
                    s->retries = 0;
                    s->short_train = TRUE;
                    if (s->error_correcting_mode)
                    {
                        set_state(s, T30_STATE_IV);
                        queue_phase(s, T30_PHASE_C_ECM_TX);
                    }
                    else
                    {
                        set_state(s, T30_STATE_I);
                        queue_phase(s, T30_PHASE_C_NON_ECM_TX);
                    }
                }
                else
                {
                    /* Do the trainability test */
                    set_state(s, T30_STATE_D_TCF);
                    set_phase(s, T30_PHASE_C_NON_ECM_TX);
                }
                break;
            }
            break;
        case T30_STATE_D_TCF:
            /* Finished sending training test. Listen for the response. */
            set_phase(s, T30_PHASE_B_RX);
            s->timer_t2_t4 = ms_to_samples(DEFAULT_TIMER_T4);
            s->timer_is_t4 = TRUE;
            set_state(s, T30_STATE_D_POST_TCF);
            break;
        case T30_STATE_I:
            /* Send the end of page message */
            set_phase(s, T30_PHASE_D_TX);
            set_state(s, T30_STATE_II_Q);
            /* We might need to resend the page we are on, but we need to check if there
               are any more pages to send, so we can send the correct signal right now. */
            send_simple_frame(s, s->next_tx_step = check_next_tx_step(s));
            break;
        case T30_STATE_IV:
            /* We have finished sending an FCD frame */
            if (s->step == 0)
            {
                if (send_next_ecm_frame(s))
                {
                    if (s->send_hdlc_handler)
                        s->send_hdlc_handler(s->send_hdlc_user_data, NULL, 0);
                    s->step++;
                }
            }
            else
            {
                /* Send the end of page or partial page message */
                set_phase(s, T30_PHASE_D_TX);
                s->next_tx_step = check_next_tx_step(s);
                if (send_pps_frame(s) == T30_NULL)
                    set_state(s, T30_STATE_IV_PPS_NULL);
                else
                    set_state(s, T30_STATE_IV_PPS_Q);
            }
            break;
        case T30_STATE_CALL_FINISHED:
            /* Just ignore anything that happens now. We might get here if a premature
               disconnect from the far end overlaps something. */
            break;
        default:
            span_log(&s->logging, SPAN_LOG_FLOW, "Bad state in t30_front_end_status - %d\n", s->state);
            break;
        }
        break;
    case T30_FRONT_END_RECEIVE_COMPLETE:
        span_log(&s->logging, SPAN_LOG_FLOW, "Receive complete in phase %s, state %d\n", phase_names[s->phase], s->state);
        /* Usually receive complete is notified by a carrier down signal. However,
           in cases like a T.38 packet stream dying in the middle of reception
           there needs to be a means to stop things. */
        switch (s->phase)
        {
        case T30_PHASE_C_NON_ECM_RX:
            t30_non_ecm_put_bit(s, PUTBIT_CARRIER_DOWN);
            break;
        default:
            t30_hdlc_accept(s, NULL, PUTBIT_CARRIER_DOWN, TRUE);
            break;
        }
        break;
    case T30_FRONT_END_SIGNAL_PRESENT:
        span_log(&s->logging, SPAN_LOG_FLOW, "A signal is present\n");
        /* The front end is explicitly telling us the signal we expect is present. This might
           be a premature indication from a T.38 implementation, but we have to believe it.
           if we don't we can time out improperly. For example, we might get an image modem
           carrier signal, but the first HDLC frame might only occur several seconds later.
           Many ECM senders idle on HDLC flags while waiting for the paper or filing system
           to become ready. T.38 offers no specific indication of correct carrier training, so
           if we don't kill the timer on the initial carrier starting signal, we will surely
           time out quite often before the next thing we receive. */
        switch (s->phase)
        {
        case T30_PHASE_A_CED:
        case T30_PHASE_A_CNG:
        case T30_PHASE_B_RX:
        case T30_PHASE_D_RX:
            /* We are running a V.21 receive modem, where an explicit training indication
               will not occur. */
            t30_hdlc_accept(s, NULL, PUTBIT_CARRIER_UP, TRUE);
            t30_hdlc_accept(s, NULL, PUTBIT_FRAMING_OK, TRUE);
            break;
        default:
            /* Cancel any receive timeout, and declare that a receive signal is present,
               since the front end is explicitly telling us we have seen something. */
            s->rx_signal_present = TRUE;
            s->timer_t2_t4 = 0;
            break;
        }
        break;
    case T30_FRONT_END_SIGNAL_ABSENT:
        span_log(&s->logging, SPAN_LOG_FLOW, "No signal is present\n");
        /* TODO: Should we do anything here? */
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void repeat_last_command(t30_state_t *s)
{
    switch (s->state)
    {
    case T30_STATE_R:
        s->dis_received = FALSE;
        set_phase(s, T30_PHASE_B_TX);
        send_dis_or_dtc_sequence(s);
        break;
    case T30_STATE_III_Q_MCF:
        set_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_MCF);
        break;
    case T30_STATE_III_Q_RTP:
        set_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RTP);
        break;
    case T30_STATE_III_Q_RTN:
        set_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RTN);
        break;
    case T30_STATE_II_Q:
        set_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, s->next_tx_step);
        break;
    case T30_STATE_IV_PPS_NULL:
    case T30_STATE_IV_PPS_Q:
        set_phase(s, T30_PHASE_D_TX);
        send_pps_frame(s);
        break;
    case T30_STATE_IV_PPS_RNR:
    case T30_STATE_IV_EOR_RNR:
        set_phase(s, T30_PHASE_D_TX);
        send_simple_frame(s, T30_RNR);
        break;
    case T30_STATE_D:
        set_phase(s, T30_PHASE_B_TX);
        send_dcs_sequence(s);
        break;
    case T30_STATE_F_FTT:
        set_phase(s, T30_PHASE_B_TX);
        send_simple_frame(s, T30_FTT);
        break;
    case T30_STATE_F_CFR:
        set_phase(s, T30_PHASE_B_TX);
        send_simple_frame(s, T30_CFR);
        break;
    case T30_STATE_D_POST_TCF:
        /* Need to send the whole training thing again */
        s->short_train = FALSE;
        set_phase(s, T30_PHASE_B_TX);
        send_dcs_sequence(s);
        break;
    case T30_STATE_F_POST_RCP_RNR:
        /* Just ignore */
        break;
    default:
        span_log(&s->logging,
                 SPAN_LOG_FLOW,
                 "Repeat command called with nothing to repeat - phase %s, state %d\n",
                 phase_names[s->phase], s->state);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void timer_t0_expired(t30_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "T0 expired in state %d\n", s->state);
    s->current_status = T30_ERR_T0_EXPIRED;
    /* Just end the call */
    disconnect(s);
}
/*- End of function --------------------------------------------------------*/

static void timer_t1_expired(t30_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "T1 expired in state %d\n", s->state);
    /* The initial connection establishment has timeout out. In other words, we
       have been unable to communicate successfully with a remote machine.
       It is time to abandon the call. */
    s->current_status = T30_ERR_T1_EXPIRED;
    switch (s->state)
    {
    case T30_STATE_T:
        /* Just end the call */
        disconnect(s);
        break;
    case T30_STATE_R:
        /* Send disconnect, and then end the call. Since we have not
           successfully contacted the far end, it is unclear why we should
           send a disconnect message at this point. However, it is what T.30
           says we should do. */
        send_dcn(s);
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static void timer_t2_expired(t30_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "T2 expired in phase %s, state %d\n", phase_names[s->phase], s->state);
    switch (s->state)
    {
    case T30_STATE_F_DOC_ECM:
    case T30_STATE_F_DOC_NON_ECM:
        /* While waiting for FAX page */
        s->current_status = T30_ERR_T2EXPFAXRX;
        break;
    case T30_STATE_F_POST_DOC_ECM:
    case T30_STATE_F_POST_DOC_NON_ECM:
        /* While waiting for next FAX page */
        s->current_status = T30_ERR_T2EXPMPSRX;
        break;
#if 0
    case ??????:
        /* While waiting for DCN */
        s->current_status = T30_ERR_T2EXPDCNRX;
        break;
    case ??????:
        /* While waiting for phase D */
        s->current_status = T30_ERR_T2EXPDRX;
        break;
#endif
    case T30_STATE_IV_PPS_RNR:
    case T30_STATE_IV_EOR_RNR:
        /* While waiting for RR command */
        s->current_status = T30_ERR_T2EXPRRRX;
        break;
    case T30_STATE_R:
        /* While waiting for NSS, DCS or MCF */
        s->current_status = T30_ERR_T2EXPRX;
        break;
    }
    set_phase(s, T30_PHASE_B_TX);
    start_receiving_document(s);
}
/*- End of function --------------------------------------------------------*/

static void timer_t3_expired(t30_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "T3 expired in phase %s, state %d\n", phase_names[s->phase], s->state);
    s->current_status = T30_ERR_T3_EXPIRED;
    disconnect(s);
}
/*- End of function --------------------------------------------------------*/

static void timer_t4_expired(t30_state_t *s)
{
    /* There was no response (or only a corrupt response) to a command */
    span_log(&s->logging, SPAN_LOG_FLOW, "T4 expired in phase %s, state %d\n", phase_names[s->phase], s->state);
    if (++s->retries > MAX_MESSAGE_TRIES)
    {
        switch (s->state)
        {
        case T30_STATE_D_POST_TCF:
            /* Received no response to DCS or TCF */
            s->current_status = T30_ERR_PHBDEADTX;
            break;
        case T30_STATE_II_Q:
        case T30_STATE_IV_PPS_NULL:
        case T30_STATE_IV_PPS_Q:
            /* No response after sending a page */
            s->current_status = T30_ERR_PHDDEADTX;
            break;
        default:
            /* Disconnected after permitted retries */
            s->current_status = T30_ERR_RETRYDCN;
            break;
        }
        send_dcn(s);
        return;
    }
    repeat_last_command(s);
}
/*- End of function --------------------------------------------------------*/

static void timer_t5_expired(t30_state_t *s)
{
    /* Give up waiting for the receiver to become ready in error correction mode */
    span_log(&s->logging, SPAN_LOG_FLOW, "T5 expired in phase %s, state %d\n", phase_names[s->phase], s->state);
    s->current_status = T30_ERR_T5_EXPIRED;
    send_dcn(s);
}
/*- End of function --------------------------------------------------------*/

void t30_timer_update(t30_state_t *s, int samples)
{
    if (s->timer_t0_t1 > 0)
    {
        s->timer_t0_t1 -= samples;
        if (s->timer_t0_t1 <= 0)
        {
            if (s->far_end_detected)
                timer_t1_expired(s);
            else
                timer_t0_expired(s);
        }
    }
    if (s->timer_t3 > 0)
    {
        s->timer_t3 -= samples;
        if (s->timer_t3 <= 0)
            timer_t3_expired(s);
    }
    if (s->timer_t2_t4 > 0)
    {
        s->timer_t2_t4 -= samples;
        if (s->timer_t2_t4 <= 0)
        {
            if (s->timer_is_t4)
                timer_t4_expired(s);
            else
                timer_t2_expired(s);
        }
    }
    if (s->timer_t5 > 0)
    {
        s->timer_t5 -= samples;
        if (s->timer_t5 <= 0)
            timer_t5_expired(s);
    }
}
/*- End of function --------------------------------------------------------*/

static void decode_20digit_msg(t30_state_t *s, char *msg, const uint8_t *pkt, int len)
{
    int p;
    int k;
    char text[20 + 1];

    if (msg == NULL)
        msg = text;
    if (len > T30_MAX_IDENT_LEN)
    {
        unexpected_frame_length(s, pkt, len);
        msg[0] = '\0';
        return;
    }
    p = len;
    /* Strip trailing spaces */
    while (p > 1  &&  pkt[p - 1] == ' ')
        p--;
    /* The string is actually backwards in the message */
    k = 0;
    while (p > 1)
        msg[k++] = pkt[--p];
    msg[k] = '\0';
    span_log(&s->logging, SPAN_LOG_FLOW, "Remote fax gave %s as: \"%s\"\n", t30_frametype(pkt[0]), msg);
}
/*- End of function --------------------------------------------------------*/

static void decode_url_msg(t30_state_t *s, char *msg, const uint8_t *pkt, int len)
{
    char text[77 + 1];

    /* TODO: decode properly, as per T.30 5.3.6.2.12 */
    if (msg == NULL)
        msg = text;
    if (len < 3  ||  len > 77 + 3  ||  len != pkt[2] + 3)
    {
        unexpected_frame_length(s, pkt, len);
        msg[0] = '\0';
        return;
    }
    /* First octet is the sequence number of the packet.
            Bit 7 = 1 for more follows, 0 for last packet in the sequence.
            Bits 6-0 = The sequence number, 0 to 0x7F
       Second octet is the type of internet address.
            Bits 7-4 = reserved
            Bits 3-0 = type:
                    0 = reserved
                    1 = e-mail address
                    2 = URL
                    3 = TCP/IP V4
                    4 = TCP/IP V6
                    5 = international phone number, in the usual +... format
                    6-15 = reserved
       Third octet is the length of the internet address
            Bit 7 = 1 for more follows, 0 for last packet in the sequence.
            Bits 6-0 = length
     */
    memcpy(msg, &pkt[3], len - 3);
    msg[len - 3] = '\0';
    span_log(&s->logging, SPAN_LOG_FLOW, "Remote fax gave %s as: %d, %d, \"%s\"\n", t30_frametype(pkt[0]), pkt[0], pkt[1], msg);
}
/*- End of function --------------------------------------------------------*/

const char *t30_frametype(uint8_t x)
{
    switch (x & 0xFE)
    {
    case T30_DIS:
        if (x == T30_DTC)
            return "DTC";
        return "DIS";
    case T30_CSI:
        if (x == T30_CIG)
            return "CIG";
        return "CSI";
    case T30_NSF:
        if (x == T30_NSC)
            return "NSC";
        return "NSF";
    case T30_PWD & 0xFE:
        if (x == T30_PWD)
            return "PWD";
        break;
    case T30_SEP & 0xFE:
        if (x == T30_SEP)
            return "SEP";
        break;
    case T30_PSA & 0xFE:
        if (x == T30_PSA)
            return "PSA";
        break;
    case T30_CIA & 0xFE:
        if (x == T30_CIA)
            return "CIA";
        break;
    case T30_ISP & 0xFE:
        if (x == T30_ISP)
            return "ISP";
        break;
    case T30_DCS:
        return "DCS";
    case T30_TSI:
        return "TSI";
    case T30_NSS:
        return "NSS";
    case T30_SUB:
        return "SUB";
    case T30_SID:
        return "SID";
    case T30_CTC:
        return "CTC";
    case T30_TSA:
        return "TSA";
    case T30_IRA:
        return "IRA";
    case T30_CFR:
        return "CFR";
    case T30_FTT:
        return "FTT";
    case T30_CTR:
        return "CTR";
    case T30_CSA:
        return "CSA";
    case T30_EOM:
        return "EOM";
    case T30_MPS:
        return "MPS";
    case T30_EOP:
        return "EOP";
    case T30_PRI_EOM:
        return "PRI_EOM";
    case T30_PRI_MPS:
        return "PRI_MPS";
    case T30_PRI_EOP:
        return "PRI_EOP";
    case T30_EOS:
        return "EOS";
    case T30_PPS:
        return "PPS";
    case T30_EOR:
        return "EOR";
    case T30_RR:
        return "RR";
    case T30_MCF:
        return "MCF";
    case T30_RTP:
        return "RTP";
    case T30_RTN:
        return "RTN";
    case T30_PIP:
        return "PIP";
    case T30_PIN:
        return "PIN";
    case T30_PPR:
        return "PPR";
    case T30_RNR:
        return "RNR";
    case T30_ERR:
        return "ERR";
    case T30_FDM:
        return "FDM";
    case T30_DCN:
        return "DCN";
    case T30_CRP:
        return "CRP";
    case T30_FNV:
        return "FNV";
    case T30_TNR:
        return "TNR";
    case T30_TR:
        return "TR";
    case T30_PID:
        return "PID";
    case T30_NULL:
        return "NULL";
    case T4_FCD:
        return "FCD";
    case T4_RCP:
        return "RCP";
    }
    return "???";
}
/*- End of function --------------------------------------------------------*/

static void octet_reserved_bit(logging_state_t *log,
                               const uint8_t *msg,
                               int bit_no,
                               int expected)
{
    char s[10] = ".... ....";
    int bit;
    uint8_t octet;
    
    /* Break out the octet and the bit number within it. */
    octet = msg[((bit_no - 1) >> 3) + 3];
    bit_no = (bit_no - 1) & 7;
    /* Now get the actual bit. */
    bit = (octet >> bit_no) & 1;
    /* Is it what it should be. */
    if (bit ^ expected)
    {
        /* Only log unexpected values. */
        s[7 - bit_no + ((bit_no < 4)  ?  1  :  0)] = (uint8_t) (bit + '0');
        span_log(log, SPAN_LOG_FLOW, "  %s= Unexpected state for reserved bit: %d\n", s, bit);
    }
}
/*- End of function --------------------------------------------------------*/

static void octet_bit_field(logging_state_t *log,
                            const uint8_t *msg,
                            int bit_no,
                            const char *desc,
                            const char *yeah,
                            const char *neigh)
{
    char s[10] = ".... ....";
    int bit;
    uint8_t octet;
    const char *tag;

    /* Break out the octet and the bit number within it. */
    octet = msg[((bit_no - 1) >> 3) + 3];
    bit_no = (bit_no - 1) & 7;
    /* Now get the actual bit. */
    bit = (octet >> bit_no) & 1;
    /* Edit the bit string for display. */
    s[7 - bit_no + ((bit_no < 4)  ?  1  :  0)] = (uint8_t) (bit + '0');
    /* Find the right tag to display. */
    if (bit)
    {
        if ((tag = yeah) == NULL)
            tag = "Set";
    }
    else
    {
        if ((tag = neigh) == NULL)
            tag = "Not set";
    }
    /* Eh, voila! */
    span_log(log, SPAN_LOG_FLOW, "  %s= %s: %s\n", s, desc, tag);
}
/*- End of function --------------------------------------------------------*/

static void octet_field(logging_state_t *log,
                        const uint8_t *msg,
                        int start,
                        int end,
                        const char *desc,
                        const value_string_t tags[])
{
    char s[10] = ".... ....";
    int i;
    uint8_t octet;
    const char *tag;
    
    /* Break out the octet and the bit number range within it. */
    octet = msg[((start - 1) >> 3) + 3];
    start = (start - 1) & 7;
    end = ((end - 1) & 7) + 1;

    /* Edit the bit string for display. */
    for (i = start;  i < end;  i++)
        s[7 - i + ((i < 4)  ?  1  :  0)] = (uint8_t) ((octet >> i) & 1) + '0';

    /* Find the right tag to display. */
    octet = (uint8_t) ((octet >> start) & ((0xFF + (1 << (end - start))) & 0xFF));
    tag = "Invalid";
    for (i = 0;  tags[i].str;  i++)
    {
        if (octet == tags[i].val)
        {
            tag = tags[i].str;
            break;
        }
    }
    /* Eh, voila! */
    span_log(log, SPAN_LOG_FLOW, "  %s= %s: %s\n", s, desc, tag);
}
/*- End of function --------------------------------------------------------*/

void t30_decode_dis_dtc_dcs(t30_state_t *s, const uint8_t *pkt, int len)
{
    logging_state_t *log;
    uint8_t frame_type;
    static const value_string_t available_signalling_rate_tags[] =
    {
        { 0x00, "V.27 ter fall-back mode" },
        { 0x01, "V.29" },
        { 0x02, "V.27 ter" },
        { 0x03, "V.27 ter and V.29" },
        { 0x0B, "V.27 ter, V.29, and V.17" },
        { 0x06, "Reserved" },
        { 0x0A, "Reserved" },
        { 0x0E, "Reserved" },
        { 0x0F, "Reserved" },
        { 0x04, "Not used" },
        { 0x05, "Not used" },
        { 0x08, "Not used" },
        { 0x09, "Not used" },
        { 0x0C, "Not used" },
        { 0x0D, "Not used" },
        { 0x00, NULL }
    };
    static const value_string_t selected_signalling_rate_tags[] =
    {
        { 0x00, "V.27ter 2400bps" },
        { 0x01, "V.29, 9600bps" },
        { 0x02, "V.27ter 4800bps" },
        { 0x03, "V.29 7200bps" },
        { 0x08, "V.17 14400bps" },
        { 0x09, "V.17 9600bps" },
        { 0x0A, "V.17 12000bps" },
        { 0x0B, "V.17 7200bps" },
        { 0x05, "Reserved" },
        { 0x07, "Reserved" },
        { 0x0C, "Reserved" },
        { 0x0D, "Reserved" },
        { 0x0E, "Reserved" },
        { 0x0F, "Reserved" },
        { 0x00, NULL }
    };
    static const value_string_t available_scan_line_length_tags[] =
    {
        { 0x00, "215mm +- 1%" },
        { 0x01, "215mm +- 1% and 255mm +- 1%" },
        { 0x02, "215mm +- 1%, 255mm +- 1% and 303mm +- 1%" },
        { 0x00, NULL }
    };
    static const value_string_t selected_scan_line_length_tags[] =
    {
        { 0x00, "215mm +- 1%" },
        { 0x01, "255mm +- 1%" },
        { 0x02, "303mm +- 1%" },
        { 0x00, NULL }
    };
    static const value_string_t available_recording_length_tags[] =
    {
        { 0x00, "A4 (297mm)" },
        { 0x01, "A4 (297mm) and B4 (364mm)" },
        { 0x02, "Unlimited" },
        { 0x00, NULL }
    };
    static const value_string_t selected_recording_length_tags[] =
    {
        { 0x00, "A4 (297mm)" },
        { 0x01, "B4 (364mm)" },
        { 0x02, "Unlimited" },
        { 0x00, NULL }
    };
    static const value_string_t available_minimum_scan_line_time_tags[] =
    {
        { 0x00, "20ms at 3.85 l/mm; T7.7 = T3.85" },
        { 0x01, "5ms at 3.85 l/mm; T7.7 = T3.85" },
        { 0x02, "10ms at 3.85 l/mm; T7.7 = T3.85" },
        { 0x03, "20ms at 3.85 l/mm; T7.7 = 1/2 T3.85" },
        { 0x04, "40ms at 3.85 l/mm; T7.7 = T3.85" },
        { 0x05, "40ms at 3.85 l/mm; T7.7 = 1/2 T3.85" },
        { 0x06, "10ms at 3.85 l/mm; T7.7 = 1/2 T3.85" },
        { 0x07, "0ms at 3.85 l/mm; T7.7 = T3.85" },
        { 0x00, NULL }
    };
    static const value_string_t selected_minimum_scan_line_time_tags[] =
    {
        { 0x00, "20ms" },
        { 0x01, "5ms" },
        { 0x02, "10ms" },
        { 0x04, "40ms" },
        { 0x07, "0ms" },
        { 0x00, NULL }
    };
    static const value_string_t shared_data_memory_capacity_tags[] =
    {
        { 0x00, "Not available" },
        { 0x01, "Level 2 = 2.0 Mbytes" },
        { 0x02, "Level 1 = 1.0 Mbytes" },
        { 0x03, "Level 3 = unlimited (i.e. >= 32 Mbytes)" },
        { 0x00, NULL }
    };
    static const value_string_t t89_profile_tags[] =
    {
        { 0x00, "Not used" },
        { 0x01, "Profiles 2 and 3" },
        { 0x02, "Profile 2" },
        { 0x04, "Profile 1" },
        { 0x06, "Profile 3" },
        { 0x03, "Reserved" },
        { 0x05, "Reserved" },
        { 0x07, "Reserved" },
        { 0x00, NULL }
    };
    static const value_string_t t44_mixed_raster_content_tags[] =
    {
        { 0x00, "0" },
        { 0x01, "1" },
        { 0x02, "2" },
        { 0x32, "3" },
        { 0x04, "4" },
        { 0x05, "5" },
        { 0x06, "6" },
        { 0x07, "7" },
        { 0x00, NULL }
    };

    if (!span_log_test(&s->logging, SPAN_LOG_FLOW))
        return;
    frame_type = pkt[2] & 0xFE;
    log = &s->logging;
    if (len <= 2)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }
    
    span_log(log, SPAN_LOG_FLOW, "%s:\n", t30_frametype(pkt[2]));
    if (len <= 3)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }
    octet_bit_field(log, pkt, 1, "Store and forward Internet fax (T.37)", NULL, NULL);
    octet_reserved_bit(log, pkt, 2, 0);
    octet_bit_field(log, pkt, 3, "Real-time Internet fax (T.38)", NULL, NULL);
    octet_bit_field(log, pkt, 4, "3G mobile network", NULL, NULL);
    octet_reserved_bit(log, pkt, 5, 0);
    if (frame_type == T30_DCS)
    {
        octet_reserved_bit(log, pkt, 6, 0);
        octet_reserved_bit(log, pkt, 7, 0);
    }
    else
    {
        octet_bit_field(log, pkt, 6, "V.8 capabilities", NULL, NULL);
        octet_bit_field(log, pkt, 7, "Preferred octets", "64 octets", "256 octets");
    }
    octet_reserved_bit(log, pkt, 8, 0);
    if (len <= 4)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }
    
    if (frame_type == T30_DCS)
    {
        octet_reserved_bit(log, pkt, 9, 0);
        octet_bit_field(log, pkt, 10, "Receive fax", NULL, NULL);
        octet_field(log, pkt, 11, 14, "Selected data signalling rate", selected_signalling_rate_tags);
    }
    else
    {
        octet_bit_field(log, pkt, 9, "Ready to transmit a fax document (polling)", NULL, NULL);
        octet_bit_field(log, pkt, 10, "Can receive fax", NULL, NULL);
        octet_field(log, pkt, 11, 14, "Supported data signalling rates", available_signalling_rate_tags);
    }
    octet_bit_field(log, pkt, 15, "R8x7.7lines/mm and/or 200x200pels/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 16, "2-D coding", NULL, NULL);
    if (len <= 5)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    if (frame_type == T30_DCS)
    {
        octet_field(log, pkt, 17, 18, "Recording width", selected_scan_line_length_tags);
        octet_field(log, pkt, 19, 20, "Recording length", selected_recording_length_tags);
        octet_field(log, pkt, 21, 23, "Minimum scan line time", selected_minimum_scan_line_time_tags);
    }
    else
    {
        octet_field(log, pkt, 17, 18, "Recording width", available_scan_line_length_tags);
        octet_field(log, pkt, 19, 20, "Recording length", available_recording_length_tags);
        octet_field(log, pkt, 21, 23, "Receiver's minimum scan line time", available_minimum_scan_line_time_tags);
    }
    octet_bit_field(log, pkt, 24, "Extension indicator", NULL, NULL);
    if (!(pkt[5] & DISBIT8))
        return;
    if (len <= 6)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_reserved_bit(log, pkt, 25, 0);
    octet_bit_field(log, pkt, 26, "Compressed/uncompressed mode", "Uncompressed", "Compressed");
    octet_bit_field(log, pkt, 27, "Error correction mode (ECM)", "ECM", "Non-ECM");
    if (frame_type == T30_DCS)
        octet_bit_field(log, pkt, 28, "Frame size", "64 octets", "256 octets");
    else
        octet_reserved_bit(log, pkt, 28, 0);
    octet_reserved_bit(log, pkt, 29, 0);
    octet_reserved_bit(log, pkt, 30, 0);
    octet_bit_field(log, pkt, 31, "T.6 coding", NULL, NULL);
    octet_bit_field(log, pkt, 32, "Extension indicator", NULL, NULL);
    if (!(pkt[6] & DISBIT8))
        return;
    if (len <= 7)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 33, "\"Field not valid\" supported", NULL, NULL);
    if (frame_type == T30_DCS)
    {
        octet_reserved_bit(log, pkt, 34, 0);
        octet_reserved_bit(log, pkt, 35, 0);
    }
    else
    {
        octet_bit_field(log, pkt, 34, "Multiple selective polling", NULL, NULL);
        octet_bit_field(log, pkt, 35, "Polled subaddress", NULL, NULL);
    }
    octet_bit_field(log, pkt, 36, "T.43 coding", NULL, NULL);
    octet_bit_field(log, pkt, 37, "Plane interleave", NULL, NULL);
    octet_bit_field(log, pkt, 38, "Voice coding with 32kbit/s ADPCM (Rec. G.726)", NULL, NULL);
    octet_bit_field(log, pkt, 39, "Reserved for the use of extended voice coding set", NULL, NULL);
    octet_bit_field(log, pkt, 40, "Extension indicator", NULL, NULL);
    if (!(pkt[7] & DISBIT8))
        return;
    if (len <= 8)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }
    octet_bit_field(log, pkt, 41, "R8x15.4lines/mm", NULL, NULL);
    octet_bit_field(log, pkt, 42, "300x300pels/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 43, "R16x15.4lines/mm and/or 400x400pels/25.4mm", NULL, NULL);
    if (frame_type == T30_DCS)
    {
        octet_bit_field(log, pkt, 44, "Resolution type selection", "Inch", "Metric");
        octet_reserved_bit(log, pkt, 45, 0);
        octet_reserved_bit(log, pkt, 46, 0);
        octet_reserved_bit(log, pkt, 47, 0);
    }
    else
    {
        octet_bit_field(log, pkt, 44, "Inch-based resolution preferred", NULL, NULL);
        octet_bit_field(log, pkt, 45, "Metric-based resolution preferred", NULL, NULL);
        octet_bit_field(log, pkt, 46, "Minimum scan line time for higher resolutions", "T15.4 = 1/2 T7.7", "T15.4 = T7.7");
        octet_bit_field(log, pkt, 47, "Selective polling", NULL, NULL);
    }
    octet_bit_field(log, pkt, 48, "Extension indicator", NULL, NULL);
    if (!(pkt[8] & DISBIT8))
        return;
    if (len <= 9)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 49, "Subaddressing", NULL, NULL);
    if (frame_type == T30_DCS)
    {
        octet_bit_field(log, pkt, 50, "Sender identification transmission", NULL, NULL);
        octet_reserved_bit(log, pkt, 51, 0);
    }
    else
    {
        octet_bit_field(log, pkt, 50, "Password", NULL, NULL);
        octet_bit_field(log, pkt, 51, "Ready to transmit a data file (polling)", NULL, NULL);
    }
    octet_reserved_bit(log, pkt, 52, 0);
    octet_bit_field(log, pkt, 53, "Binary file transfer (BFT)", NULL, NULL);
    octet_bit_field(log, pkt, 54, "Document transfer mode (DTM)", NULL, NULL);
    octet_bit_field(log, pkt, 55, "Electronic data interchange (EDI)", NULL, NULL);
    octet_bit_field(log, pkt, 56, "Extension indicator", NULL, NULL);
    if (!(pkt[9] & DISBIT8))
        return;
    if (len <= 10)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 57, "Basic transfer mode (BTM)", NULL, NULL);
    octet_reserved_bit(log, pkt, 58, 0);
    if (frame_type == T30_DCS)
        octet_reserved_bit(log, pkt, 59, 0);
    else
        octet_bit_field(log, pkt, 59, "Ready to transfer a character or mixed mode document (polling)", NULL, NULL);
    octet_bit_field(log, pkt, 60, "Character mode", NULL, NULL);
    octet_reserved_bit(log, pkt, 61, 0);
    octet_bit_field(log, pkt, 62, "Mixed mode (Annex E/T.4)", NULL, NULL);
    octet_reserved_bit(log, pkt, 63, 0);
    octet_bit_field(log, pkt, 64, "Extension indicator", NULL, NULL);
    if (!(pkt[10] & DISBIT8))
        return;
    if (len <= 11)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 65, "Processable mode 26 (Rec. T.505)", NULL, NULL);
    octet_bit_field(log, pkt, 66, "Digital network capability", NULL, NULL);
    octet_bit_field(log, pkt, 67, "Duplex capability", "Full", "Half only");
    if (frame_type == T30_DCS)
        octet_bit_field(log, pkt, 68, "Full colour mode", NULL, NULL);
    else
        octet_bit_field(log, pkt, 68, "JPEG coding", NULL, NULL);
    octet_bit_field(log, pkt, 69, "Full colour mode", NULL, NULL);
    if (frame_type == T30_DCS)
        octet_bit_field(log, pkt, 70, "Preferred Huffman tables", NULL, NULL);
    else
        octet_reserved_bit(log, pkt, 70, 0);
    octet_bit_field(log, pkt, 71, "12bits/pel component", NULL, NULL);
    octet_bit_field(log, pkt, 72, "Extension indicator", NULL, NULL);
    if (!(pkt[11] & DISBIT8))
        return;
    if (len <= 12)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 73, "No subsampling (1:1:1)", NULL, NULL);
    octet_bit_field(log, pkt, 74, "Custom illuminant", NULL, NULL);
    octet_bit_field(log, pkt, 75, "Custom gamut range", NULL, NULL);
    octet_bit_field(log, pkt, 76, "North American Letter (215.9mm x 279.4mm)", NULL, NULL);
    octet_bit_field(log, pkt, 77, "North American Legal (215.9mm x 355.6mm)", NULL, NULL);
    octet_bit_field(log, pkt, 78, "Single-progression sequential coding (Rec. T.85) basic", NULL, NULL);
    octet_bit_field(log, pkt, 79, "Single-progression sequential coding (Rec. T.85) optional L0", NULL, NULL);
    octet_bit_field(log, pkt, 80, "Extension indicator", NULL, NULL);
    if (!(pkt[12] & DISBIT8))
        return;
    if (len <= 13)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 81, "HKM key management", NULL, NULL);
    octet_bit_field(log, pkt, 82, "RSA key management", NULL, NULL);
    octet_bit_field(log, pkt, 83, "Override", NULL, NULL);
    octet_bit_field(log, pkt, 84, "HFX40 cipher", NULL, NULL);
    octet_bit_field(log, pkt, 85, "Alternative cipher number 2", NULL, NULL);
    octet_bit_field(log, pkt, 86, "Alternative cipher number 3", NULL, NULL);
    octet_bit_field(log, pkt, 87, "HFX40-I hashing", NULL, NULL);
    octet_bit_field(log, pkt, 88, "Extension indicator", NULL, NULL);
    if (!(pkt[13] & DISBIT8))
        return;
    if (len <= 14)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 89, "Alternative hashing system 2", NULL, NULL);
    octet_bit_field(log, pkt, 90, "Alternative hashing system 3", NULL, NULL);
    octet_bit_field(log, pkt, 91, "Reserved for future security features", NULL, NULL);
    octet_field(log, pkt, 92, 94, "T.44 (Mixed Raster Content)", t44_mixed_raster_content_tags);
    octet_bit_field(log, pkt, 95, "Page length maximum stripe size for T.44 (Mixed Raster Content)", NULL, NULL);
    octet_bit_field(log, pkt, 96, "Extension indicator", NULL, NULL);
    if (!(pkt[14] & DISBIT8))
        return;
    if (len <= 15)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 97, "Colour/gray-scale 300pels/25.4mm x 300lines/25.4mm or 400pels/25.4mm x 400lines/25.4mm resolution", NULL, NULL);
    octet_bit_field(log, pkt, 98, "100pels/25.4mm x 100lines/25.4mm for colour/gray scale", NULL, NULL);
    octet_bit_field(log, pkt, 99, "Simple phase C BFT negotiations", NULL, NULL);
    if (frame_type == T30_DCS)
    {
        octet_reserved_bit(log, pkt, 100, 0);
        octet_reserved_bit(log, pkt, 101, 0);
    }
    else
    {
        octet_bit_field(log, pkt, 100, "Extended BFT Negotiations capable", NULL, NULL);
        octet_bit_field(log, pkt, 101, "Internet Selective Polling address (ISP)", NULL, NULL);
    }
    octet_bit_field(log, pkt, 102, "Internet Routing Address (IRA)", NULL, NULL);
    octet_reserved_bit(log, pkt, 103, 0);
    octet_bit_field(log, pkt, 104, "Extension indicator", NULL, NULL);
    if (!(pkt[15] & DISBIT8))
        return;
    if (len <= 16)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 105, "600pels/25.4mm x 600lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 106, "1200pels/25.4mm x 1200lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 107, "300pels/25.4mm x 600lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 108, "400pels/25.4mm x 800lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 109, "600pels/25.4mm x 1200lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 110, "Colour/gray scale 600pels/25.4mm x 600lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 111, "Colour/gray scale 1200pels/25.4mm x 1200lines/25.4mm", NULL, NULL);
    octet_bit_field(log, pkt, 112, "Extension indicator", NULL, NULL);
    if (!(pkt[16] & DISBIT8))
        return;
    if (len <= 17)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 113, "Double sided printing capability (alternate mode)", NULL, NULL);
    octet_bit_field(log, pkt, 114, "Double sided printing capability (continuous mode)", NULL, NULL);
    if (frame_type == T30_DCS)
        octet_bit_field(log, pkt, 115, "Black and white mixed raster content profile (MRCbw)", NULL, NULL);
    else
        octet_reserved_bit(log, pkt, 115, 0);
    octet_bit_field(log, pkt, 116, "T.45 (run length colour encoded)", NULL, NULL);
    octet_field(log, pkt, 117, 118, "Shared memory", shared_data_memory_capacity_tags);
    octet_bit_field(log, pkt, 119, "T.44 colour space", NULL, NULL);
    octet_bit_field(log, pkt, 120, "Extension indicator", NULL, NULL);
    if (!(pkt[17] & DISBIT8))
        return;
    if (len <= 18)
    {
        span_log(log, SPAN_LOG_FLOW, "  Frame is short\n");
        return;
    }

    octet_bit_field(log, pkt, 121, "Flow control capability for T.38 communication", NULL, NULL);
    octet_bit_field(log, pkt, 122, "K>4", NULL, NULL);
    octet_bit_field(log, pkt, 123, "Internet aware T.38 mode fax (not affected by data signal rate bits)", NULL, NULL);
    octet_field(log, pkt, 124, 126, "T.89 (Application profiles for ITU-T Rec T.8)", t89_profile_tags);
    octet_bit_field(log, pkt, 127, "sYCC-JPEG coding", NULL, NULL);
    octet_bit_field(log, pkt, 128, "Extension indicator", NULL, NULL);
    if (!(pkt[18] & DISBIT8))
        return;

    span_log(log, SPAN_LOG_FLOW, "  Extended beyond the current T.30 specification!\n");
}
/*- End of function --------------------------------------------------------*/

int t30_restart(t30_state_t *s)
{
    s->phase = T30_PHASE_IDLE;
    s->next_phase = T30_PHASE_IDLE;
    s->current_fallback = 0;
    s->rx_signal_present = FALSE;
    s->rx_trained = FALSE;
    s->current_status = T30_ERR_OK;
    s->ppr_count = 0;
    s->receiver_not_ready_count = 0;

    build_dis_or_dtc(s);
    if (s->calling_party)
    {
        set_state(s, T30_STATE_T);
        set_phase(s, T30_PHASE_A_CNG);
    }
    else
    {
        set_state(s, T30_STATE_ANSWERING);
        set_phase(s, T30_PHASE_A_CED);
    }
    s->far_end_detected = FALSE;
    s->timer_t0_t1 = ms_to_samples(DEFAULT_TIMER_T0);
    return 0;
}
/*- End of function --------------------------------------------------------*/

t30_state_t *t30_init(t30_state_t *s,
                      int calling_party,
                      t30_set_handler_t *set_rx_type_handler,
                      void *set_rx_type_user_data,
                      t30_set_handler_t *set_tx_type_handler,
                      void *set_tx_type_user_data,
                      t30_send_hdlc_handler_t *send_hdlc_handler,
                      void *send_hdlc_user_data)
{
    if (s == NULL)
    {
        if ((s = (t30_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));
    s->calling_party = calling_party;
    s->set_rx_type_handler = set_rx_type_handler;
    s->set_rx_type_user_data = set_rx_type_user_data;
    s->set_tx_type_handler = set_tx_type_handler;
    s->set_tx_type_user_data = set_tx_type_user_data;
    s->send_hdlc_handler = send_hdlc_handler;
    s->send_hdlc_user_data = send_hdlc_user_data;

    /* Default to the basic modems. */
    s->supported_modems = T30_SUPPORT_V27TER | T30_SUPPORT_V29;
    s->supported_compressions = T30_SUPPORT_T4_1D_COMPRESSION | T30_SUPPORT_T4_2D_COMPRESSION;
    s->supported_resolutions = T30_SUPPORT_STANDARD_RESOLUTION | T30_SUPPORT_FINE_RESOLUTION | T30_SUPPORT_SUPERFINE_RESOLUTION
                             | T30_SUPPORT_R8_RESOLUTION;
    s->supported_image_sizes = T30_SUPPORT_US_LETTER_LENGTH | T30_SUPPORT_US_LEGAL_LENGTH | T30_SUPPORT_UNLIMITED_LENGTH
                             | T30_SUPPORT_215MM_WIDTH;
    /* Set the output encoding to something safe. Most things get 1D and 2D
       encoding right. Quite a lot get other things wrong. */
    s->output_encoding = T4_COMPRESSION_ITU_T4_2D;
    span_log_init(&s->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&s->logging, "T.30");
    t30_restart(s);
    return s;
}
/*- End of function --------------------------------------------------------*/

int t30_release(t30_state_t *s)
{
    /* Make sure any FAX in progress is tidied up. If the tidying up has
       already happened, repeating it here is harmless. */
    t4_rx_end(&(s->t4));
    t4_tx_end(&(s->t4));
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_free(t30_state_t *s)
{
    t30_release(s);
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

void t30_terminate(t30_state_t *s)
{
    if (s->phase != T30_PHASE_CALL_FINISHED)
    {
        /* The far end disconnected early, but was it just a tiny bit too early,
           as we were just tidying up, or seriously early as in a failure? */
        switch (s->state)
        {
        case T30_STATE_C:
            /* We were sending the final disconnect, so just hussle things along. */
            disconnect(s);
            break;
        case T30_STATE_B:
            /* We were in the final wait for everything to flush through, so just
               hussle things along. */
            break;
        default:
            /* The call terminated prematurely. */
            s->current_status = T30_ERR_CALLDROPPED;
            break;
        }
        if (s->phase_e_handler)
            s->phase_e_handler(s, s->phase_e_user_data, s->current_status);
        set_state(s, T30_STATE_CALL_FINISHED);
        set_phase(s, T30_PHASE_CALL_FINISHED);
    }
}
/*- End of function --------------------------------------------------------*/

void t30_set_iaf_mode(t30_state_t *s, int iaf)
{
    s->iaf = iaf;
}
/*- End of function --------------------------------------------------------*/

int t30_set_header_info(t30_state_t *s, const char *info)
{
    if (info == NULL)
    {
        s->header_info[0] = '\0';
        return 0;
    }
    if (strlen(info) > 50)
        return -1;
    strcpy(s->header_info, info);
    t4_tx_set_header_info(&(s->t4), s->header_info);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_local_ident(t30_state_t *s, const char *id)
{
    if (id == NULL)
    {
        s->local_ident[0] = '\0';
        return 0;
    }
    if (strlen(id) > 20)
        return -1;
    strcpy(s->local_ident, id);
    t4_tx_set_local_ident(&(s->t4), s->local_ident);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_local_nsf(t30_state_t *s, const uint8_t *nsf, int len)
{
    if (len > T30_MAX_LOCAL_NSF_LEN)
        return -1;
    memcpy(s->local_nsf, nsf, len);
    s->local_nsf_len = len;
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_local_sub_address(t30_state_t *s, const char *sub_address)
{
    if (sub_address == NULL)
    {
        s->local_sub_address[0] = '\0';
        return 0;
    }
    if (strlen(sub_address) > 20)
        return -1;
    strcpy(s->local_sub_address, sub_address);
    return 0;
}
/*- End of function --------------------------------------------------------*/

size_t t30_get_sub_address(t30_state_t *s, char *sub_address)
{
    if (sub_address)
        strcpy(sub_address, s->far_sub_address);
    return strlen(s->far_sub_address);
}
/*- End of function --------------------------------------------------------*/

size_t t30_get_header_info(t30_state_t *s, char *info)
{
    if (info)
        strcpy(info, s->header_info);
    return strlen(s->header_info);
}
/*- End of function --------------------------------------------------------*/

size_t t30_get_local_ident(t30_state_t *s, char *id)
{
    if (id)
        strcpy(id, s->local_ident);
    return strlen(s->local_ident);
}
/*- End of function --------------------------------------------------------*/

size_t t30_get_far_ident(t30_state_t *s, char *id)
{
    if (id)
        strcpy(id, s->far_ident);
    return strlen(s->far_ident);
}
/*- End of function --------------------------------------------------------*/

const char *t30_get_far_country(t30_state_t *s)
{
    return s->country;
}
/*- End of function --------------------------------------------------------*/

const char *t30_get_far_vendor(t30_state_t *s)
{
    return s->vendor;
}
/*- End of function --------------------------------------------------------*/

const char *t30_get_far_model(t30_state_t *s)
{
    return s->model;
}
/*- End of function --------------------------------------------------------*/

void t30_get_transfer_statistics(t30_state_t *s, t30_stats_t *t)
{
    t4_stats_t stats;

    t->bit_rate = fallback_sequence[s->current_fallback].bit_rate;
    t->error_correcting_mode = s->error_correcting_mode;
    t4_get_transfer_statistics(&(s->t4), &stats);
    t->pages_transferred = stats.pages_transferred;
    t->pages_in_file = stats.pages_in_file;
    t->width = stats.width;
    t->length = stats.length;
    t->bad_rows = stats.bad_rows;
    t->longest_bad_row_run = stats.longest_bad_row_run;
    t->x_resolution = stats.x_resolution;
    t->y_resolution = stats.y_resolution;
    t->encoding = stats.encoding;
    t->image_size = stats.image_size;
    t->current_status = s->current_status;
}
/*- End of function --------------------------------------------------------*/

void t30_set_phase_b_handler(t30_state_t *s, t30_phase_b_handler_t *handler, void *user_data)
{
    s->phase_b_handler = handler;
    s->phase_b_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

void t30_set_phase_d_handler(t30_state_t *s, t30_phase_d_handler_t *handler, void *user_data)
{
    s->phase_d_handler = handler;
    s->phase_d_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

void t30_set_phase_e_handler(t30_state_t *s, t30_phase_e_handler_t *handler, void *user_data)
{
    s->phase_e_handler = handler;
    s->phase_e_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

void t30_set_document_handler(t30_state_t *s, t30_document_handler_t *handler, void *user_data)
{
    s->document_handler = handler;
    s->document_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

void t30_set_rx_file(t30_state_t *s, const char *file, int stop_page)
{
    strncpy(s->rx_file, file, sizeof(s->rx_file));
    s->rx_file[sizeof(s->rx_file) - 1] = '\0';
    s->rx_stop_page = stop_page;
}
/*- End of function --------------------------------------------------------*/

void t30_set_tx_file(t30_state_t *s, const char *file, int start_page, int stop_page)
{
    strncpy(s->tx_file, file, sizeof(s->tx_file));
    s->tx_file[sizeof(s->tx_file) - 1] = '\0';
    s->tx_start_page = start_page;
    s->tx_stop_page = stop_page;
}
/*- End of function --------------------------------------------------------*/

int t30_set_receiver_not_ready(t30_state_t *s, int count)
{
    s->receiver_not_ready_count = count;
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_supported_modems(t30_state_t *s, int supported_modems)
{
    s->supported_modems = supported_modems;
    build_dis_or_dtc(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_supported_compressions(t30_state_t *s, int supported_compressions)
{
    s->supported_compressions = supported_compressions;
    build_dis_or_dtc(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_supported_resolutions(t30_state_t *s, int supported_resolutions)
{
    s->supported_resolutions = supported_resolutions;
    build_dis_or_dtc(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_supported_image_sizes(t30_state_t *s, int supported_image_sizes)
{
    s->supported_image_sizes = supported_image_sizes;
    build_dis_or_dtc(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_ecm_capability(t30_state_t *s, int enabled)
{
    s->ecm_allowed = enabled;
    build_dis_or_dtc(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_min_non_ecm_row_bits(t30_state_t *s, int bits)
{
    s->forced_min_non_ecm_row_bits = bits;
    return 0;
}
/*- End of function --------------------------------------------------------*/

int t30_set_rx_encoding(t30_state_t *s, int encoding)
{
    switch (encoding)
    {
    case T4_COMPRESSION_ITU_T4_1D:
    case T4_COMPRESSION_ITU_T4_2D:
    case T4_COMPRESSION_ITU_T6:
        s->output_encoding = encoding;
        return 0;
    }
    return -1;
}
/*- End of function --------------------------------------------------------*/

void t30_local_interrupt_request(t30_state_t *s, int state)
{
    if (s->timer_t3 > 0)
    {
        /* Accept the far end's outstanding request for interrupt. */
        /* TODO: */
        send_simple_frame(s, (state)  ?  T30_PIP  :  T30_PIN);
    }
    s->local_interrupt_pending = state;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
