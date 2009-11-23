/*
 * SpanDSP - a series of DSP components for telephony
 *
 * t38_terminal.h - T.38 termination, less the packet exchange part
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2005 Steve Underwood
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
 * $Id: t38_terminal.h,v 1.24 2007/11/30 12:20:35 steveu Exp $
 */

/*! \file */

#if !defined(_SPANDSP_T38_TERMINAL_H_)
#define _SPANDSP_T38_TERMINAL_H_

/*! \page t38_terminal_page T.38 real time FAX over IP termination
\section t38_terminal_page_sec_1 What does it do?

\section t38_terminal_page_sec_2 How does it work?
*/

/* Make sure the HDLC frame buffers are big enough for ECM frames. */
#define T38_MAX_HDLC_LEN        260

typedef struct
{
    t38_core_state_t t38;

    /*! \brief Use (actually allow time for) talker echo protection when transmitting. */
    int use_tep;    

    /*! \brief HDLC transmit buffer */
    uint8_t tx_buf[T38_MAX_HDLC_LEN];
    int tx_len;
    int tx_ptr;

    /*! \brief HDLC receive buffer */
    uint8_t rx_buf[T38_MAX_HDLC_LEN];
    int rx_len;

    /*! \brief The current transmit step being timed */
    int timed_step;

    /*! \brief The next queued tramsit indicator */
    int next_tx_indicator;
    /*! \brief The current T.38 data type being transmitted */
    int current_tx_data_type;

    /*! \brief TRUE if a carrier is present. Otherwise FALSE. */
    int rx_signal_present;

    /*! \brief The T.30 back-end */
    t30_state_t t30_state;

    int current_rx_type;
    int current_tx_type;
    
    int trailer_bytes;

    /*! \brief TRUE is there has been some T.38 data missed (i.e. lost packets) */
    int missing_data;

    /*! \brief The number of octets to send in each image packet (non-ECM or ECM) at the current
               rate and the current specified packet interval. */
    int octets_per_data_packet;
    
    int ms_per_tx_chunk;
    int merge_tx_fields;

    /*! \brief The number of times an indicator packet will be sent. Numbers greater than one
               will increase reliability for UDP transmission. Zero is valid, to suppress all
               indicator packets for TCP transmission. */
    int indicator_tx_count;

    /*! \brief The number of times a data packet which ends transmission will be sent. Numbers
               greater than one will increase reliability for UDP transmission. Zero is not valid. */
    int data_end_tx_count;

    /*! \brief A "sample" count, used to time events */
    int32_t samples;
    int32_t next_tx_samples;
    int32_t timeout_rx_samples;

    logging_state_t logging;
} t38_terminal_state_t;

#if defined(__cplusplus)
extern "C"
{
#endif

int t38_terminal_send_timeout(t38_terminal_state_t *s, int samples);

void t38_terminal_set_config(t38_terminal_state_t *s, int without_pacing);

/*! Select whether the time for talker echo protection tone will be allowed for when sending.
    \brief Select whether TEP time will be allowed for.
    \param s The T.38 context.
    \param use_tep TRUE if TEP should be allowed for.
*/
void t38_terminal_set_tep_mode(t38_terminal_state_t *s, int use_tep);

    /*! \brief Initialise a termination mode T.38 context.
    \param s The T.38 context.
    \param calling_party TRUE if the context is for a calling party. FALSE if the
           context is for an answering party.
    \param tx_packet_handler A callback routine to encapsulate and transmit T.38 packets.
    \param tx_packet_user_data An opaque pointer passed to the tx_packet_handler routine.
    \return A pointer to the termination mode T.38 context, or NULL if there was a problem. */
t38_terminal_state_t *t38_terminal_init(t38_terminal_state_t *s,
                                        int calling_party,
                                        t38_tx_packet_handler_t *tx_packet_handler,
                                        void *tx_packet_user_data);

/*! Release a termination mode T.38 context.
    \brief Release a T.38 context.
    \param s The T.38 context.
    \return 0 for OK, else -1. */
int t38_terminal_release(t38_terminal_state_t *s);

/*! Free a a termination mode T.38 context.
    \brief Free a T.38 context.
    \param s The T.38 context.
    \return 0 for OK, else -1. */
int t38_terminal_free(t38_terminal_state_t *s);

#if defined(__cplusplus)
}
#endif

#endif
/*- End of file ------------------------------------------------------------*/
