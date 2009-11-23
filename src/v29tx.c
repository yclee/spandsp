/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v29tx.c - ITU V.29 modem transmit part
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2003 Steve Underwood
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
 * $Id: v29tx.c,v 1.67 2007/11/30 12:20:35 steveu Exp $
 */

/*! \file */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif

#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/vector_float.h"
#include "spandsp/complex_vector_float.h"
#include "spandsp/async.h"
#include "spandsp/dds.h"
#include "spandsp/power_meter.h"

#include "spandsp/v29tx.h"

#define CARRIER_NOMINAL_FREQ        1700.0f

/* Segments of the training sequence */
#define V29_TRAINING_SEG_TEP        0
#define V29_TRAINING_SEG_1          (V29_TRAINING_SEG_TEP + 480)
#define V29_TRAINING_SEG_2          (V29_TRAINING_SEG_1 + 48)
#define V29_TRAINING_SEG_3          (V29_TRAINING_SEG_2 + 128)
#define V29_TRAINING_SEG_4          (V29_TRAINING_SEG_3 + 384)
#define V29_TRAINING_END            (V29_TRAINING_SEG_4 + 48)
#define V29_TRAINING_SHUTDOWN_END   (V29_TRAINING_END + 32)

/* Raised root cosine pulse shaping; Beta = 0.25; 4 symbols either
   side of the centre. */
/* Created with mkshape -r 0.05 0.25 91 -l and then split up */
#define PULSESHAPER_GAIN            (9.9888356312f/10.0f)
#define PULSESHAPER_COEFF_SETS      10

#if defined(SPANDSP_USE_FIXED_POINT)
static const int16_t pulseshaper[PULSESHAPER_COEFF_SETS][V29_TX_FILTER_STEPS] =
{
    {
          -90,          /* Filter 0 */
         -561,
         2003,
        -5224,
        19072,
        19072,
        -5224,
         2003,
         -561
    },
    {
           97,          /* Filter 1 */
         -922,
         2554,
        -6055,
        23508,
        14325,
        -3960,
         1301,
         -183
    },
    {
          298,          /* Filter 2 */
        -1210,
         2855,
        -6269,
        27331,
         9578,
        -2462,
          549,
          159
    },
    {
          478,          /* Filter 3 */
        -1371,
         2828,
        -5714,
        30276,
         5121,
         -925,
         -157,
          427
    },
    {
          606,          /* Filter 4 */
        -1360,
         2421,
        -4291,
        32132,
         1208,
          482,
         -741,
          594
    },
    {
          651,          /* Filter 5 */
        -1151,
         1627,
        -1969,
        32767,
        -1969,
         1627,
        -1151,
          651
    },
    {
          594,          /* Filter 6 */
         -741,
          482,
         1208,
        32132,
        -4291,
         2421,
        -1360,
          606
    },
    {
          427,          /* Filter 7 */
         -157,
         -925,
         5121,
        30276,
        -5714,
         2828,
        -1371,
          478
    },
    {
          159,          /* Filter 8 */
          549,
        -2462,
         9578,
        27331,
        -6269,
         2855,
        -1210,
          298
    },
    {
         -183,          /* Filter 9 */
         1301,
        -3960,
        14325,
        23508,
        -6055,
         2554,
         -922,
           97
    }
};
#else
static const float pulseshaper[PULSESHAPER_COEFF_SETS][V29_TX_FILTER_STEPS] =
{
    {
        -0.0029426223f,         /* Filter 0 */
        -0.0183060118f,
         0.0653192857f,
        -0.1703207714f,
         0.6218069936f,
         0.6218069936f,
        -0.1703207714f,
         0.0653192857f,
        -0.0183060118f
    },
    {
         0.0031876922f,         /* Filter 1 */
        -0.0300884145f,
         0.0832744718f,
        -0.1974255221f,
         0.7664229820f,
         0.4670580725f,
        -0.1291107519f,
         0.0424189243f,
        -0.0059810465f
    },
    {
         0.0097229236f,         /* Filter 2 */
        -0.0394811291f,
         0.0931039664f,
        -0.2043906784f,
         0.8910868760f,
         0.3122713836f,
        -0.0802880559f,
         0.0179050490f,
         0.0052057308f
    },
    {
         0.0156117223f,         /* Filter 3 */
        -0.0447125347f,
         0.0922040267f,
        -0.1862939416f,
         0.9870942864f,
         0.1669790517f,
        -0.0301581072f,
        -0.0051358510f,
         0.0139350286f
    },
    {
         0.0197702545f,         /* Filter 4 */
        -0.0443470335f,
         0.0789538534f,
        -0.1399184160f,
         1.0476130256f,
         0.0393903028f,
         0.0157339854f,
        -0.0241879599f,
         0.0193774571f
    },
    {
         0.0212455717f,         /* Filter 5 */
        -0.0375307894f,
         0.0530516472f,
        -0.0642195521f,
         1.0682849922f,
        -0.0642195521f,
         0.0530516472f,
        -0.0375307894f,
         0.0212455717f
    },
    {
         0.0193774571f,         /* Filter 6 */
        -0.0241879599f,
         0.0157339854f,
         0.0393903028f,
         1.0476130256f,
        -0.1399184160f,
         0.0789538534f,
        -0.0443470335f,
         0.0197702545f
    },
    {
         0.0139350286f,         /* Filter 7 */
        -0.0051358510f,
        -0.0301581072f,
         0.1669790517f,
         0.9870942864f,
        -0.1862939416f,
         0.0922040267f,
        -0.0447125347f,
         0.0156117223f
    },
    {
         0.0052057308f,         /* Filter 8 */
         0.0179050490f,
        -0.0802880559f,
         0.3122713836f,
         0.8910868760f,
        -0.2043906784f,
         0.0931039664f,
        -0.0394811291f,
         0.0097229236f
    },
    {
        -0.0059810465f,         /* Filter 9 */
         0.0424189243f,
        -0.1291107519f,
         0.4670580725f,
         0.7664229820f,
        -0.1974255221f,
         0.0832744718f,
        -0.0300884145f,
         0.0031876922f
    },
};
#endif

static int fake_get_bit(void *user_data)
{
    return 1;
}
/*- End of function --------------------------------------------------------*/

static __inline__ int get_scrambled_bit(v29_tx_state_t *s)
{
    int bit;
    int out_bit;

    if ((bit = s->current_get_bit(s->user_data)) == PUTBIT_END_OF_DATA)
    {
        /* End of real data. Switch to the fake get_bit routine, until we
           have shut down completely. */
        s->current_get_bit = fake_get_bit;
        s->in_training = TRUE;
        bit = 1;
    }
    out_bit = (bit ^ (s->scramble_reg >> 17) ^ (s->scramble_reg >> 22)) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | out_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINT)
static __inline__ complexi16_t getbaud(v29_tx_state_t *s)
#else
static __inline__ complexf_t getbaud(v29_tx_state_t *s)
#endif
{
    static const int phase_steps_9600[8] =
    {
        1, 0, 2, 3, 6, 7, 5, 4
    };
    static const int phase_steps_4800[4] =
    {
        0, 2, 6, 4
    };
#if defined(SPANDSP_USE_FIXED_POINT)
    static const complexi16_t constellation[16] =
    {
        { 3,  0},           /*   0deg low  */
        { 1,  1},           /*  45deg low  */
        { 0,  3},           /*  90deg low  */
        {-1,  1},           /* 135deg low  */
        {-3,  0},           /* 180deg low  */
        {-1, -1},           /* 225deg low  */
        { 0, -3},           /* 270deg low  */
        { 1, -1},           /* 315deg low  */
        { 5,  0},           /*   0deg high */
        { 3,  3},           /*  45deg high */
        { 0,  5},           /*  90deg high */
        {-3,  3},           /* 135deg high */
        {-5,  0},           /* 180deg high */
        {-3, -3},           /* 225deg high */
        { 0, -5},           /* 270deg high */
        { 3, -3}            /* 315deg high */
    };
    static const complexi16_t abab[6] =
    {
        { 3, -3},           /* 315deg high 9600 */
        {-3,  0},           /* 180deg low       */
        { 1, -1},           /* 315deg low 7200  */
        {-3,  0},           /* 180deg low       */
        { 0, -3},           /* 270deg low 4800  */
        {-3,  0}            /* 180deg low       */
    };
    static const complexi16_t cdcd[6] =
    {
        { 3,  0},           /*   0deg low 9600  */
        {-3,  3},           /* 135deg high      */
        { 3,  0},           /*   0deg low 7200  */
        {-1,  1},           /* 135deg low       */
        { 3,  0},           /*   0deg low 4800  */
        { 0,  3}            /*  90deg low       */
    };
#else
    static const complexf_t constellation[16] =
    {
        { 3.0f,  0.0f},     /*   0deg low  */
        { 1.0f,  1.0f},     /*  45deg low  */
        { 0.0f,  3.0f},     /*  90deg low  */
        {-1.0f,  1.0f},     /* 135deg low  */
        {-3.0f,  0.0f},     /* 180deg low  */
        {-1.0f, -1.0f},     /* 225deg low  */
        { 0.0f, -3.0f},     /* 270deg low  */
        { 1.0f, -1.0f},     /* 315deg low  */
        { 5.0f,  0.0f},     /*   0deg high */
        { 3.0f,  3.0f},     /*  45deg high */
        { 0.0f,  5.0f},     /*  90deg high */
        {-3.0f,  3.0f},     /* 135deg high */
        {-5.0f,  0.0f},     /* 180deg high */
        {-3.0f, -3.0f},     /* 225deg high */
        { 0.0f, -5.0f},     /* 270deg high */
        { 3.0f, -3.0f}      /* 315deg high */
    };
    static const complexf_t abab[6] =
    {
        { 3.0f, -3.0f},     /* 315deg high 9600 */
        {-3.0f,  0.0f},     /* 180deg low       */
        { 1.0f, -1.0f},     /* 315deg low 7200  */
        {-3.0f,  0.0f},     /* 180deg low       */
        { 0.0f, -3.0f},     /* 270deg low 4800  */
        {-3.0f,  0.0f}      /* 180deg low       */
    };
    static const complexf_t cdcd[6] =
    {
        { 3.0f,  0.0f},     /*   0deg low 9600  */
        {-3.0f,  3.0f},     /* 135deg high      */
        { 3.0f,  0.0f},     /*   0deg low 7200  */
        {-1.0f,  1.0f},     /* 135deg low       */
        { 3.0f,  0.0f},     /*   0deg low 4800  */
        { 0.0f,  3.0f}      /*  90deg low       */
    };
#endif
    int bits;
    int amp;
    int bit;

    if (s->in_training)
    {
        /* Send the training sequence */
        if (++s->training_step <= V29_TRAINING_SEG_4)
        {
            if (s->training_step <= V29_TRAINING_SEG_3)
            {
                if (s->training_step <= V29_TRAINING_SEG_1)
                {
                    /* Optional segment: Unmodulated carrier (talker echo protection) */
                    return constellation[0];
                }
                if (s->training_step <= V29_TRAINING_SEG_2)
                {
                    /* Segment 1: silence */
#if defined(SPANDSP_USE_FIXED_POINT)
                    return complex_seti16(0, 0);
#else
                    return complex_setf(0.0f, 0.0f);
#endif
                }
                /* Segment 2: ABAB... */
                return abab[(s->training_step & 1) + s->training_offset];
            }
            /* Segment 3: CDCD... */
            /* Apply the 1 + x^-6 + x^-7 training scrambler */
            bit = s->training_scramble_reg & 1;
            s->training_scramble_reg >>= 1;
            s->training_scramble_reg |= (((bit ^ s->training_scramble_reg) & 1) << 6);
            return cdcd[bit + s->training_offset];
        }
        /* We should be in the block of test ones, or shutdown ones, if we get here. */
        /* There is no graceful shutdown procedure defined for V.29. Just
           send some ones, to ensure we get the real data bits through, even
           with bad ISI. */
        if (s->training_step == V29_TRAINING_END + 1)
        {
            /* Switch from the fake get_bit routine, to the user supplied real
               one, and we are up and running. */
            s->current_get_bit = s->get_bit;
            s->in_training = FALSE;
        }
    }
    /* 9600bps uses the full constellation.
       7200bps uses only the first half of the full constellation.
       4800bps uses the smaller constellation. */
    amp = 0;
    /* We only use an amplitude bit at 9600bps */
    if (s->bit_rate == 9600  &&  get_scrambled_bit(s))
        amp = 8;
    /*endif*/
    bits = get_scrambled_bit(s);
    bits = (bits << 1) | get_scrambled_bit(s);
    if (s->bit_rate == 4800)
    {
        bits = phase_steps_4800[bits];
    }
    else
    {
        bits = (bits << 1) | get_scrambled_bit(s);
        bits = phase_steps_9600[bits];
    }
    s->constellation_state = (s->constellation_state + bits) & 7;
    return constellation[amp | s->constellation_state];
}
/*- End of function --------------------------------------------------------*/

int v29_tx(v29_tx_state_t *s, int16_t amp[], int len)
{
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi_t x;
    complexi_t z;
#else
    complexf_t x;
    complexf_t z;
#endif
    int i;
    int sample;

    if (s->training_step >= V29_TRAINING_SHUTDOWN_END)
    {
        /* Once we have sent the shutdown symbols, we stop sending completely. */
        return 0;
    }
    for (sample = 0;  sample < len;  sample++)
    {
        if ((s->baud_phase += 3) >= 10)
        {
            s->baud_phase -= 10;
            s->rrc_filter[s->rrc_filter_step] =
            s->rrc_filter[s->rrc_filter_step + V29_TX_FILTER_STEPS] = getbaud(s);
            if (++s->rrc_filter_step >= V29_TX_FILTER_STEPS)
                s->rrc_filter_step = 0;
        }
        /* Root raised cosine pulse shaping at baseband */
#if defined(SPANDSP_USE_FIXED_POINT)
        x = complex_seti(0, 0);
        for (i = 0;  i < V29_TX_FILTER_STEPS;  i++)
        {
            x.re += (int32_t) pulseshaper[9 - s->baud_phase][i]*(int32_t) s->rrc_filter[i + s->rrc_filter_step].re;
            x.im += (int32_t) pulseshaper[9 - s->baud_phase][i]*(int32_t) s->rrc_filter[i + s->rrc_filter_step].im;
        }
        /* Now create and modulate the carrier */
        x.re >>= 4;
        x.im >>= 4;
        z = dds_complexi(&(s->carrier_phase), s->carrier_phase_rate);
        /* Don't bother saturating. We should never clip. */
        i = (x.re*z.re - x.im*z.im) >> 15;
        amp[sample] = (int16_t) ((i*s->gain) >> 15);
#else
        x = complex_setf(0.0f, 0.0f);
        for (i = 0;  i < V29_TX_FILTER_STEPS;  i++)
        {
            x.re += pulseshaper[9 - s->baud_phase][i]*s->rrc_filter[i + s->rrc_filter_step].re;
            x.im += pulseshaper[9 - s->baud_phase][i]*s->rrc_filter[i + s->rrc_filter_step].im;
        }
        /* Now create and modulate the carrier */
        z = dds_complexf(&(s->carrier_phase), s->carrier_phase_rate);
        /* Don't bother saturating. We should never clip. */
        amp[sample] = (int16_t) rintf((x.re*z.re - x.im*z.im)*s->gain);
#endif
    }
    return sample;
}
/*- End of function --------------------------------------------------------*/

static void set_working_gain(v29_tx_state_t *s)
{
#if defined(SPANDSP_USE_FIXED_POINT)
    switch (s->bit_rate)
    {
    case 9600:
        s->gain = 0.387f*s->base_gain*16.0f*32767.0f/30672.52f;
        break;
    case 7200:
        s->gain = 0.605f*s->base_gain*16.0f*32767.0f/30672.52f;
        break;
    case 4800:
        s->gain = 0.470f*s->base_gain*16.0f*32767.0f/30672.52f;
        break;
    default:
        break;
    }
#else
    switch (s->bit_rate)
    {
    case 9600:
        s->gain = 0.387f*s->base_gain;
        break;
    case 7200:
        s->gain = 0.605f*s->base_gain;
        break;
    case 4800:
        s->gain = 0.470f*s->base_gain;
        break;
    default:
        break;
    }
#endif
}
/*- End of function --------------------------------------------------------*/

void v29_tx_power(v29_tx_state_t *s, float power)
{
    /* The constellation does not maintain constant average power as we change bit rates.
       We need to scale the gain we get here by a bit rate specific scaling factor each
       time we restart the modem. */
    s->base_gain = powf(10.0f, (power - DBM0_MAX_POWER)/20.0f)*32768.0f/PULSESHAPER_GAIN;
    set_working_gain(s);
}
/*- End of function --------------------------------------------------------*/

void v29_tx_set_get_bit(v29_tx_state_t *s, get_bit_func_t get_bit, void *user_data)
{
    if (s->get_bit == s->current_get_bit)
        s->current_get_bit = get_bit;
    s->get_bit = get_bit;
    s->user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

int v29_tx_restart(v29_tx_state_t *s, int rate, int tep)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Restarting V.29\n");
    s->bit_rate = rate;
    set_working_gain(s);
    switch (s->bit_rate)
    {
    case 9600:
        s->training_offset = 0;
        break;
    case 7200:
        s->training_offset = 2;
        break;
    case 4800:
        s->training_offset = 4;
        break;
    default:
        return -1;
    }
#if defined(SPANDSP_USE_FIXED_POINT)
    memset(s->rrc_filter, 0, sizeof(s->rrc_filter));
#else
    cvec_zerof(s->rrc_filter, sizeof(s->rrc_filter)/sizeof(s->rrc_filter[0]));
#endif
    s->rrc_filter_step = 0;
    s->scramble_reg = 0;
    s->training_scramble_reg = 0x2A;
    s->in_training = TRUE;
    s->training_step = (tep)  ?  V29_TRAINING_SEG_TEP  :  V29_TRAINING_SEG_1;
    s->carrier_phase = 0;
    s->baud_phase = 0;
    s->constellation_state = 0;
    s->current_get_bit = fake_get_bit;
    return 0;
}
/*- End of function --------------------------------------------------------*/

v29_tx_state_t *v29_tx_init(v29_tx_state_t *s, int rate, int tep, get_bit_func_t get_bit, void *user_data)
{
    if (s == NULL)
    {
        if ((s = (v29_tx_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));
    span_log_init(&s->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&s->logging, "V.29 TX");
    s->get_bit = get_bit;
    s->user_data = user_data;
    s->carrier_phase_rate = dds_phase_ratef(CARRIER_NOMINAL_FREQ);
    v29_tx_power(s, -14.0f);
    v29_tx_restart(s, rate, tep);
    return s;
}
/*- End of function --------------------------------------------------------*/

int v29_tx_free(v29_tx_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
