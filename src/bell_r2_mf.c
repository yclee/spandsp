/*
 * SpanDSP - a series of DSP components for telephony
 *
 * bell_r2_mf.c - Bell MF and MFC/R2 tone generation and detection.
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2001 Steve Underwood
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
 * $Id: bell_r2_mf.c,v 1.19 2007/11/30 12:20:33 steveu Exp $
 */

/*! \file bell_r2_mf.h */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif

#include "spandsp/telephony.h"
#include "spandsp/queue.h"
#include "spandsp/dc_restore.h"
#include "spandsp/complex.h"
#include "spandsp/dds.h"
#include "spandsp/tone_detect.h"
#include "spandsp/tone_generate.h"
#include "spandsp/bell_r2_mf.h"

#if !defined(M_PI)
/* C99 systems may not define M_PI */
#define M_PI 3.14159265358979323846264338327
#endif

#define ms_to_samples(t)            (((t)*SAMPLE_RATE)/1000)

typedef struct
{
    float       f1;         /* First freq */
    float       f2;         /* Second freq */
    int8_t      level1;     /* Level of the first freq (dB) */
    int8_t      level2;     /* Level of the second freq (dB) */
    uint8_t     on_time;    /* Tone on time (ms) */
    uint8_t     off_time;   /* Minimum post tone silence (ms) */
} mf_digit_tones_t;

int bell_mf_gen_inited = FALSE;
tone_gen_descriptor_t bell_mf_digit_tones[15];

int r2_mf_gen_inited = FALSE;
tone_gen_descriptor_t r2_mf_fwd_digit_tones[15];
tone_gen_descriptor_t r2_mf_back_digit_tones[15];

#if 0
tone_gen_descriptor_t socotel_mf_digit_tones[18];
#endif

/* Bell R1 tone generation specs.
 *  Power: -7dBm +- 1dB
 *  Frequency: within +-1.5%
 *  Mismatch between the start time of a pair of tones: <=6ms.
 *  Mismatch between the end time of a pair of tones: <=6ms.
 *  Tone duration: 68+-7ms, except KP which is 100+-7ms.
 *  Inter-tone gap: 68+-7ms.
 */
static const mf_digit_tones_t bell_mf_tones[] =
{
    { 700.0f,  900.0f, -7, -7,  68, 68},
    { 700.0f, 1100.0f, -7, -7,  68, 68},
    { 900.0f, 1100.0f, -7, -7,  68, 68},
    { 700.0f, 1300.0f, -7, -7,  68, 68},
    { 900.0f, 1300.0f, -7, -7,  68, 68},
    {1100.0f, 1300.0f, -7, -7,  68, 68},
    { 700.0f, 1500.0f, -7, -7,  68, 68},
    { 900.0f, 1500.0f, -7, -7,  68, 68},
    {1100.0f, 1500.0f, -7, -7,  68, 68},
    {1300.0f, 1500.0f, -7, -7,  68, 68},
    { 700.0f, 1700.0f, -7, -7,  68, 68}, /* ST''' - use 'C' */
    { 900.0f, 1700.0f, -7, -7,  68, 68}, /* ST'   - use 'A' */
    {1100.0f, 1700.0f, -7, -7, 100, 68}, /* KP    - use '*' */
    {1300.0f, 1700.0f, -7, -7,  68, 68}, /* ST''  - use 'B' */
    {1500.0f, 1700.0f, -7, -7,  68, 68}, /* ST    - use '#' */
    {0.0f, 0.0f, 0, 0, 0, 0}
};

/* The order of the digits here must match the list above */
static const char bell_mf_tone_codes[] = "1234567890CA*B#";

/* R2 tone generation specs.
 *  Power: -11.5dBm +- 1dB
 *  Frequency: within +-4Hz
 *  Mismatch between the start time of a pair of tones: <=1ms.
 *  Mismatch between the end time of a pair of tones: <=1ms.
 */
static const mf_digit_tones_t r2_mf_fwd_tones[] =
{
    {1380.0f, 1500.0f, -11, -11, 1, 0},
    {1380.0f, 1620.0f, -11, -11, 1, 0},
    {1500.0f, 1620.0f, -11, -11, 1, 0},
    {1380.0f, 1740.0f, -11, -11, 1, 0},
    {1500.0f, 1740.0f, -11, -11, 1, 0},
    {1620.0f, 1740.0f, -11, -11, 1, 0},
    {1380.0f, 1860.0f, -11, -11, 1, 0},
    {1500.0f, 1860.0f, -11, -11, 1, 0},
    {1620.0f, 1860.0f, -11, -11, 1, 0},
    {1740.0f, 1860.0f, -11, -11, 1, 0},
    {1380.0f, 1980.0f, -11, -11, 1, 0},
    {1500.0f, 1980.0f, -11, -11, 1, 0},
    {1620.0f, 1980.0f, -11, -11, 1, 0},
    {1740.0f, 1980.0f, -11, -11, 1, 0},
    {1860.0f, 1980.0f, -11, -11, 1, 0},
    {0.0f, 0.0f, 0, 0, 0, 0}
};

static const mf_digit_tones_t r2_mf_back_tones[] =
{
    {1140.0f, 1020.0f, -11, -11, 1, 0},
    {1140.0f,  900.0f, -11, -11, 1, 0},
    {1020.0f,  900.0f, -11, -11, 1, 0},
    {1140.0f,  780.0f, -11, -11, 1, 0},
    {1020.0f,  780.0f, -11, -11, 1, 0},
    { 900.0f,  780.0f, -11, -11, 1, 0},
    {1140.0f,  660.0f, -11, -11, 1, 0},
    {1020.0f,  660.0f, -11, -11, 1, 0},
    { 900.0f,  660.0f, -11, -11, 1, 0},
    { 780.0f,  660.0f, -11, -11, 1, 0},
    {1140.0f,  540.0f, -11, -11, 1, 0},
    {1020.0f,  540.0f, -11, -11, 1, 0},
    { 900.0f,  540.0f, -11, -11, 1, 0},
    { 780.0f,  540.0f, -11, -11, 1, 0},
    { 660.0f,  540.0f, -11, -11, 1, 0},
    {0.0f, 0.0f, 0, 0, 0, 0}
};

/* The order of the digits here must match the lists above */
static const char r2_mf_tone_codes[] = "1234567890BCDEF";

#if 0
static const mf_digit_tones_t socotel_tones[] =
{
    { 700.0f,  900.0f, -11, -11, 1, 0},
    { 700.0f, 1100.0f, -11, -11, 1, 0},
    { 900.0f, 1100.0f, -11, -11, 1, 0},
    { 700.0f, 1300.0f, -11, -11, 1, 0},
    { 900.0f, 1300.0f, -11, -11, 1, 0},
    {1100.0f, 1300.0f, -11, -11, 1, 0},
    { 700.0f, 1500.0f, -11, -11, 1, 0},
    { 900.0f, 1500.0f, -11, -11, 1, 0},
    {1100.0f, 1500.0f, -11, -11, 1, 0},
    {1300.0f, 1500.0f, -11, -11, 1, 0},
    {1500.0f, 1700.0f, -11, -11, 1, 0},
    { 700.0f, 1700.0f, -11, -11, 1, 0},
    { 900.0f, 1700.0f, -11, -11, 1, 0},
    {1300.0f, 1700.0f, -11, -11, 1, 0},
    {1100.0f, 1700.0f, -11, -11, 1, 0},
    {1700.0f,    0.0f, -11, -11, 1, 0},   /* Use 'F' */
    {1900.0f,    0.0f, -11, -11, 1, 0},   /* Use 'G' */
    {0.0f, 0.0f, 0, 0, 0, 0}
};

/* The order of the digits here must match the list above */
static char socotel_mf_tone_codes[] = "1234567890ABCDEFG";
#endif

#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
#define BELL_MF_THRESHOLD           (1.6e9f/65536.0f)
#define BELL_MF_TWIST               4.0f    /* 6dB */
#define BELL_MF_RELATIVE_PEAK       12.6f   /* 11dB */

#define R2_MF_THRESHOLD             (5.0e8f/4096.0f)
#define R2_MF_TWIST                 5.0f    /* 7dB */
#define R2_MF_RELATIVE_PEAK         12.6f   /* 11dB */
#else
#define BELL_MF_THRESHOLD           1.6e9f
#define BELL_MF_TWIST               4.0f    /* 6dB */
#define BELL_MF_RELATIVE_PEAK       12.6f   /* 11dB */

#define R2_MF_THRESHOLD             5.0e8f
#define R2_MF_TWIST                 5.0f    /* 7dB */
#define R2_MF_RELATIVE_PEAK         12.6f   /* 11dB */
#endif

static goertzel_descriptor_t bell_mf_detect_desc[6];

static goertzel_descriptor_t mf_fwd_detect_desc[6];
static goertzel_descriptor_t mf_back_detect_desc[6];

static const float bell_mf_frequencies[] =
{
     700.0f,  900.0f, 1100.0f, 1300.0f, 1500.0f, 1700.0f
};

/* Use the follow characters for the Bell MF special signals:
    KP    - use '*'
    ST    - use '#'
    ST'   - use 'A'
    ST''  - use 'B'
    ST''' - use 'C' */
static const char bell_mf_positions[] = "1247C-358A--69*---0B----#";

static const float r2_mf_fwd_frequencies[] =
{
    1380.0f, 1500.0f, 1620.0f, 1740.0f, 1860.0f, 1980.0f
};

static const float r2_mf_back_frequencies[] =
{
    1140.0f, 1020.0f,  900.0f,  780.0f,  660.0f,  540.0f
};

/* Use codes '1' to 'F' for the R2 signals 1 to 15, except for signal 'A'.
   Use '0' for this, so the codes match the digits 0-9. */
static const char r2_mf_positions[] = "1247B-358C--69D---0E----F";

static void bell_mf_gen_init(void)
{
    int i;
    const mf_digit_tones_t *tones;

    if (bell_mf_gen_inited)
        return;
    i = 0;
    tones = bell_mf_tones;
    while (tones->on_time)
    {
        /* Note: The duration of KP is longer than the other signals. */
        make_tone_gen_descriptor(&bell_mf_digit_tones[i++],
                                 (int) tones->f1,
                                 tones->level1,
                                 (int) tones->f2,
                                 tones->level2,
                                 tones->on_time,
                                 tones->off_time,
                                 0,
                                 0,
                                 FALSE);
        tones++;
    }
    bell_mf_gen_inited = TRUE;
}
/*- End of function --------------------------------------------------------*/

int bell_mf_tx(bell_mf_tx_state_t *s, int16_t amp[], int max_samples)
{
    int len;
    const char *cp;
    int digit;

    len = 0;
    if (s->tones.current_section >= 0)
    {
        /* Deal with the fragment left over from last time */
        len = tone_gen(&(s->tones), amp, max_samples);
    }
    while (len < max_samples  &&  (digit = queue_read_byte(&s->queue)) >= 0)
    {
        /* Step to the next digit */
        if ((cp = strchr(bell_mf_tone_codes, digit)) == NULL)
            continue;
        tone_gen_init(&(s->tones), &bell_mf_digit_tones[cp - bell_mf_tone_codes]);
        len += tone_gen(&(s->tones), amp + len, max_samples - len);
    }
    return len;
}
/*- End of function --------------------------------------------------------*/

size_t bell_mf_tx_put(bell_mf_tx_state_t *s, const char *digits, ssize_t len)
{
    size_t space;

    /* This returns the number of characters that would not fit in the buffer.
       The buffer will only be loaded if the whole string of digits will fit,
       in which case zero is returned. */
    if (len < 0)
    {
        if ((len = strlen(digits)) == 0)
            return 0;
    }
    if ((space = queue_free_space(&s->queue)) < len)
        return len - space;
    if (queue_write(&s->queue, (const uint8_t *) digits, len) >= 0)
        return 0;
    return -1;
}
/*- End of function --------------------------------------------------------*/

bell_mf_tx_state_t *bell_mf_tx_init(bell_mf_tx_state_t *s)
{
    if (s == NULL)
    {
        if ((s = (bell_mf_tx_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));

    if (!bell_mf_gen_inited)
        bell_mf_gen_init();
    tone_gen_init(&(s->tones), &bell_mf_digit_tones[0]);
    s->current_sample = 0;
    queue_init(&s->queue, MAX_BELL_MF_DIGITS, QUEUE_READ_ATOMIC | QUEUE_WRITE_ATOMIC);
    s->tones.current_section = -1;
    return s;
}
/*- End of function --------------------------------------------------------*/

int bell_mf_tx_free(bell_mf_tx_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int r2_mf_tx(r2_mf_tx_state_t *s, int16_t amp[], int samples)
{
    int len;

    if (s->digit == 0)
    {
        len = samples;
        memset(amp, 0, len*sizeof(int16_t));
    }
    else
    {
        len = tone_gen(&s->tone, amp, samples);
    }
    return len;
}
/*- End of function --------------------------------------------------------*/

int r2_mf_tx_put(r2_mf_tx_state_t *s, char digit)
{
    char *cp;

    if (digit  &&  (cp = strchr(r2_mf_tone_codes, digit)))
    {
        if (s->fwd)
            tone_gen_init(&s->tone, &r2_mf_fwd_digit_tones[cp - r2_mf_tone_codes]);
        else
            tone_gen_init(&s->tone, &r2_mf_back_digit_tones[cp - r2_mf_tone_codes]);
        s->digit = digit;
    }
    else
    {
        s->digit = 0;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

r2_mf_tx_state_t *r2_mf_tx_init(r2_mf_tx_state_t *s, int fwd)
{
    int i;
    const mf_digit_tones_t *tones;

    if (s == NULL)
    {
        if ((s = (r2_mf_tx_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));

    if (!r2_mf_gen_inited)
    {
        i = 0;
        tones = r2_mf_fwd_tones;
        while (tones->on_time)
        {
            make_tone_gen_descriptor(&r2_mf_fwd_digit_tones[i++],
                                     (int) tones->f1,
                                     tones->level1,
                                     (int) tones->f2,
                                     tones->level2,
                                     tones->on_time,
                                     tones->off_time,
                                     0,
                                     0,
                                     (tones->off_time == 0));
            tones++;
        }
        i = 0;
        tones = r2_mf_back_tones;
        while (tones->on_time)
        {
            make_tone_gen_descriptor(&r2_mf_back_digit_tones[i++],
                                     (int) tones->f1,
                                     tones->level1,
                                     (int) tones->f2,
                                     tones->level2,
                                     tones->on_time,
                                     tones->off_time,
                                     0,
                                     0,
                                     (tones->off_time == 0));
            tones++;
        }
        r2_mf_gen_inited = TRUE;
    }
    s->fwd = fwd;
    return s;
}
/*- End of function --------------------------------------------------------*/

int r2_mf_tx_free(r2_mf_tx_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int bell_mf_rx(bell_mf_rx_state_t *s, const int16_t amp[], int samples)
{
    float energy[6];
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    int32_t famp;
    int32_t v1;
#else
    float famp;
    float v1;
#endif
    int i;
    int j;
    int sample;
    int best;
    int second_best;
    int limit;
    uint8_t hit;

    hit = 0;
    for (sample = 0;  sample < samples;  sample = limit)
    {
        if ((samples - sample) >= (120 - s->current_sample))
            limit = sample + (120 - s->current_sample);
        else
            limit = samples;
        for (j = sample;  j < limit;  j++)
        {
            famp = amp[j];
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
            famp >>= 8;
            /* With GCC 2.95, the following unrolled code seems to take about 35%
               (rough estimate) as long as a neat little 0-5 loop */
            v1 = s->out[0].v2;
            s->out[0].v2 = s->out[0].v3;
            s->out[0].v3 = ((s->out[0].fac*s->out[0].v2) >> 12) - v1 + famp;
    
            v1 = s->out[1].v2;
            s->out[1].v2 = s->out[1].v3;
            s->out[1].v3 = ((s->out[1].fac*s->out[1].v2) >> 12) - v1 + famp;
    
            v1 = s->out[2].v2;
            s->out[2].v2 = s->out[2].v3;
            s->out[2].v3 = ((s->out[2].fac*s->out[2].v2) >> 12) - v1 + famp;
    
            v1 = s->out[3].v2;
            s->out[3].v2 = s->out[3].v3;
            s->out[3].v3 = ((s->out[3].fac*s->out[3].v2) >> 12) - v1 + famp;
    
            v1 = s->out[4].v2;
            s->out[4].v2 = s->out[4].v3;
            s->out[4].v3 = ((s->out[4].fac*s->out[4].v2) >> 12) - v1 + famp;
    
            v1 = s->out[5].v2;
            s->out[5].v2 = s->out[5].v3;
            s->out[5].v3 = ((s->out[5].fac*s->out[5].v2) >> 12) - v1 + famp;
#else
            /* With GCC 2.95, the following unrolled code seems to take about 35%
               (rough estimate) as long as a neat little 0-5 loop */
            v1 = s->out[0].v2;
            s->out[0].v2 = s->out[0].v3;
            s->out[0].v3 = s->out[0].fac*s->out[0].v2 - v1 + famp;
    
            v1 = s->out[1].v2;
            s->out[1].v2 = s->out[1].v3;
            s->out[1].v3 = s->out[1].fac*s->out[1].v2 - v1 + famp;
    
            v1 = s->out[2].v2;
            s->out[2].v2 = s->out[2].v3;
            s->out[2].v3 = s->out[2].fac*s->out[2].v2 - v1 + famp;
    
            v1 = s->out[3].v2;
            s->out[3].v2 = s->out[3].v3;
            s->out[3].v3 = s->out[3].fac*s->out[3].v2 - v1 + famp;
    
            v1 = s->out[4].v2;
            s->out[4].v2 = s->out[4].v3;
            s->out[4].v3 = s->out[4].fac*s->out[4].v2 - v1 + famp;
    
            v1 = s->out[5].v2;
            s->out[5].v2 = s->out[5].v3;
            s->out[5].v3 = s->out[5].fac*s->out[5].v2 - v1 + famp;
#endif
        }
        s->current_sample += (limit - sample);
        if (s->current_sample < 120)
            continue;

        /* We are at the end of an MF detection block */
        /* Find the two highest energies. The spec says to look for
           two tones and two tones only. Taking this literally -ie
           only two tones pass the minimum threshold - doesn't work
           well. The sinc function mess, due to rectangular windowing
           ensure that! Find the two highest energies and ensure they
           are considerably stronger than any of the others. */
        energy[0] = goertzel_result(&s->out[0]);
        energy[1] = goertzel_result(&s->out[1]);
        if (energy[0] > energy[1])
        {
            best = 0;
            second_best = 1;
        }
        else
        {
            best = 1;
            second_best = 0;
        }
        for (i = 2;  i < 6;  i++)
        {
            energy[i] = goertzel_result(&s->out[i]);
            if (energy[i] >= energy[best])
            {
                second_best = best;
                best = i;
            }
            else if (energy[i] >= energy[second_best])
            {
                second_best = i;
            }
        }
        /* Basic signal level and twist tests */
        hit = 0;
        if (energy[best] >= BELL_MF_THRESHOLD
            &&
            energy[second_best] >= BELL_MF_THRESHOLD
            &&
            energy[best] < energy[second_best]*BELL_MF_TWIST
            &&
            energy[best]*BELL_MF_TWIST > energy[second_best])
        {
            /* Relative peak test */
            hit = 'X';
            for (i = 0;  i < 6;  i++)
            {
                if (i != best  &&  i != second_best)
                {
                    if (energy[i]*BELL_MF_RELATIVE_PEAK >= energy[second_best])
                    {
                        /* The best two are not clearly the best */
                        hit = 0;
                        break;
                    }
                }
            }
        }
        if (hit)
        {
            /* Get the values into ascending order */
            if (second_best < best)
            {
                i = best;
                best = second_best;
                second_best = i;
            }
            best = best*5 + second_best - 1;
            hit = bell_mf_positions[best];
            /* Look for two successive similar results */
            /* The logic in the next test is:
               For KP we need 4 successive identical clean detects, with
               two blocks of something different preceeding it. For anything
               else we need two successive identical clean detects, with
               two blocks of something different preceeding it. */
            if (hit == s->hits[4]
                &&
                hit == s->hits[3]
                &&
                   ((hit != '*'  &&  hit != s->hits[2]  &&  hit != s->hits[1])
                    ||
                    (hit == '*'  &&  hit == s->hits[2]  &&  hit != s->hits[1]  &&  hit != s->hits[0])))
            {
                if (s->current_digits < MAX_BELL_MF_DIGITS)
                {
                    s->digits[s->current_digits++] = (char) hit;
                    s->digits[s->current_digits] = '\0';
                    if (s->callback)
                    {
                        s->callback(s->callback_data, s->digits, s->current_digits);
                        s->current_digits = 0;
                    }
                }
                else
                {
                    s->lost_digits++;
                }
            }
        }
        s->hits[0] = s->hits[1];
        s->hits[1] = s->hits[2];
        s->hits[2] = s->hits[3];
        s->hits[3] = s->hits[4];
        s->hits[4] = hit;
        /* Reinitialise the detector for the next block */
        for (i = 0;  i < 6;  i++)
            goertzel_reset(&s->out[i]);
        s->current_sample = 0;
    }
    if (s->current_digits  &&  s->callback)
    {
        s->callback(s->callback_data, s->digits, s->current_digits);
        s->digits[0] = '\0';
        s->current_digits = 0;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

size_t bell_mf_rx_get(bell_mf_rx_state_t *s, char *buf, int max)
{
    if (max > s->current_digits)
        max = s->current_digits;
    if (max > 0)
    {
        memcpy(buf, s->digits, max);
        memmove(s->digits, s->digits + max, s->current_digits - max);
        s->current_digits -= max;
    }
    buf[max] = '\0';
    return  max;
}
/*- End of function --------------------------------------------------------*/

bell_mf_rx_state_t *bell_mf_rx_init(bell_mf_rx_state_t *s,
                                    void (*callback)(void *user_data, const char *digits, int len),
                                    void *user_data)
{
    int i;
    static int initialised = FALSE;

    if (s == NULL)
    {
        if ((s = (bell_mf_rx_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));

    if (!initialised)
    {
        for (i = 0;  i < 6;  i++)
            make_goertzel_descriptor(&bell_mf_detect_desc[i], bell_mf_frequencies[i], 120);
        initialised = TRUE;
    }
    s->callback = callback;
    s->callback_data = user_data;

    s->hits[0] = 
    s->hits[1] =
    s->hits[2] =
    s->hits[3] = 
    s->hits[4] = 0;

    for (i = 0;  i < 6;  i++)
        goertzel_init(&s->out[i], &bell_mf_detect_desc[i]);
    s->current_sample = 0;
    s->lost_digits = 0;
    s->current_digits = 0;
    s->digits[0] = '\0';
    return s;
}
/*- End of function --------------------------------------------------------*/

int bell_mf_rx_free(bell_mf_rx_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

int r2_mf_rx(r2_mf_rx_state_t *s, const int16_t amp[], int samples)
{
    float energy[6];
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    int32_t famp;
    int32_t v1;
#else
    float famp;
    float v1;
#endif
    int i;
    int j;
    int sample;
    int best;
    int second_best;
    int hit;
    int hit_char;
    int limit;

    hit = 0;
    hit_char = 0;
    for (sample = 0;  sample < samples;  sample = limit)
    {
        if ((samples - sample) >= (s->samples - s->current_sample))
            limit = sample + (s->samples - s->current_sample);
        else
            limit = samples;
        for (j = sample;  j < limit;  j++)
        {
            famp = amp[j];
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
            famp >>= 6;
            /* With GCC 2.95, the following unrolled code seems to take about 35%
               (rough estimate) as long as a neat little 0-5 loop */
            v1 = s->out[0].v2;
            s->out[0].v2 = s->out[0].v3;
            s->out[0].v3 = ((s->out[0].fac*s->out[0].v2) >> 12) - v1 + famp;
    
            v1 = s->out[1].v2;
            s->out[1].v2 = s->out[1].v3;
            s->out[1].v3 = ((s->out[1].fac*s->out[1].v2) >> 12) - v1 + famp;
    
            v1 = s->out[2].v2;
            s->out[2].v2 = s->out[2].v3;
            s->out[2].v3 = ((s->out[2].fac*s->out[2].v2) >> 12) - v1 + famp;
    
            v1 = s->out[3].v2;
            s->out[3].v2 = s->out[3].v3;
            s->out[3].v3 = ((s->out[3].fac*s->out[3].v2) >> 12) - v1 + famp;
    
            v1 = s->out[4].v2;
            s->out[4].v2 = s->out[4].v3;
            s->out[4].v3 = ((s->out[4].fac*s->out[4].v2) >> 12) - v1 + famp;
    
            v1 = s->out[5].v2;
            s->out[5].v2 = s->out[5].v3;
            s->out[5].v3 = ((s->out[5].fac*s->out[5].v2) >> 12) - v1 + famp;
#else    
            /* With GCC 2.95, the following unrolled code seems to take about 35%
               (rough estimate) as long as a neat little 0-5 loop */
            v1 = s->out[0].v2;
            s->out[0].v2 = s->out[0].v3;
            s->out[0].v3 = s->out[0].fac*s->out[0].v2 - v1 + famp;
    
            v1 = s->out[1].v2;
            s->out[1].v2 = s->out[1].v3;
            s->out[1].v3 = s->out[1].fac*s->out[1].v2 - v1 + famp;
    
            v1 = s->out[2].v2;
            s->out[2].v2 = s->out[2].v3;
            s->out[2].v3 = s->out[2].fac*s->out[2].v2 - v1 + famp;
    
            v1 = s->out[3].v2;
            s->out[3].v2 = s->out[3].v3;
            s->out[3].v3 = s->out[3].fac*s->out[3].v2 - v1 + famp;
    
            v1 = s->out[4].v2;
            s->out[4].v2 = s->out[4].v3;
            s->out[4].v3 = s->out[4].fac*s->out[4].v2 - v1 + famp;
    
            v1 = s->out[5].v2;
            s->out[5].v2 = s->out[5].v3;
            s->out[5].v3 = s->out[5].fac*s->out[5].v2 - v1 + famp;
#endif
        }
        s->current_sample += (limit - sample);
        if (s->current_sample < s->samples)
            continue;

        /* We are at the end of an MF detection block */
        /* Find the two highest energies */
        energy[0] = goertzel_result(&s->out[0]);
        energy[1] = goertzel_result(&s->out[1]);
        if (energy[0] > energy[1])
        {
            best = 0;
            second_best = 1;
        }
        else
        {
            best = 1;
            second_best = 0;
        }
        
        for (i = 2;  i < 6;  i++)
        {
            energy[i] = goertzel_result(&s->out[i]);
            if (energy[i] >= energy[best])
            {
                second_best = best;
                best = i;
            }
            else if (energy[i] >= energy[second_best])
            {
                second_best = i;
            }
        }
        /* Basic signal level and twist tests */
        hit = FALSE;
        if (energy[best] >= R2_MF_THRESHOLD
            &&
            energy[second_best] >= R2_MF_THRESHOLD
            &&
            energy[best] < energy[second_best]*R2_MF_TWIST
            &&
            energy[best]*R2_MF_TWIST > energy[second_best])
        {
            /* Relative peak test */
            hit = TRUE;
            for (i = 0;  i < 6;  i++)
            {
                if (i != best  &&  i != second_best)
                {
                    if (energy[i]*R2_MF_RELATIVE_PEAK >= energy[second_best])
                    {
                        /* The best two are not clearly the best */
                        hit = FALSE;
                        break;
                    }
                }
            }
        }
        if (hit)
        {
            /* Get the values into ascending order */
            if (second_best < best)
            {
                i = best;
                best = second_best;
                second_best = i;
            }
            best = best*5 + second_best - 1;
            hit_char = r2_mf_positions[best];
        }
        else
        {
            hit_char = 0;
        }

        /* Reinitialise the detector for the next block */
        if (s->fwd)
        {
            for (i = 0;  i < 6;  i++)
                goertzel_reset(&s->out[i]);
        }
        else
        {
            for (i = 0;  i < 6;  i++)
                goertzel_reset(&s->out[i]);
        }
        s->current_sample = 0;
    }
    return hit_char;
}
/*- End of function --------------------------------------------------------*/

r2_mf_rx_state_t *r2_mf_rx_init(r2_mf_rx_state_t *s, int fwd)
{
    int i;
    static int initialised = FALSE;

    if (s == NULL)
    {
        if ((s = (r2_mf_rx_state_t *) malloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));

    s->fwd = fwd;

    if (!initialised)
    {
        for (i = 0;  i < 6;  i++)
        {
            make_goertzel_descriptor(&mf_fwd_detect_desc[i], r2_mf_fwd_frequencies[i], 133);
            make_goertzel_descriptor(&mf_back_detect_desc[i], r2_mf_back_frequencies[i], 133);
        }
        initialised = TRUE;
    }
    if (fwd)
    {
        for (i = 0;  i < 6;  i++)
            goertzel_init(&s->out[i], &mf_fwd_detect_desc[i]);
    }
    else
    {
        for (i = 0;  i < 6;  i++)
            goertzel_init(&s->out[i], &mf_back_detect_desc[i]);
    }
    s->samples = 133;
    s->current_sample = 0;
    return s;
}
/*- End of function --------------------------------------------------------*/

int r2_mf_rx_free(r2_mf_rx_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
