/*
 * SpanDSP - a series of DSP components for telephony
 *
 * tone_detect.c - General telephony tone detection.
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2001-2003, 2005 Steve Underwood
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
 * $Id: tone_detect.c,v 1.39 2007/09/07 13:22:25 steveu Exp $
 */
 
/*! \file tone_detect.h */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <inttypes.h>
#include <stdlib.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>

#include "spandsp/telephony.h"
#include "spandsp/complex.h"
#include "spandsp/complex_vector_float.h"
#include "spandsp/tone_detect.h"
#include "spandsp/tone_generate.h"

#if !defined(M_PI)
/* C99 systems may not define M_PI */
#define M_PI 3.14159265358979323846264338327
#endif

void make_goertzel_descriptor(goertzel_descriptor_t *t, float freq, int samples)
{
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    t->fac = 4096.0f*2.0f*cosf(2.0f*M_PI*(freq/(float) SAMPLE_RATE));
#else
    t->fac = 2.0f*cosf(2.0f*M_PI*(freq/(float) SAMPLE_RATE));
#endif
    t->samples = samples;
}
/*- End of function --------------------------------------------------------*/

goertzel_state_t *goertzel_init(goertzel_state_t *s,
                                goertzel_descriptor_t *t)
{
    if (s  ||  (s = malloc(sizeof(goertzel_state_t))))
    {
        s->v2 =
        s->v3 = 0.0;
        s->fac = t->fac;
        s->samples = t->samples;
        s->current_sample = 0;
    }
    return s;
}
/*- End of function --------------------------------------------------------*/

void goertzel_reset(goertzel_state_t *s)
{
    s->v2 =
    s->v3 = 0.0;
    s->current_sample = 0;
}
/*- End of function --------------------------------------------------------*/

int goertzel_update(goertzel_state_t *s,
                    const int16_t amp[],
                    int samples)
{
    int i;
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    int32_t v1;
#else
    float v1;
#endif

    if (samples > s->samples - s->current_sample)
        samples = s->samples - s->current_sample;
    for (i = 0;  i < samples;  i++)
    {
        v1 = s->v2;
        s->v2 = s->v3;
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
        s->v3 = ((s->fac*s->v2) >> 12) - v1 + (amp[i] >> 8);
#else
        s->v3 = s->fac*s->v2 - v1 + amp[i];
#endif
    }
    s->current_sample += samples;
    return samples;
}
/*- End of function --------------------------------------------------------*/

float goertzel_result(goertzel_state_t *s)
{
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    int32_t v1;
#else
    float v1;
#endif

    /* Push a zero through the process to finish things off. */
    v1 = s->v2;
    s->v2 = s->v3;
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    s->v3 = ((s->fac*s->v2) >> 12) - v1;
#else
    s->v3 = s->fac*s->v2 - v1;
#endif
    /* Now calculate the non-recursive side of the filter. */
    /* The result here is not scaled down to allow for the magnification
       effect of the filter (the usual DFT magnification effect). */
#if defined(SPANDSP_USE_FIXED_POINT_EXPERIMENTAL)
    return s->v3*s->v3 + s->v2*s->v2 - ((s->v2*s->v3) >> 12)*s->fac;
#else
    return s->v3*s->v3 + s->v2*s->v2 - s->v2*s->v3*s->fac;
#endif
}
/*- End of function --------------------------------------------------------*/

complexf_t periodogram(const complexf_t coeffs[], const complexf_t amp[], int len)
{
    complexf_t sum;
    complexf_t diff;
    complexf_t x;
    int i;

    x = complex_setf(0.0f, 0.0f);
    for (i = 0;  i < len/2;  i++)
    {
        sum = complex_addf(&amp[i], &amp[len - 1 - i]);
        diff = complex_subf(&amp[i], &amp[len - 1 - i]);
        x.re += (coeffs[i].re*sum.re - coeffs[i].im*diff.im);
        x.im += (coeffs[i].re*sum.im + coeffs[i].im*diff.re);
    }
    return x;
}
/*- End of function --------------------------------------------------------*/

int periodogram_prepare(complexf_t sum[], complexf_t diff[], const complexf_t amp[], int len)
{
    int i;

    for (i = 0;  i < len/2;  i++)
    {
        sum[i] = complex_addf(&amp[i], &amp[len - 1 - i]);
        diff[i] = complex_subf(&amp[i], &amp[len - 1 - i]);
    }
    return len/2;
}
/*- End of function --------------------------------------------------------*/

complexf_t periodogram_apply(const complexf_t coeffs[], const complexf_t sum[], const complexf_t diff[], int len)
{
    complexf_t x;
    int i;

    x = complex_setf(0.0f, 0.0f);
    for (i = 0;  i < len/2;  i++)
    {
        x.re += (coeffs[i].re*sum[i].re - coeffs[i].im*diff[i].im);
        x.im += (coeffs[i].re*sum[i].im + coeffs[i].im*diff[i].re);
    }
    return x;
}
/*- End of function --------------------------------------------------------*/

int periodogram_generate_coeffs(complexf_t coeffs[], float freq, int sample_rate, int window_len)
{
    float window;
    float sum;
    float x;
    int i;

    sum = 0.0f;
    for (i = 0;  i < window_len/2;  i++)
    {
        /* Apply a Hamming window as we go */
        window = 0.53836f - 0.46164f*cosf(2.0f*3.1415926535f*i/(window_len - 1.0f));
        x = (i - window_len/2.0f + 0.5f)*freq*2.0f*3.1415926535f/sample_rate;
        coeffs[i].re = cosf(x)*window;
        coeffs[i].im = -sinf(x)*window;
        sum += window;
    }
    /* Rescale for unity gain in the periodogram. The 2.0 factor is to allow for the full window,
       rather than just the half over which we have summed the coefficients. */
    sum = 1.0f/(2.0f*sum);
    for (i = 0;  i < window_len/2;  i++)
    {
        coeffs[i].re *= sum;
        coeffs[i].im *= sum;
    }
    return window_len/2;
}
/*- End of function --------------------------------------------------------*/

float periodogram_generate_phase_offset(complexf_t *offset, float freq, int sample_rate, int interval)
{
    float x;

    /* The phase offset is how far the phase rotates in one frame */
    x = 2.0f*3.1415926535f*(float) interval/(float) sample_rate;
    offset->re = cosf(freq*x);
    offset->im = sinf(freq*x);
    return 1.0f/x;
}
/*- End of function --------------------------------------------------------*/

float periodogram_freq_error(const complexf_t *phase_offset, float scale, const complexf_t *last_result, const complexf_t *result)
{
    complexf_t prediction;

    prediction = complex_mulf(last_result, phase_offset);
    return scale*(result->im*prediction.re - result->re*prediction.im)/(result->re*result->re + result->im*result->im);
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
