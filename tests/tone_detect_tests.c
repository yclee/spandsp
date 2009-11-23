/*
 * SpanDSP - a series of DSP components for telephony
 *
 * tone_detect_tests.c
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2007 Steve Underwood
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
 * $Id: tone_detect_tests.c,v 1.3 2007/11/10 11:14:59 steveu Exp $
 */

/*! \page tone_detect_tests_page Tone detection tests
\section tone_detect_tests_page_sec_1 What does it do?
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <audiofile.h>

#include "spandsp.h"

#define SAMPLE_RATE 8000
#define BLOCK_LEN   56
#define PG_WINDOW   56
#define FREQ        480.0f

int main(int argc, char *argv[])
{
    int i;
    int j;
    int len;
    complexf_t coeffs[PG_WINDOW/2];
    complexf_t camp[BLOCK_LEN];
    complexf_t last_result;
    complexf_t result;
    complexf_t phase_offset;
    float freq_error;
    float scale;
    float pg_scale;
    float level;
    int32_t phase_rate;
    uint32_t phase_acc;

    phase_rate = dds_phase_ratef(FREQ - 5.0f);
    phase_acc = 0;
    len = periodogram_generate_coeffs(coeffs, FREQ, SAMPLE_RATE, PG_WINDOW);
    if (len != PG_WINDOW/2)
    {
        printf("Test failed\n");
        exit(2);
    }
    pg_scale = periodogram_generate_phase_offset(&phase_offset, FREQ, SAMPLE_RATE, PG_WINDOW);
    scale = 10000.0f;
    last_result = complex_setf(0.0f, 0.0f);
    for (i = 0;  i < 100;  i++)
    {
        for (j = 0;  j < PG_WINDOW;  j++)
        {
            result = dds_complexf(&phase_acc, phase_rate);
            camp[j].re = result.re*scale;
            camp[j].im = result.im*scale;
        }
        result = periodogram(coeffs, camp, PG_WINDOW);
        level = sqrtf(result.re*result.re + result.im*result.im);
        freq_error = periodogram_freq_error(&phase_offset, pg_scale, &last_result, &result);

        printf("Signal level = %.5f, freq error = %.5f\n", level, freq_error);
        if (level < scale - 20.0f  ||  level > scale + 20.0f)
        {
            printf("Test failed\n");
            exit(2);
        }
        if (freq_error < -5.5f  ||  freq_error > 5.5f)
        {
            printf("Test failed\n");
            exit(2);
        }
        last_result = result;
    }
    printf("Tests passed\n");
    return  0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
