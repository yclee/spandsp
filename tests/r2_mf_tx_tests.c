/*
 * SpanDSP - a series of DSP components for telephony
 *
 * r2_mf_tx_tests.c - Test the Bell MF generator.
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
 * $Id: r2_mf_tx_tests.c,v 1.7 2007/11/10 11:14:58 steveu Exp $
 */

/*! \file */

/*! \page r2_mf_tx_tests_page R2 MF generation tests
\section r2_mf_tx_tests_page_sec_1 What does it do?
???.

\section r2_mf_tx_tests_page_sec_2 How does it work?
???.
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

#define OUTPUT_FILE_NAME    "r2_mf.wav"

int main(int argc, char *argv[])
{
    r2_mf_tx_state_t gen;
    int16_t amp[1000];
    int len;
    AFfilehandle outhandle;
    AFfilesetup filesetup;
    int outframes;
    int digit;
    const char *digits = "0123456789BCDEF";

    filesetup = afNewFileSetup();
    if (filesetup == AF_NULL_FILESETUP)
    {
        fprintf(stderr, "    Failed to create file setup\n");
        exit(2);
    }
    afInitSampleFormat(filesetup, AF_DEFAULT_TRACK, AF_SAMPFMT_TWOSCOMP, 16);
    afInitRate(filesetup, AF_DEFAULT_TRACK, 8000.0);
    //afInitCompression(filesetup, AF_DEFAULT_TRACK, AF_COMPRESSION_G711_ALAW);
    afInitFileFormat(filesetup, AF_FILE_WAVE);
    afInitChannels(filesetup, AF_DEFAULT_TRACK, 1);

    outhandle = afOpenFile(OUTPUT_FILE_NAME, "w", filesetup);
    if (outhandle == AF_NULL_FILEHANDLE)
    {
        fprintf(stderr, "    Cannot open wave file '%s'\n", OUTPUT_FILE_NAME);
        exit(2);
    }

    r2_mf_tx_init(&gen, FALSE);
    for (digit = 0;  digits[digit];  digit++)
    {
        r2_mf_tx_put(&gen, digits[digit]);
        len = r2_mf_tx(&gen, amp, 1000);
        printf("Generated %d samples of %c\n", len, digits[digit]);
        if (len > 0)
        {
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      len);
        }
        r2_mf_tx_put(&gen, 0);
        len = r2_mf_tx(&gen, amp, 1000);
        printf("Generated %d samples\n", len);
        if (len > 0)
        {
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      len);
        }
    }

    r2_mf_tx_init(&gen, TRUE);
    for (digit = 0;  digits[digit];  digit++)
    {
        r2_mf_tx_put(&gen, digits[digit]);
        len = r2_mf_tx(&gen, amp, 1000);
        printf("Generated %d samples of %c\n", len, digits[digit]);
        if (len > 0)
        {
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      len);
        }
        r2_mf_tx_put(&gen, 0);
        len = r2_mf_tx(&gen, amp, 1000);
        printf("Generated %d samples\n", len);
        if (len > 0)
        {
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      len);
        }
    }

    if (afCloseFile(outhandle) != 0)
    {
        fprintf(stderr, "    Cannot close wave file '%s'\n", OUTPUT_FILE_NAME);
        exit (2);
    }
    afFreeFileSetup(filesetup);

    return  0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
