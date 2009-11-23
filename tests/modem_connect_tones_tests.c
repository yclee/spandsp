/*
 * SpanDSP - a series of DSP components for telephony
 *
 * modem_connect_tones_tests.c
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2006 Steve Underwood
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
 * $Id: modem_connect_tones_tests.c,v 1.12 2007/11/10 11:14:58 steveu Exp $
 */

/*! \page modem_connect_tones_tests_page Modem connect tones tests
\section modem_connect_tones_rx_tests_page_sec_1 What does it do?
These tests...
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <audiofile.h>

#include "spandsp.h"

#define OUTPUT_FILE_NAME    "modem_connect_tones.wav"

#define MITEL_DIR           "../itutests/mitel/"
#define BELLCORE_DIR        "../itutests/bellcore/"

#define FALSE 0
#define TRUE (!FALSE)

const char *bellcore_files[] =
{
    MITEL_DIR    "mitel-cm7291-talkoff.wav",
    BELLCORE_DIR "tr-tsy-00763-1.wav",
    BELLCORE_DIR "tr-tsy-00763-2.wav",
    BELLCORE_DIR "tr-tsy-00763-3.wav",
    BELLCORE_DIR "tr-tsy-00763-4.wav",
    BELLCORE_DIR "tr-tsy-00763-5.wav",
    BELLCORE_DIR "tr-tsy-00763-6.wav",
    ""
};

#define PERFORM_TEST_1A         (1 << 1)
#define PERFORM_TEST_1B         (1 << 2)
#define PERFORM_TEST_1C         (1 << 3)
#define PERFORM_TEST_1D         (1 << 4)
#define PERFORM_TEST_2A         (1 << 5)
#define PERFORM_TEST_2B         (1 << 6)
#define PERFORM_TEST_2C         (1 << 7)
#define PERFORM_TEST_3A         (1 << 8)
#define PERFORM_TEST_3B         (1 << 9)
#define PERFORM_TEST_4          (1 << 10)
#define PERFORM_TEST_5          (1 << 11)

int preamble_count = 0;
int preamble_on_at = -1;
int preamble_off_at = -1;

static int preamble_get_bit(void *user_data)
{
    static int bit = 0;
    
    /* Generate a section of 101010... preamble, with a scattering of bit errors.
       Then generate some random bits, which should not look like preamble. */
    if (++preamble_count < 255)
    {
        bit ^= 1;
#if 1
        /* Inject some bad bits */
        if (rand()%15 == 0)
            return bit ^ 1;
#endif
    }
    else
    {
        bit = rand() & 1;
    }
    return bit;
}
/*- End of function --------------------------------------------------------*/

static void tone_detected(void *user_data, int on, int level, int delay)
{
    printf("Preamble declared %s at bit %d\n", (on)  ?  "on"  :  "off", preamble_count);
    if (on)
        preamble_on_at = preamble_count;
    else
        preamble_off_at = preamble_count;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

int main(int argc, char *argv[])
{
    int i;
    int j;
    int pitch;
    int level;
    int16_t amp[8000];
    modem_connect_tones_rx_state_t cng_rx;
    modem_connect_tones_rx_state_t ced_rx;
    modem_connect_tones_rx_state_t ec_dis_rx;
    modem_connect_tones_tx_state_t ec_dis_tx;
    awgn_state_t chan_noise_source;
    int hits;
    AFfilehandle inhandle;
    AFfilehandle outhandle;
    AFfilesetup filesetup;
    int outframes;
    int frames;
    int samples;
    int hit;
    int false_hit;
    int false_miss;
    float x;
    tone_gen_descriptor_t tone_desc;
    tone_gen_state_t tone_tx;
    power_meter_t power_state;
    int power;
    int max_power;
    int level2;
    int max_level2;
    int test_list;
    int when;
    fsk_tx_state_t preamble_tx;

    test_list = 0;
    for (i = 1;  i < argc;  i++)
    {
        if (strcasecmp(argv[i], "1a") == 0)
            test_list |= PERFORM_TEST_1A;
        else if (strcasecmp(argv[i], "1b") == 0)
            test_list |= PERFORM_TEST_1B;
        else if (strcasecmp(argv[i], "1c") == 0)
            test_list |= PERFORM_TEST_1C;
        else if (strcasecmp(argv[i], "1d") == 0)
            test_list |= PERFORM_TEST_1D;
        else if (strcasecmp(argv[i], "2a") == 0)
            test_list |= PERFORM_TEST_2A;
        else if (strcasecmp(argv[i], "2b") == 0)
            test_list |= PERFORM_TEST_2B;
        else if (strcasecmp(argv[i], "2c") == 0)
            test_list |= PERFORM_TEST_2C;
        else if (strcasecmp(argv[i], "3a") == 0)
            test_list |= PERFORM_TEST_3A;
        else if (strcasecmp(argv[i], "3b") == 0)
            test_list |= PERFORM_TEST_3B;
        else if (strcasecmp(argv[i], "4") == 0)
            test_list |= PERFORM_TEST_4;
        else if (strcasecmp(argv[i], "5") == 0)
            test_list |= PERFORM_TEST_5;
        else
        {
            fprintf(stderr, "Unknown test '%s' specified\n", argv[i]);
            exit(2);
        }
    }
    if (test_list == 0)
        test_list = 0xFFFFFFFF;

    if ((filesetup = afNewFileSetup()) == AF_NULL_FILESETUP)
    {
        fprintf(stderr, "    Failed to create file setup\n");
        exit(2);
    }
    afInitSampleFormat(filesetup, AF_DEFAULT_TRACK, AF_SAMPFMT_TWOSCOMP, 16);
    afInitRate(filesetup, AF_DEFAULT_TRACK, (float) SAMPLE_RATE);
    afInitFileFormat(filesetup, AF_FILE_WAVE);
    afInitChannels(filesetup, AF_DEFAULT_TRACK, 1);

    if ((outhandle = afOpenFile(OUTPUT_FILE_NAME, "w", filesetup)) == AF_NULL_FILEHANDLE)
    {
        fprintf(stderr, "    Cannot create wave file '%s'\n", OUTPUT_FILE_NAME);
        exit(2);
    }

    if ((test_list & PERFORM_TEST_1A))
    {
        printf("Test 1a: CNG generation to a file\n");
        modem_connect_tones_tx_init(&ec_dis_tx, MODEM_CONNECT_TONES_FAX_CNG);
        for (i = 0;  i < 1000;  i++)
        {
            samples = modem_connect_tones_tx(&ec_dis_tx, amp, 160);
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      samples);
            if (outframes != samples)
            {
                fprintf(stderr, "    Error writing wave file\n");
                exit(2);
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endif*/
    
    if ((test_list & PERFORM_TEST_1B))
    {
        printf("Test 1b: CED generation to a file\n");
        modem_connect_tones_tx_init(&ec_dis_tx, MODEM_CONNECT_TONES_FAX_CED);
        for (i = 0;  i < 1000;  i++)
        {
            samples = modem_connect_tones_tx(&ec_dis_tx, amp, 160);
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      samples);
            if (outframes != samples)
            {
                fprintf(stderr, "    Error writing wave file\n");
                exit(2);
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_1C))
    {
        printf("Test 1c: Modulated EC-disable generation to a file\n");
        /* Some with modulation */
        modem_connect_tones_tx_init(&ec_dis_tx, MODEM_CONNECT_TONES_EC_DISABLE_MOD);
        for (i = 0;  i < 1000;  i++)
        {
            samples = modem_connect_tones_tx(&ec_dis_tx, amp, 160);
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      samples);
            if (outframes != samples)
            {
                fprintf(stderr, "    Error writing wave file\n");
                exit(2);
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_1D))
    {
        printf("Test 1d: EC-disable generation to a file\n");
        /* Some without modulation */
        modem_connect_tones_tx_init(&ec_dis_tx, MODEM_CONNECT_TONES_EC_DISABLE);
        for (i = 0;  i < 1000;  i++)
        {
            samples = modem_connect_tones_tx(&ec_dis_tx, amp, 160);
            outframes = afWriteFrames(outhandle,
                                      AF_DEFAULT_TRACK,
                                      amp,
                                      samples);
            if (outframes != samples)
            {
                fprintf(stderr, "    Error writing wave file\n");
                exit(2);
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endif*/
    
    if (afCloseFile(outhandle) != 0)
    {
        printf("    Cannot close wave file '%s'\n", OUTPUT_FILE_NAME);
        exit(2);
    }
    /*endif*/
    
    if ((test_list & PERFORM_TEST_2A))
    {
        printf("Test 2a: CNG detection with frequency\n");
        awgn_init_dbm0(&chan_noise_source, 7162534, -50.0f);
        false_hit = FALSE;
        false_miss = FALSE;
        for (pitch = 600;  pitch < 1600;  pitch++)
        {
            make_tone_gen_descriptor(&tone_desc,
                                     pitch,
                                     -11,
                                     0,
                                     0,
                                     425,
                                     3000,
                                     0,
                                     0,
                                     TRUE);
            tone_gen_init(&tone_tx, &tone_desc);

            modem_connect_tones_rx_init(&cng_rx, MODEM_CONNECT_TONES_FAX_CNG, NULL, NULL);
            power_meter_init(&power_state, 5);
            power = 0;
            max_power = 0;
            level2 = 0;
            max_level2 = 0;
            for (i = 0;  i < 500;  i++)
            {
                samples = tone_gen(&tone_tx, amp, 160);
                for (j = 0;  j < samples;  j++)
                {
                    amp[j] += awgn(&chan_noise_source);
                    power = power_meter_update(&power_state, amp[j]);
                    if (power > max_power)
                        max_power = power;
                    /*endif*/
                    level2 += ((abs(amp[j]) - level2) >> 5);
                    if (level2 > max_level2)
                        max_level2 = level2;
                }
                /*endfor*/
                modem_connect_tones_rx(&cng_rx, amp, samples);
            }
            /*endfor*/
//printf("max power is %d %f\n", max_power, log10f((float) max_power/(32767.0f*32767.0f))*10.0f + DBM0_MAX_POWER);
//printf("level2 %d (%f)\n", max_level2, log10f((float) max_level2/32768.0f)*20.0f + DBM0_MAX_POWER);
            hit = modem_connect_tones_rx_get(&cng_rx);
            if (pitch < (1100 - 70)  ||  pitch > (1100 + 70))
            {
                if (hit)
                    false_hit = TRUE;
                /*endif*/
            }
            else if (pitch > (1100 - 50)  &&  pitch < (1100 + 50))
            {
                if (!hit)
                    false_miss = TRUE;
                /*endif*/
            }
            /*endif*/
            if (hit)
                printf("Detected at %5dHz %12d %12d %d\n", pitch, cng_rx.channel_level, cng_rx.notch_level, hit);
            /*endif*/
        }
        /*endfor*/
        if (false_hit  ||  false_miss)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/
    
    if ((test_list & PERFORM_TEST_2B))
    {
        printf("Test 2b: CED detection with frequency\n");
        awgn_init_dbm0(&chan_noise_source, 7162534, -50.0f);
        false_hit = FALSE;
        false_miss = FALSE;
        for (pitch = 1600;  pitch < 2600;  pitch++)
        {
            make_tone_gen_descriptor(&tone_desc,
                                     pitch,
                                     -11,
                                     0,
                                     0,
                                     2600,
                                     0,
                                     0,
                                     0,
                                     FALSE);
            tone_gen_init(&tone_tx, &tone_desc);

            modem_connect_tones_rx_init(&ced_rx, MODEM_CONNECT_TONES_FAX_CED, NULL, NULL);
            for (i = 0;  i < 500;  i++)
            {
                samples = tone_gen(&tone_tx, amp, 160);
                for (j = 0;  j < samples;  j++)
                    amp[j] += awgn(&chan_noise_source);
                /*endfor*/
                modem_connect_tones_rx(&ced_rx, amp, samples);
            }
            /*endfor*/
            hit = modem_connect_tones_rx_get(&ced_rx);
            if (pitch < (2100 - 70)  ||  pitch > (2100 + 70))
            {
                if (hit)
                    false_hit = TRUE;
            }
            else if (pitch > (2100 - 50)  &&  pitch < (2100 + 50))
            {
                if (!hit)
                    false_miss = TRUE;
            }
            /*endif*/
            if (hit)
                printf("Detected at %5dHz %12d %12d %d\n", pitch, ced_rx.channel_level, ced_rx.notch_level, hit);
            /*endif*/
        }
        if (false_hit  ||  false_miss)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_2C))
    {
        printf("Test 2c: EC disable detection with frequency\n");
        awgn_init_dbm0(&chan_noise_source, 7162534, -50.0f);
        false_hit = FALSE;
        false_miss = FALSE;
        for (pitch = 2000;  pitch < 2200;  pitch++)
        {
            /* Use the transmitter to test the receiver */
            modem_connect_tones_tx_init(&ec_dis_tx, MODEM_CONNECT_TONES_EC_DISABLE);
            /* Fudge things for the test */
            ec_dis_tx.tone_phase_rate = dds_phase_rate(pitch);
            ec_dis_tx.level = dds_scaling_dbm0(-25);
            modem_connect_tones_rx_init(&ec_dis_rx, MODEM_CONNECT_TONES_EC_DISABLE, NULL, NULL);
            for (i = 0;  i < 500;  i++)
            {
                samples = modem_connect_tones_tx(&ec_dis_tx, amp, 160);
                for (j = 0;  j < samples;  j++)
                    amp[j] += awgn(&chan_noise_source);
                /*endfor*/
                modem_connect_tones_rx(&ec_dis_rx, amp, samples);
            }
            /*endfor*/
            hit = modem_connect_tones_rx_get(&ec_dis_rx);
            if (pitch < (2100 - 70)  ||  pitch > (2100 + 70))
            {
                if (hit)
                    false_hit = TRUE;
                /*endif*/
            }
            else if (pitch > (2100 - 50)  &&  pitch < (2100 + 50))
            {
                if (!hit)
                    false_miss = TRUE;
                /*endif*/
            }
            /*endif*/
            if (hit)
                printf("Detected at %5dHz %12d %12d %d\n", pitch, ec_dis_rx.channel_level, ec_dis_rx.notch_level, hit);
            /*endif*/
        }
        /*endfor*/
        if (false_hit  ||  false_miss)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_3A))
    {
        printf("Test 3a: CNG detection with level\n");
        awgn_init_dbm0(&chan_noise_source, 7162534, -60.0f);
        false_hit = FALSE;
        false_miss = FALSE;
        for (pitch = 1062;  pitch <= 1138;  pitch += 2*38)
        {
            for (level = 0;  level >= -43;  level--)
            {
                make_tone_gen_descriptor(&tone_desc,
                                         pitch,
                                         level,
                                         0,
                                         0,
                                         500,
                                         3000,
                                         0,
                                         0,
                                         TRUE);
                tone_gen_init(&tone_tx, &tone_desc);

                modem_connect_tones_rx_init(&cng_rx, MODEM_CONNECT_TONES_FAX_CNG, NULL, NULL);
                for (i = 0;  i < 500;  i++)
                {
                    samples = tone_gen(&tone_tx, amp, 160);
                    for (j = 0;  j < samples;  j++)
                        amp[j] += awgn(&chan_noise_source);
                    /*endfor*/
                    modem_connect_tones_rx(&cng_rx, amp, samples);
                }
                /*endfor*/
                hit = modem_connect_tones_rx_get(&cng_rx);
                if (level < -43)
                {
                    if (hit)
                        false_hit = TRUE;
                    /*endif*/
                }
                else if (level > -43)
                {
                    if (!hit)
                        false_miss = TRUE;
                    /*endif*/
                }
                /*endif*/
                if (hit)
                    printf("Detected at %5dHz %ddB %12d %12d %d\n", pitch, level, cng_rx.channel_level, cng_rx.notch_level, hit);
                /*endif*/
            }
            /*endfor*/
        }
        /*endfor*/
        if (false_hit  ||  false_miss)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_3B))
    {
        printf("Test 3b: CED detection with level\n");
        awgn_init_dbm0(&chan_noise_source, 7162534, -60.0f);
        false_hit = FALSE;
        false_miss = FALSE;
        for (pitch = 2062;  pitch <= 2138;  pitch += 2*38)
        {
            for (level = 0;  level >= -43;  level--)
            {
                make_tone_gen_descriptor(&tone_desc,
                                         pitch,
                                         level,
                                         0,
                                         0,
                                         2600,
                                         0,
                                         0,
                                         0,
                                         FALSE);
                tone_gen_init(&tone_tx, &tone_desc);
                modem_connect_tones_rx_init(&ced_rx, MODEM_CONNECT_TONES_FAX_CED, NULL, NULL);
                for (i = 0;  i < 500;  i++)
                {
                    samples = tone_gen(&tone_tx, amp, 160);
                    for (j = 0;  j < samples;  j++)
                        amp[j] += awgn(&chan_noise_source);
                    /*endfor*/
                    modem_connect_tones_rx(&ced_rx, amp, samples);
                }
                /*endfor*/
                hit = modem_connect_tones_rx_get(&ced_rx);
                if (level < -43)
                {
                    if (hit)
                        false_hit = TRUE;
                    /*endif*/
                }
                else if (level > -43)
                {
                    if (!hit)
                        false_miss = TRUE;
                    /*endif*/
                }
                /*endif*/
                if (hit)
                    printf("Detected at %5dHz %ddB %12d %12d %d\n", pitch, level, ced_rx.channel_level, ced_rx.notch_level, hit);
                /*endif*/
            }
            /*endfor*/
        }
        /*endfor*/
        if (false_hit  ||  false_miss)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_4))
    {
        printf("Test 4: CED detection, when stimulated with V.21 preamble\n");
        false_hit = FALSE;
        false_miss = FALSE;

        /* Send 255 bits of preamble (0.85s, the minimum specified preamble for T.30), and then
           some random bits. Check the preamble detector comes on, and goes off at reasonable times. */
        fsk_tx_init(&preamble_tx, &preset_fsk_specs[FSK_V21CH2], preamble_get_bit, NULL);
        modem_connect_tones_rx_init(&ced_rx, MODEM_CONNECT_TONES_FAX_CED, tone_detected, NULL);
        for (i = 0;  i < 100;  i++)
        {
            samples = fsk_tx(&preamble_tx, amp, 160);
            modem_connect_tones_rx(&ced_rx, amp, samples);
        }
        /*endfor*/
        if (preamble_on_at < 40  ||  preamble_on_at > 80
            ||
            preamble_off_at < (255 + 40)  ||  preamble_off_at > (255 + 80))
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    if ((test_list & PERFORM_TEST_5))
    {
        /* Talk-off test */
        /* Here we use the BellCore talk off test tapes, intended for DTMF detector
           testing. Presumably they should also have value here, but I am not sure.
           If those voice snippets were chosen to be tough on DTMF detectors, they
           might go easy on detectors looking for different pitches. However, the
           Mitel DTMF test tape is known (the hard way) to exercise 2280Hz tone
           detectors quite well. */
        printf("Test 5: Talk-off test\n");
        modem_connect_tones_rx_init(&cng_rx, MODEM_CONNECT_TONES_FAX_CNG, NULL, NULL);
        modem_connect_tones_rx_init(&ced_rx, MODEM_CONNECT_TONES_FAX_CED, NULL, NULL);
        modem_connect_tones_rx_init(&ec_dis_rx, MODEM_CONNECT_TONES_EC_DISABLE, NULL, NULL);
        hits = 0;
        for (j = 0;  bellcore_files[j][0];  j++)
        {
            if ((inhandle = afOpenFile(bellcore_files[j], "r", 0)) == AF_NULL_FILEHANDLE)
            {
                fprintf(stderr, "    Cannot open speech file '%s'\n", bellcore_files[j]);
                exit (2);
            }
            /*endif*/
            if ((x = afGetFrameSize(inhandle, AF_DEFAULT_TRACK, 1)) != 2.0)
            {
                fprintf(stderr, "    Unexpected frame size in speech file '%s'\n", bellcore_files[j]);
                exit (2);
            }
            /*endif*/
            if ((x = afGetRate(inhandle, AF_DEFAULT_TRACK)) != (float) SAMPLE_RATE)
            {
                fprintf(stderr, "    Unexpected sample rate in speech file '%s'\n", bellcore_files[j]);
                exit(2);
            }
            /*endif*/
            if ((x = afGetChannels(inhandle, AF_DEFAULT_TRACK)) != 1.0)
            {
                fprintf(stderr, "    Unexpected number of channels in speech file '%s'\n", bellcore_files[j]);
                exit(2);
            }
            /*endif*/

            when = 0;
            hits = 0;
            while ((frames = afReadFrames(inhandle, AF_DEFAULT_TRACK, amp, 8000)))
            {
                when++;
                modem_connect_tones_rx(&cng_rx, amp, frames);
                modem_connect_tones_rx(&ced_rx, amp, frames);
                modem_connect_tones_rx(&ec_dis_rx, amp, frames);
                if (modem_connect_tones_rx_get(&cng_rx))
                {
                    /* This is not a true measure of hits, as there might be more
                       than one in a block of data. However, since the only good
                       result is no hits, this approximation is OK. */
                    printf("Hit CNG at %ds\n", when);
                    hits++;
                    modem_connect_tones_rx_init(&cng_rx, MODEM_CONNECT_TONES_FAX_CNG, NULL, NULL);
                }
                /*endif*/
                if (modem_connect_tones_rx_get(&ced_rx))
                {
                    printf("Hit CED at %ds\n", when);
                    hits++;
                    modem_connect_tones_rx_init(&ced_rx, MODEM_CONNECT_TONES_FAX_CED, NULL, NULL);
                }
                /*endif*/
                if (modem_connect_tones_rx_get(&ec_dis_rx))
                {
                    printf("Hit EC disable at %ds\n", when);
                    hits++;
                    modem_connect_tones_rx_init(&ec_dis_rx, MODEM_CONNECT_TONES_EC_DISABLE, NULL, NULL);
                }
                /*endif*/
            }
            /*endwhile*/
            if (afCloseFile(inhandle) != 0)
            {
                fprintf(stderr, "    Cannot close speech file '%s'\n", bellcore_files[j]);
                exit(2);
            }
            /*endif*/
            printf("    File %d gave %d false hits.\n", j + 1, hits);
        }
        /*endfor*/
        if (hits)
        {
            printf("Test failed.\n");
            exit(2);
        }
        /*endif*/
    }
    /*endif*/

    printf("Tests passed.\n");
    return  0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
