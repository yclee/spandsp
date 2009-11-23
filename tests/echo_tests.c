/*
 * SpanDSP - a series of DSP components for telephony
 *
 * echo_tests.c
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2001 Steve Underwood
 *
 * Based on a bit from here, a bit from there, eye of toad,
 * ear of bat, etc - plus, of course, my own 2 cents.
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
 * $Id: echo_tests.c,v 1.29 2007/11/10 11:14:58 steveu Exp $
 */

/*! \page echo_can_tests_page Line echo cancellation for voice tests

\section echo_can_tests_page_sec_1 What does it do?
The echo cancellation tests test the echo cancellor against the G.168 spec. Not
all the tests in G.168 are fully implemented at this time.

\section echo_can_tests_page_sec_2 How does it work?

\section echo_can_tests_page_sec_2 How do I use it?

*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if defined(HAVE_FL_FL_H)  &&  defined(HAVE_FL_FL_CARTESIAN_H)  &&  defined(HAVE_FL_FL_AUDIO_METER_H)
#define ENABLE_GUI
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <assert.h>
#include <audiofile.h>

#define GEN_CONST
#include <math.h>

#include "spandsp.h"
#include "spandsp/g168models.h"
#if defined(ENABLE_GUI)
#include "echo_monitor.h"
#endif

#define TEST_EC_TAPS            256

#define PERFORM_TEST_2A         (1 << 1)
#define PERFORM_TEST_2B         (1 << 2)
#define PERFORM_TEST_2C         (1 << 3)
#define PERFORM_TEST_3A         (1 << 4)
#define PERFORM_TEST_3B         (1 << 5)
#define PERFORM_TEST_3C         (1 << 6)
#define PERFORM_TEST_4          (1 << 7)
#define PERFORM_TEST_5          (1 << 8)
#define PERFORM_TEST_6          (1 << 9)
#define PERFORM_TEST_7          (1 << 10)
#define PERFORM_TEST_8          (1 << 11)
#define PERFORM_TEST_9          (1 << 12)
#define PERFORM_TEST_10A        (1 << 13)
#define PERFORM_TEST_10B        (1 << 14)
#define PERFORM_TEST_10C        (1 << 15)
#define PERFORM_TEST_11         (1 << 16)
#define PERFORM_TEST_12         (1 << 17)
#define PERFORM_TEST_13         (1 << 18)
#define PERFORM_TEST_14         (1 << 19)
#define PERFORM_TEST_15         (1 << 20)

int test_list;

#if !defined(NULL)
#define NULL (void *) 0
#endif

typedef struct
{
    const char *name;
    int max;
    int cur;
    AFfilehandle handle;
    int16_t signal[SAMPLE_RATE];
} signal_source_t;

typedef struct
{
    int type;
    fir_float_state_t *fir;
    float history[35*8];
    int pos;
    float factor; 
    float power;
} level_measurement_device_t;

signal_source_t local_css;
signal_source_t far_css;

fir32_state_t line_model;

AFfilehandle residuehandle;
int16_t residue_sound[SAMPLE_RATE];
int residue_cur = 0;

static inline void put_residue(int16_t amp)
{
    int outframes;

    residue_sound[residue_cur++] = amp;
    if (residue_cur >= SAMPLE_RATE)
    {
        outframes = afWriteFrames(residuehandle,
                                  AF_DEFAULT_TRACK,
                                  residue_sound,
                                  residue_cur);
        if (outframes != residue_cur)
        {
            fprintf(stderr, "    Error writing residue sound\n");
            exit(2);
        }
        residue_cur = 0;
    }
}
/*- End of function --------------------------------------------------------*/

static void signal_load(signal_source_t *sig, const char *name)
{
    float x;

    sig->name = name;
    if ((sig->handle = afOpenFile(sig->name, "r", 0)) == AF_NULL_FILEHANDLE)
    {
        fprintf(stderr, "    Cannot open wave file '%s'\n", sig->name);
        exit(2);
    }
    if ((x = afGetFrameSize(sig->handle, AF_DEFAULT_TRACK, 1)) != 2.0)
    {
        fprintf(stderr, "    Unexpected frame size in wave file '%s'\n", sig->name);
        exit(2);
    }
    if ((x = afGetRate(sig->handle, AF_DEFAULT_TRACK)) != (float) SAMPLE_RATE)
    {
        printf("    Unexpected sample rate in wave file '%s'\n", sig->name);
        exit(2);
    }
    if ((x = afGetChannels(sig->handle, AF_DEFAULT_TRACK)) != 1.0)
    {
        printf("    Unexpected number of channels in wave file '%s'\n", sig->name);
        exit(2);
    }
    sig->max = afReadFrames(sig->handle, AF_DEFAULT_TRACK, sig->signal, SAMPLE_RATE);
    if (sig->max < 0)
    {
        fprintf(stderr, "    Error reading sound file '%s'\n", sig->name);
        exit(2);
    }
}
/*- End of function --------------------------------------------------------*/

static void signal_free(signal_source_t *sig)
{
    if (afCloseFile(sig->handle) != 0)
    {
        fprintf(stderr, "    Cannot close sound file '%s'\n", sig->name);
        exit(2);
    }
}
/*- End of function --------------------------------------------------------*/

static void signal_restart(signal_source_t *sig)
{
    sig->cur = 0;
}
/*- End of function --------------------------------------------------------*/

static int16_t signal_amp(signal_source_t *sig)
{
    int16_t tx;

    tx = sig->signal[sig->cur++];
    if (sig->cur >= sig->max)
        sig->cur = 0;
    return tx;
}
/*- End of function --------------------------------------------------------*/

static inline int16_t codec_munge(int16_t amp)
{
    return alaw_to_linear(linear_to_alaw(amp));
}
/*- End of function --------------------------------------------------------*/

static int channel_model_create(int model)
{
    static const int32_t *line_models[] =
    {
        line_model_d2_coeffs,
        line_model_d3_coeffs,
        line_model_d4_coeffs,
        line_model_d5_coeffs,
        line_model_d6_coeffs,
        line_model_d7_coeffs,
        line_model_d8_coeffs,
        line_model_d9_coeffs
    };

    static int line_model_sizes[] =
    {
        sizeof(line_model_d2_coeffs)/sizeof(int32_t),
        sizeof(line_model_d3_coeffs)/sizeof(int32_t),
        sizeof(line_model_d4_coeffs)/sizeof(int32_t),
        sizeof(line_model_d5_coeffs)/sizeof(int32_t),
        sizeof(line_model_d6_coeffs)/sizeof(int32_t),
        sizeof(line_model_d7_coeffs)/sizeof(int32_t),
        sizeof(line_model_d8_coeffs)/sizeof(int32_t),
        sizeof(line_model_d9_coeffs)/sizeof(int32_t)
    };

    if (model < 0  ||  model >= (int) (sizeof(line_model_sizes)/sizeof(line_model_sizes[0])))
        return -1;
    fir32_create(&line_model, line_models[model], line_model_sizes[model]);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int16_t channel_model(int16_t *new_local, int16_t *new_far, int16_t local, int16_t far)
{
    int16_t echo;
    int16_t rx;

    /* Channel modelling is merely simulating the effects of A-law or u-law distortion
       and using one of the echo models from G.168. Simulating the codec is very important,
       as this is usually the limiting factor in how much echo reduction is achieved. */

    /* The local tx signal will usually have gone through an A-law munging before
       it reached the line's analogue area, where the echo occurs. */
    local = codec_munge(local);
    /* Now we need to model the echo. We only model a single analogue segment, as per
       the G.168 spec. However, there will generally be near end and far end analogue/echoey
       segments in the real world, unless an end is purely digital. */
    echo = fir32(&line_model, local/8);
    /* The far end signal will have been through an A-law munging, although
       this should not affect things. */
    rx = echo + codec_munge(far);
    /* This mixed echo and far end signal will have been through an A-law munging
       when it came back into the digital network. */
    rx = codec_munge(rx);
    if (new_far)
        *new_far = rx;
    if (new_local)
        *new_local = local;
    return  rx;
}
/*- End of function --------------------------------------------------------*/

static level_measurement_device_t *level_measurement_device_create(int type)
{
    level_measurement_device_t *dev;
    int i;

    dev = (level_measurement_device_t *) malloc(sizeof(level_measurement_device_t));
    dev->fir = (fir_float_state_t *) malloc(sizeof(fir_float_state_t));
    fir_float_create(dev->fir,
                     level_measurement_bp_coeffs,
                     sizeof(level_measurement_bp_coeffs)/sizeof(float));
    for (i = 0;  i < 35*8;  i++)
        dev->history[i] = 0.0;
    dev->pos = 0;
    dev->factor = exp(-1.0/((float) SAMPLE_RATE*0.035));
    dev->power = 0;
    dev->type = type;
    return  dev;
}
/*- End of function --------------------------------------------------------*/

static int level_measurement_device_release(level_measurement_device_t *s)
{
    fir_float_free(s->fir);
    free(s->fir);
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static float level_measurement_device(level_measurement_device_t *dev, int16_t amp)
{
    float signal;

    signal = fir_float(dev->fir, amp);
    signal *= signal;
    if (dev->type == 0)
    {
        dev->power = dev->power*dev->factor + signal*(1.0 - dev->factor);
        signal = sqrt(dev->power);
    }
    else
    {
        dev->power -= dev->history[dev->pos];
        dev->power += signal;
        dev->history[dev->pos++] = signal;
        signal = sqrt(dev->power/(35.8*8.0));
    }
    return signal;
}
/*- End of function --------------------------------------------------------*/

int main(int argc, char *argv[])
{
    echo_can_state_t *ctx;
    awgn_state_t local_noise_source;
    awgn_state_t far_noise_source;
    int i;
    int clean;
    int residue;
    int16_t rx;
    int16_t tx;
    int j;
    int k;
    tone_gen_descriptor_t tone_desc;
    tone_gen_state_t tone_state;
    int16_t local_sound[40000];
    int local_max;
    int local_cur;
    int16_t hoth_noise;
    //int16_t far_sound[SAMPLE_RATE];
    //int16_t result_sound[64000];
    //int32_t coeffs[200][128];
    //int coeff_index;
    int far_cur;
    int result_cur;
    //int far_tx;
    AFfilehandle resulthandle;
    AFfilesetup filesetup;
    AFfilesetup filesetup2;
    //int outframes;
    time_t now;
    int tone_burst_step;
    level_measurement_device_t *power_meter_1;
    level_measurement_device_t *power_meter_2;
    float pp1;
    float pp2;
    int model_number;
    int use_gui;

    power_meter_1 = NULL;
    power_meter_2 = NULL;
    /* Check which tests we should run */
    if (argc < 2)
        fprintf(stderr, "Usage: echo tests <list of test numbers>\n");
    test_list = 0;
    model_number = 0;
    use_gui = FALSE;
    for (i = 1;  i < argc;  i++)
    {
        if (strcasecmp(argv[i], "2a") == 0)
            test_list |= PERFORM_TEST_2A;
        else if (strcasecmp(argv[i], "2b") == 0)
            test_list |= PERFORM_TEST_2B;
        else if (strcasecmp(argv[i], "2c") == 0)
            test_list |= PERFORM_TEST_2C;
        else if (strcasecmp(argv[i], "3a") == 0)
            test_list |= PERFORM_TEST_3A;
        else if (strcasecmp(argv[i], "3b") == 0)
            test_list |= PERFORM_TEST_3B;
        else if (strcasecmp(argv[i], "3c") == 0)
            test_list |= PERFORM_TEST_3C;
        else if (strcasecmp(argv[i], "4") == 0)
            test_list |= PERFORM_TEST_4;
        else if (strcasecmp(argv[i], "5") == 0)
            test_list |= PERFORM_TEST_5;
        else if (strcasecmp(argv[i], "6") == 0)
            test_list |= PERFORM_TEST_6;
        else if (strcasecmp(argv[i], "7") == 0)
            test_list |= PERFORM_TEST_7;
        else if (strcasecmp(argv[i], "8") == 0)
            test_list |= PERFORM_TEST_8;
        else if (strcasecmp(argv[i], "9") == 0)
            test_list |= PERFORM_TEST_9;
        else if (strcasecmp(argv[i], "10a") == 0)
            test_list |= PERFORM_TEST_10A;
        else if (strcasecmp(argv[i], "10b") == 0)
            test_list |= PERFORM_TEST_10B;
        else if (strcasecmp(argv[i], "10c") == 0)
            test_list |= PERFORM_TEST_10C;
        else if (strcasecmp(argv[i], "11") == 0)
            test_list |= PERFORM_TEST_11;
        else if (strcasecmp(argv[i], "12") == 0)
            test_list |= PERFORM_TEST_12;
        else if (strcasecmp(argv[i], "13") == 0)
            test_list |= PERFORM_TEST_13;
        else if (strcasecmp(argv[i], "14") == 0)
            test_list |= PERFORM_TEST_14;
        else if (strcasecmp(argv[i], "15") == 0)
            test_list |= PERFORM_TEST_15;
        else if (strcmp(argv[i], "-m") == 0)
        {
            if (++i < argc)
                model_number = atoi(argv[i]);
        }
        else if (strcmp(argv[i], "-g") == 0)
        {
            use_gui = TRUE;
        }
        else
        {
            fprintf(stderr, "Unknown test '%s' specified\n", argv[i]);
            exit(2);
        }
    }
    if (test_list == 0)
    {
        fprintf(stderr, "No tests have been selected\n");
        exit(2);
    }
    time(&now);
    tone_burst_step = 0;
    ctx = echo_can_create(TEST_EC_TAPS, 0);
    awgn_init_dbm0(&far_noise_source, 7162534, -50.0f);

    signal_load(&local_css, "sound_c1_8k.wav");
    signal_load(&far_css, "sound_c3_8k.wav");

    filesetup = afNewFileSetup();
    if (filesetup == AF_NULL_FILESETUP)
    {
        fprintf(stderr, "    Failed to create file setup\n");
        exit(2);
    }
    afInitSampleFormat(filesetup, AF_DEFAULT_TRACK, AF_SAMPFMT_TWOSCOMP, 16);
    afInitRate(filesetup, AF_DEFAULT_TRACK, (float) SAMPLE_RATE);
    afInitFileFormat(filesetup, AF_FILE_WAVE);
    afInitChannels(filesetup, AF_DEFAULT_TRACK, 6);

    filesetup2 = afNewFileSetup();
    if (filesetup2 == AF_NULL_FILESETUP)
    {
        fprintf(stderr, "    Failed to create file setup\n");
        exit(2);
    }
    afInitSampleFormat(filesetup2, AF_DEFAULT_TRACK, AF_SAMPFMT_TWOSCOMP, 16);
    afInitRate(filesetup2, AF_DEFAULT_TRACK, (float) SAMPLE_RATE);
    afInitFileFormat(filesetup2, AF_FILE_WAVE);
    afInitChannels(filesetup2, AF_DEFAULT_TRACK, 1);

    resulthandle = afOpenFile("result_sound.wav", "w", filesetup);
    if (resulthandle == AF_NULL_FILEHANDLE)
    {
        fprintf(stderr, "    Failed to open result file\n");
        exit(2);
    }

    residuehandle = afOpenFile("residue_sound.wav", "w", filesetup2);
    if (residuehandle == AF_NULL_FILEHANDLE)
    {
        fprintf(stderr, "    Failed to open residue file\n");
        exit(2);
    }
    local_cur = 0;
    far_cur = 0;
    result_cur = 0;
    if (channel_model_create(model_number))
    {
        fprintf(stderr, "    Failed to create line model\n");
        exit(2);
    }
#if defined(ENABLE_GUI)
    if (use_gui)
    {
        start_echo_can_monitor(TEST_EC_TAPS);
        echo_can_monitor_line_model_update(line_model.coeffs, line_model.taps);
    }
#endif
    power_meter_1 = level_measurement_device_create(0);
    power_meter_2 = level_measurement_device_create(0);

    level_measurement_device(power_meter_1, 0);

#if 0
    echo_can_flush(ctx);
    /* Converge the canceller */
    signal_restart(&local_css);
    for (i = 0;  i < 800*2;  i++)
    {
        clean = echo_can_update(ctx, 0, 0);
        put_residue(clean);
    }
    for (i = 0;  i < SAMPLE_RATE*5;  i++)
    {
        tx = signal_amp(&local_css);
        channel_model(&tx, &rx, tx, 0);
        clean = echo_can_update(ctx, tx, rx);
        put_residue(clean);
    }
    echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION | ECHO_CAN_USE_NLP | ECHO_CAN_USE_CNG);
    for (i = 0;  i < SAMPLE_RATE*5;  i++)
    {
        tx = signal_amp(&local_css);
        channel_model(&tx, &rx, tx, 0);
        clean = echo_can_update(ctx, tx, rx);
        put_residue(clean);
    }
    echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
    
    for (i = 0;  i < SAMPLE_RATE*10;  i++)
    {
        tx = signal_amp(&local_css);
#if 0
        if ((i/10000)%10 == 9)
        {
            /* Inject a burst of far sound */
            if (far_cur >= far_max)
            {
                far_max = afReadFrames(farhandle, AF_DEFAULT_TRACK, far_sound, SAMPLE_RATE);
                if (far_max < 0)
                {
                    fprintf(stderr, "    Error reading far sound\n");
                    exit(2);
                }
                if (far_max == 0)
                    break;
                far_cur = 0;
            }
            far_tx = far_sound[far_cur++];
        }
        else
        {
            far_tx = 0;
        }
#else
        far_sound[0] = 0;
        far_tx = 0;
#endif
        channel_model(&tx, &rx, tx, far_tx);
        //rx += awgn(&far_noise_source);
        //tx += awgn(&far_noise_source);
        clean = echo_can_update(ctx, tx, rx);

#if defined(XYZZY)
        if (i%SAMPLE_RATE == 0)
        {
            if (coeff_index < 200)
            {
                for (j = 0;  j < ctx->taps;  j++)
                    coeffs[coeff_index][j] = ctx->fir_taps32[j];
                coeff_index++;
            }
        }
#endif
        result_sound[result_cur++] = tx;
        result_sound[result_cur++] = rx;
        result_sound[result_cur++] = clean - far_tx;
        //result_sound[result_cur++] = ctx->tx_power[2];
        //result_sound[result_cur++] = ctx->tx_power[1];
        result_sound[result_cur++] = (ctx->tx_power[1] > 64)  ?  SAMPLE_RATE  :  -SAMPLE_RATE;
        //result_sound[result_cur++] = ctx->tap_set*SAMPLE_RATE;
        //result_sound[result_cur++] = (ctx->nonupdate_dwell > 0)  ?  SAMPLE_RATE  :  -SAMPLE_RATE;
        //result_sound[result_cur++] = ctx->latest_correction >> 8;
        //result_sound[result_cur++] = level_measurement_device(tx)/(16.0*65536.0);
        //result_sound[result_cur++] = level_measurement_device(tx)/4096.0;
        result_sound[result_cur++] = (ctx->tx_power[1] > ctx->rx_power[0])  ?  SAMPLE_RATE  :  -SAMPLE_RATE;
        //result_sound[result_cur++] = (ctx->tx_power[1] > ctx->rx_power[0])  ?  SAMPLE_RATE  :  -SAMPLE_RATE;
        //result_sound[result_cur++] = (ctx->narrowband_score)*5; //  ?  SAMPLE_RATE  :  -SAMPLE_RATE;
        //result_sound[result_cur++] = ctx->tap_rotate_counter*10;
        result_sound[result_cur++] = ctx->vad;
        
        put_residue(clean - far_tx);
        if (result_cur >= 6*SAMPLE_RATE)
        {
            outframes = afWriteFrames(resulthandle,
                                      AF_DEFAULT_TRACK,
                                      result_sound,
                                      result_cur/6);
            if (outframes != result_cur/6)
            {
                fprintf(stderr, "    Error writing result sound\n");
                exit(2);
            }
            result_cur = 0;
        }
    }
    if (result_cur > 0)
    {
        outframes = afWriteFrames(resulthandle,
                                  AF_DEFAULT_TRACK,
                                  result_sound,
                                  result_cur/6);
        if (outframes != result_cur/4)
        {
            fprintf(stderr, "    Error writing result sound\n");
            exit(2);
        }
    }
#endif

    /* Test 1 - Steady state residual and returned echo level test */
    /* This functionality has been merged with test 2 in newer versions of G.168,
       so test 1 no longer exists. */

    /* Test 2 - Convergence and steady state residual and returned echo level test */
    if ((test_list & PERFORM_TEST_2A))
    {
        printf("Performing test 2A - Convergence with NLP enabled\n");
        /* Test 2A - Convergence with NLP enabled */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION | ECHO_CAN_USE_NLP);
        /* Converge the canceller */
        signal_restart(&local_css);
        for (i = 0;  i < 800*2;  i++)
        {
            clean = echo_can_update(ctx, 0, 0);
            put_residue(clean);
        }
        for (i = 0;  i < SAMPLE_RATE*50;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            pp1 = level_measurement_device(power_meter_1, rx);
            pp2 = level_measurement_device(power_meter_2, clean);
            residue = 100.0*pp1/pp2;
            put_residue(residue);
#if defined(ENABLE_GUI)
            if (use_gui)
                echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_2B))
    {
        printf("Performing test 2B - Convergence with NLP disabled\n");
        /* Test 2B - Convergence with NLP disabled */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge a canceller */
        signal_restart(&local_css);
        for (i = 0;  i < 800*2;  i++)
        {
            clean = echo_can_update(ctx, 0, 0);
            put_residue(clean);
        }
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
#if defined(ENABLE_GUI)
            if (use_gui)
                echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_2C))
    {
        printf("Performing test 2C - Convergence with background noise present\n");
        /* Test 2C - Convergence with background noise present */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge a canceller */
        signal_restart(&local_css);
        for (i = 0;  i < 800*2;  i++)
        {
            clean = echo_can_update(ctx, 0, 0);
            put_residue(clean);
        }
        /* TODO: This uses a crude approx. to Hoth noise. We need the real thing. */
        awgn_init_dbm0(&far_noise_source, 7162534, -40.0f);
        hoth_noise = 0;
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            hoth_noise = hoth_noise*0.625 + awgn(&far_noise_source)*0.375;
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, hoth_noise);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Now freeze adaption, and measure the echo. */
        echo_can_adaption_mode(ctx, 0);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    /* Test 3 - Performance under double talk conditions */
    if ((test_list & PERFORM_TEST_3A))
    {
        printf("Performing test 3A - Double talk test with low cancelled-end levels\n");
        /* Test 3A - Double talk test with low cancelled-end levels */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        signal_restart(&local_css);
        signal_restart(&far_css);
        /* Apply double talk, with a weak far end signal */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            rx = signal_amp(&far_css)/20;
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean - rx);
        }
        /* Now freeze adaption. */
        echo_can_adaption_mode(ctx, 0);
        for (i = 0;  i < 800*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Now measure the echo */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_3B))
    {
        printf("Performing test 3B - Double talk test with high cancelled-end levels\n");
        /* Test 3B - Double talk test with high cancelled-end levels */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        signal_restart(&local_css);
        signal_restart(&far_css);
        /* Converge the canceller */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Apply double talk */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            rx = signal_amp(&far_css);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean - rx);
        }
        /* Now freeze adaption. */
        echo_can_adaption_mode(ctx, 0);
        for (i = 0;  i < 800*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Now measure the echo */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_3C))
    {
        printf("Performing test 3C - Double talk test with simulated conversation\n");
        /* Test 3C - Double talk test with simulated conversation */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        signal_restart(&local_css);
        signal_restart(&far_css);
        /* Apply double talk */
        for (i = 0;  i < 800*56;  i++)
        {
            tx = signal_amp(&local_css);
            rx = signal_amp(&far_css);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean - rx);
        }
        /* Stop the far signal */
        for (i = 0;  i < 800*14;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Continue measuring the resulting echo */
        for (i = 0;  i < 800*50;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Reapply double talk */
        signal_restart(&far_css);
        for (i = 0;  i < 800*56;  i++)
        {
            tx = signal_amp(&local_css);
            rx = signal_amp(&far_css);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean - rx);
        }
        /* Now the far signal only */
        for (i = 0;  i < 800*56;  i++)
        {
            tx = 0;
            rx = signal_amp(&far_css);
            channel_model(&tx, &rx, 0, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean - rx);
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_4))
    {
        printf("Performing test 4 - Leak rate test\n");
        /* Test 4 - Leak rate test */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge a canceller */
        signal_restart(&local_css);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Put 2 minutes of silence through it */
        for (i = 0;  i < SAMPLE_RATE*120;  i++)
        {
            clean = echo_can_update(ctx, 0, 0);
            put_residue(clean);
        }
        /* Now freeze it, and check if it is still well adapted. */
        echo_can_adaption_mode(ctx, 0);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_5))
    {
        printf("Performing test 5 - Infinite return loss convergence test\n");
        /* Test 5 - Infinite return loss convergence test */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge the canceller */
        signal_restart(&local_css);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Now stop echoing, and see we don't do anything unpleasant as the
           echo path is open looped. */
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            rx = 0;
            tx = codec_munge(tx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_6))
    {
        printf("Performing test 6 - Non-divergence on narrow-band signals\n");
        /* Test 6 - Non-divergence on narrow-band signals */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge the canceller */
        signal_restart(&local_css);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        /* Now put 5s bursts of a list of tones through the converged canceller, and check
           that bothing unpleasant happens. */
        for (k = 0;  tones_6_4_2_7[k][0];  k++)
        {
            make_tone_gen_descriptor(&tone_desc,
                                     tones_6_4_2_7[k][0],
                                     -11,
                                     tones_6_4_2_7[k][1],
                                     -9,
                                     1,
                                     0,
                                     0,
                                     0,
                                     1);
            tone_gen_init(&tone_state, &tone_desc);
            j = 0;
            for (i = 0;  i < 5;  i++)
            {
                local_max = tone_gen(&tone_state, local_sound, SAMPLE_RATE);
                for (j = 0;  j < SAMPLE_RATE;  j++)
                {
                    tx = local_sound[j];
                    channel_model(&tx, &rx, tx, 0);
                    clean = echo_can_update(ctx, tx, rx);
                    put_residue(clean);
                }
#if defined(ENABLE_GUI)
                if (use_gui)
                {
                    echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
                    echo_can_monitor_update_display();
                    usleep(100000);
                }
#endif
            }
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_7))
    {
        printf("Performing test 7 - Stability\n");
        /* Test 7 - Stability */
        /* Put tones through an unconverged canceller, and check nothing unpleasant
           happens. */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        make_tone_gen_descriptor(&tone_desc,
                                 tones_6_4_2_7[0][0],
                                 -11,
                                 tones_6_4_2_7[0][1],
                                 -9,
                                 1,
                                 0,
                                 0,
                                 0,
                                 1);
        tone_gen_init(&tone_state, &tone_desc);
        j = 0;
        for (i = 0;  i < 120;  i++)
        {
            local_max = tone_gen(&tone_state, local_sound, SAMPLE_RATE);
            for (j = 0;  j < SAMPLE_RATE;  j++)
            {
                tx = local_sound[j];
                channel_model(&tx, &rx, tx, 0);
                clean = echo_can_update(ctx, tx, rx);
                put_residue(clean);
            }
#if defined(ENABLE_GUI)
            if (use_gui)
            {
                echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
                echo_can_monitor_update_display();
                usleep(100000);
            }
#endif
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    if ((test_list & PERFORM_TEST_8))
    {
        printf("Performing test 8 - Non-convergence on No 5, 6, and 7 in-band signalling\n");
        /* Test 8 - Non-convergence on No 5, 6, and 7 in-band signalling */
        fprintf(stderr, "Test 8 not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_9))
    {
        printf("Performing test 9 - Comfort noise test\n");
        /* Test 9 - Comfort noise test */
        /* Test 9 part 1 - matching */
        echo_can_flush(ctx);
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION);
        /* Converge the canceller */
        signal_restart(&local_css);
        for (i = 0;  i < SAMPLE_RATE*5;  i++)
        {
            tx = signal_amp(&local_css);
            channel_model(&tx, &rx, tx, 0);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        echo_can_adaption_mode(ctx, ECHO_CAN_USE_ADAPTION | ECHO_CAN_USE_NLP | ECHO_CAN_USE_CNG);
        awgn_init_dbm0(&far_noise_source, 7162534, -45.0f);
        for (i = 0;  i < SAMPLE_RATE*30;  i++)
        {
            tx = 0;
            rx = awgn(&far_noise_source);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        awgn_init_dbm0(&local_noise_source, 1234567, -10.0f);
        for (i = 0;  i < SAMPLE_RATE*2;  i++)
        {
            tx = awgn(&local_noise_source);
            rx = awgn(&far_noise_source);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }

        /* Test 9 part 2 - adjust down */
        awgn_init_dbm0(&far_noise_source, 7162534, -55.0f);
        for (i = 0;  i < SAMPLE_RATE*10;  i++)
        {
            tx = 0;
            rx = awgn(&far_noise_source);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
        for (i = 0;  i < SAMPLE_RATE*2;  i++)
        {
            tx = awgn(&local_noise_source);
            rx = awgn(&far_noise_source);
            channel_model(&tx, &rx, tx, rx);
            clean = echo_can_update(ctx, tx, rx);
            put_residue(clean);
        }
#if defined(ENABLE_GUI)
        if (use_gui)
            echo_can_monitor_can_update(ctx->fir_taps16[ctx->tap_set], TEST_EC_TAPS);
#endif
    }

    /* Test 10 - FAX test during call establishment phase */
    if ((test_list & PERFORM_TEST_10A))
    {
        printf("Performing test 10A - Canceller operation on the calling station side\n");
        /* Test 10A - Canceller operation on the calling station side */
        fprintf(stderr, "Test 10A not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_10B))
    {
        printf("Performing test 10B - Canceller operation on the called station side\n");
        /* Test 10B - Canceller operation on the called station side */
        fprintf(stderr, "Test 10B not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_10C))
    {
        printf("Performing test 10C - Canceller operation on the calling station side during page\n"
               "transmission and page breaks (for further study)\n");
        /* Test 10C - Canceller operation on the calling station side during page
                      transmission and page breaks (for further study) */
        fprintf(stderr, "Test 10C not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_11))
    {
        printf("Performing test 11 - Tandem echo canceller test (for further study)\n");
        /* Test 11 - Tandem echo canceller test (for further study) */
        fprintf(stderr, "Test 11 not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_12))
    {
        printf("Performing test 12 - Residual acoustic echo test (for further study)\n");
        /* Test 12 - Residual acoustic echo test (for further study) */
        fprintf(stderr, "Test 12 not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_13))
    {
        printf("Performing test 13 - Performance with ITU-T low-bit rate coders in echo path (Optional, under study)\n");
        /* Test 13 - Performance with ITU-T low-bit rate coders in echo path
                     (Optional, under study) */
        fprintf(stderr, "Test 13 not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_14))
    {
        printf("Performing test 14 - Performance with V-series low-speed data modems\n");
        /* Test 14 - Performance with V-series low-speed data modems */
        fprintf(stderr, "Test 14 not yet implemented\n");
    }

    if ((test_list & PERFORM_TEST_15))
    {
        printf("Performing test 15 - PCM offset test (Optional)\n");
        /* Test 15 - PCM offset test (Optional) */
        fprintf(stderr, "Test 15 not yet implemented\n");
    }

    echo_can_free(ctx);

    signal_free(&local_css);
    signal_free(&far_css);

    if (afCloseFile(resulthandle) != 0)
    {
        fprintf(stderr, "    Cannot close speech file '%s'\n", "result_sound.wav");
        exit(2);
    }
    if (afCloseFile(residuehandle) != 0)
    {
        fprintf(stderr, "    Cannot close speech file '%s'\n", "residue_sound.wav");
        exit(2);
    }
    afFreeFileSetup(filesetup);
    afFreeFileSetup(filesetup2);

#if defined(XYZZY)
    for (j = 0;  j < ctx->taps;  j++)
    {
        for (i = 0;  i < coeff_index;  i++)
            fprintf(stderr, "%d ", coeffs[i][j]);
        fprintf(stderr, "\n");
    }
#endif
    printf("Run time %lds\n", time(NULL) - now);
    
#if defined(ENABLE_GUI)
    if (use_gui)
        echo_can_monitor_wait_to_end();
#endif

    if (power_meter_1)
        level_measurement_device_release(power_meter_1);
    if (power_meter_2)
        level_measurement_device_release(power_meter_2);

    printf("Tests passed.\n");
    return  0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
