/****************************************************************************
 * digitalFilter.c
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

#include "digitalFilter.h"

/* Filter constant */

#define MINIMUM_NUMBER_OF_ITERATIONS            16

/* Filter global variables */

static float yc0;

static float gain;

static float xv0, xv1;
static float yv0, yv1;

/* Static filter function */

static inline float applyHighPassFilter(float sample) {

    xv0 = xv1;
    xv1 = sample * gain;

    yv0 = yv1;
    yv1 = xv1 - xv0 + yc0 * yv0;

    return yv1;

}

/* General filter routine */

static void filter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size) {

    uint32_t index = 0;

    for (uint32_t i = 0; i < size; i += sampleRateDivider) {

        float sample = 0.0f;

        for (uint32_t j = 0; j < sampleRateDivider; j += 1) {

            sample += source[i + j];

        }

        float filterOutput = applyHighPassFilter(sample);

        /* Apply output range limits */

        if (filterOutput > INT16_MAX) {

            filterOutput = INT16_MAX;

        } else if (filterOutput < -INT16_MAX) {

            filterOutput = -INT16_MAX;

        }

        /* Write the output value */

        dest[index++] = (int16_t)filterOutput;

    }

}

/* Fast filter routine */

static void fastFilter(int16_t *source, int16_t *dest, uint32_t size) {

    uint32_t index = 0;

    for (uint32_t i = 0; i < size / MINIMUM_NUMBER_OF_ITERATIONS; i += 1) {

        for (uint32_t j = 0; j < MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

            float sample = source[index];

            float filterOutput = applyHighPassFilter(sample);

            /* Apply output range limits */

            if (filterOutput > INT16_MAX) {

                filterOutput = INT16_MAX;

            } else if (filterOutput < -INT16_MAX) {

                filterOutput = -INT16_MAX;

            }

            /* Write the output value */

            dest[index++] = (int16_t)filterOutput;

        }

    }

}

/* Apply digital filter */

void DigitalFilter_applyFilter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size) {

    if (sampleRateDivider == 1) {

        fastFilter(source, dest, size);

    } else {

        filter(source, dest, sampleRateDivider, size);

    }

}

/* Reset the filter */

void DigitalFilter_reset() {

    xv0 = 0.0f;
    xv1 = 0.0f;

    yv0 = 0.0f;
    yv1 = 0.0f;

}

/* Update filter gain */

void DigitalFilter_setAdditionalGain(float g) {

    gain *= g;

}

void DigitalFilter_setHighPassFilterConstants(float g, float c) {

    gain = g;

    yc0 = c;

}