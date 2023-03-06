/* Edge Impulse inferencing library
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/****************************************************************************
 * ei_classifier_porting.cpp
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ei_classifier_porting.h"

/*  Dummy-Implementations of un-used Edge Impulse functions */ 

__attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled(){
    
    return EI_IMPULSE_OK;

}

__attribute__((weak)) EI_IMPULSE_ERROR ei_sleep(int32_t time_ms){

    return EI_IMPULSE_OK;

}

uint64_t ei_read_timer_ms(){

    return 0;

}

uint64_t ei_read_timer_us(){

    return 0; 

}

void ei_printf(const char *format, ...){}

void ei_printf_float(float f){}

/*  Implementation of used Edge Impulse functions */

const uint32_t MARGIN = 128;

extern void *maxHeapAddress;

__attribute__((weak)) void *ei_malloc(size_t size) {

    void *ptr = malloc(size);

    if ((uint32_t)ptr + size >= (uint32_t)maxHeapAddress - MARGIN) {

        return NULL;
    }

    return ptr;
}

__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size){

    void *ptr = calloc(nitems, size);

    if ((uint32_t)ptr + (nitems * size) >= (uint32_t)maxHeapAddress - MARGIN) {

        return NULL;
    }

    return ptr;
}

__attribute__((weak)) void ei_free(void *ptr){

    free(ptr);

}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
    __attribute__((weak)) void
    DebugLog(const char *s)
{
    ei_printf("%s", s);
}