/****************************************************************************
 * ei_main_loop.cpp
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#include <string>
#include "ei_run_classifier.h"
#include "ei_classifier_porting.h"
#include "audioMoth.h"

/* NOTE: Depending on how the EI model was trained, the "interesting word" might be at index 0 or 1
We keep track of this with this variable */

#define INTERESTING_SOUND_INDEX 1

/* Pointer set to the current audio sample to be classified. Used by the callback function */ 

static int16_t *audio_sample;

/* Callback function declaration */

static int get_signal_data(size_t offset, size_t length, float *out_ptr);

/* Main function to classify audio signal using new callback function */

extern "C" float ei_classify(int16_t *raw_features, uint32_t signal_size, bool *model_error){

    audio_sample = raw_features;

    ei_impulse_result_t result;

    signal_t signal;

    signal.total_length = signal_size;

    signal.get_data = &get_signal_data;

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

    /* Check if there is an error */
    if (res != EI_IMPULSE_OK) {

        *model_error = true;
        
        return 0;
    
    }

    /* Finally return the probability of having spotted the wanted sound to the AudioMoth system */ 

    float detection_prob = result.classification[INTERESTING_SOUND_INDEX].value;

    return detection_prob;
}

/* Callback: fill a section of the out_ptr buffer when requested */

static int get_signal_data(size_t offset, size_t length, float *out_ptr){
    
    for (size_t i = 0; i < length; i++){

        out_ptr[i] = (float)(audio_sample + offset)[i];

    }

    return EIDSP_OK;
}
