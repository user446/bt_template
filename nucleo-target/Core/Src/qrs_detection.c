#include "qrs_detection.h"
#include "filter.h"
#include <stdlib.h>
#include <string.h>

#define WINDOW_SEC (0.160f)
#define MIN_RR_SEC (0.200f)

static void FilterSignal(float const* signal, int size, float rate, float* output);
static void ApplyArticleFilter(float const* signal, int size, float* output);
static void ComputeDerivative(float const* signal, int size, float* output);
static void ArrayPow2(float* signal, int size);
static void WindowIntegration(float const* signal, int size, float* output, int window_size);
static int Thresholding(const float* integrated, int size, int mir_rr_width, char* result);
static void Normalize(float* values, int size);
static void SubtractDelay(char* qrs_detection_result, int size, int samples_delay);

int DetectQrsPeaks(float const* signal, int size, char* result, float rate)
{
    const int WINDOW_SIZE = (int)(WINDOW_SEC * rate);
    const int MIN_RR = (int)(MIN_RR_SEC * rate);
    int count;
    float *buffer;  // filtered signal, integrated signal
    float *derivative;

    buffer = malloc(size * sizeof(float));
    FilterSignal(signal, size, rate, buffer);
    Normalize(buffer, size);

    derivative = malloc(size * sizeof(float));
    ComputeDerivative(buffer, size, derivative);
    Normalize(derivative, size);
    ArrayPow2(derivative, size);

    WindowIntegration(derivative, size, buffer, WINDOW_SIZE);
    free(derivative);
    memset(result, 0, size * sizeof(char));
    count = Thresholding(buffer, size, MIN_RR, result);
    free(buffer);
    // TODO: subtract filters delay
    //SubtractDelay(result, size, WINDOW_SIZE / 2);
		SubtractDelay(result, size, 0);
    return count;
}

/*****************************************************************************/

void ApplyArticleFilter(float const* signal, int size, float* output)
{
    int index;
    float* buffer;

    buffer = malloc(size * sizeof(float));

    /* Low Pass Filter */
    for (index = 0; index < size; ++index) {
        float value = signal[index];
        if (index >= 1) {
            value += 2 * buffer[index - 1];
        }
        if (index >= 2) {
            value -= buffer[index - 2];
        }
        if (index >= 6) {
            value -= 2 * signal[index - 6];
        }
        if (index >= 12) {
            value += signal[index - 12];
        }
        buffer[index] = value;
    }

    /* High Pass Filter */
    for (index = 0; index < size; ++index) {
        float value = -buffer[index];
        if (index >= 1) {
            value -= output[index - 1];
        }
        if (index >= 16) {
            value += 32 * buffer[index - 16];
        }
        if (index >= 32) {
            value += buffer[index - 32];
        }
        output[index] = value;
    }

    free(buffer);
}

void FilterSignal(float const* signal, int size, float rate, float* output)
{
    const float ARTICLE_SAMPLING_RATE = 200.0f;
    const float LOWER_HZ = 5.0f;
    const float UPPER_HZ = 15.0f;
    Filter* filter;

    if (rate == ARTICLE_SAMPLING_RATE) {
        ApplyArticleFilter(signal, size, output);
        return;
    }

    memcpy(output, signal, size * sizeof(float));

    filter = InitFilter(UPPER_HZ, rate, FT_LOW_PASS);
    FilterData(filter, output, size);
    CloseFilter(&filter);

    filter = InitFilter(LOWER_HZ, rate, FT_HIGH_PASS);
    FilterData(filter, output, size);
    CloseFilter(&filter);
}

void ComputeDerivative(float const* signal, int size, float* output)
{
    int i;

    for (i = 2; i < size - 2; ++i) {
        float value = (-signal[i - 2] - 2 * signal[i - 1] +
                        2 * signal[i + 1] + signal[i + 2]);
        output[i] = value / 8.0f;
    }
    output[0] = output[1] = output[2];
    output[size - 3] = output[size - 2] = output[size - 1];
}

void ArrayPow2(float* signal, int size)
{
    int i;

    for (i = 0; i < size; ++i) {
        signal[i] *= signal[i];
    }
}

void WindowIntegration(float const* signal, int size, float* output, int window_size)
{
    int i;
    float value = 0.0;
    //float inv_window_size = 1.0 / window_size;

    for (i = 0; i < size; ++i) {
        int first = i - (window_size - 1);
        //value += signal[index] * inv_window_size;
        value += signal[i] / window_size;
        if (first > 0) {
            //value -= signal[first - 1] * inv_window_size;
            value -= signal[first - 1] / window_size;
        }
        output[i] = value;
    }
}

int Thresholding(const float* integrated, int size, int mir_rr_width, char* result)
{
    int i, count, previous;
    float spki, npki, threshold1;

    spki = npki = 0.0f;
    count = 0;
    threshold1 = 0.0f;
    previous = -1;
    for (i = 1; i < size - 1; ++i) {
        float peaki = integrated[i];
        if (peaki < integrated[i - 1] || peaki <= integrated[i + 1]) {
            continue;
        }

        if (peaki <= threshold1) {
            npki = 0.875f * npki + 0.125f * peaki;
        } else {
            spki = 0.875f * spki + 0.125f * peaki;
        }
        threshold1 = npki + 0.25f * (spki - npki);

        if (peaki > threshold1) {
            if (count == 0) {
                result[i] = MARK_QRS;
                previous = i;
                ++count;
            } else if (i - previous >= mir_rr_width) {
                result[i] = MARK_QRS;
                previous = i;
                ++count;
            } else if (integrated[previous] < peaki) {
                result[previous] = MARK_NO_QRS;
                result[i] = MARK_QRS;
                previous = i;
            }
        }
    }
    return count;
}

void Normalize(float* values, int size)
{
    int i;
    float max_value = values[0];

    for (i = 1; i < size; ++i) {
        if (values[i] > max_value) {
            max_value = values[i];
        }
    }

    for (i = 0; i < size; ++i) {
        values[i] /= max_value;
    }
}

void SubtractDelay(char* qrs_detection_result, int size, int samples_delay)
{
    int i;

    for (i = samples_delay; i < size; ++i) {
        if (qrs_detection_result[i] == MARK_NO_QRS) {
            continue;
        }
        qrs_detection_result[i] = MARK_NO_QRS;
        qrs_detection_result[i - samples_delay] = MARK_QRS;
    }
}
