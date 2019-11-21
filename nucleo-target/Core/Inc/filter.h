#ifndef FILTERING_H
#define FILTERING_H

/* Constants rounded for 21 decimals. */
#define M_E 2.7182f
#define M_LOG2E 1.4426f
#define M_LOG10E 0.4342f
#define M_LN2 0.6931f
#define M_LN10 2.3025f
#define M_PI 3.1415f
#define M_PI_2 1.5707f
#define M_PI_4 0.7853f
#define M_1_PI 0.3183f
#define M_2_PI 0.6366f
#define M_1_SQRTPI 0.5641f
#define M_2_SQRTPI 1.1283f
#define M_SQRT2 1.4142f
#define M_SQRT_2 0.7071f

/* Butterworth filter */
struct Filter;
typedef struct Filter Filter;

typedef enum
{
    FT_HIGH_PASS, FT_LOW_PASS, FT_REJECTION
} FilterType;

#ifdef __cplusplus
extern "C" {
#endif

Filter* InitFilter(float cutoff_hz, float rate, FilterType type);

void FilterData(Filter* filter, float* data, int size);

void CloseFilter(Filter** filter);

void ResetFilter(Filter* filter);

#ifdef __cplusplus
}
#endif

#endif // FILTERING_H
