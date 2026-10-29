#ifndef TINYML_ONLINE_H
#define TINYML_ONLINE_H

#include <stdint.h>
#include "stai.h"

typedef struct
{
    float raw[3];
    float reference[3];
    float neural[3];
    float error[3];
    float max_abs_error;
    float mean_abs_error;
} TinyML_OnlineResult_t;

stai_return_code TinyML_Online_Init(void);
stai_return_code TinyML_Online_Run(
    int16_t raw_x,
    int16_t raw_y,
    int16_t raw_z,
    TinyML_OnlineResult_t *result);

#endif
