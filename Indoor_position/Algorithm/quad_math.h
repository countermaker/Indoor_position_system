#ifndef _QUAD_MATH_
#define _QUAD_MATH_
#include "mcu.h"

#ifndef PI
#define PI     (3.14159265358979f)
#endif

#define DEG2RAD        (PI/180.0f)
#define HZ2RAD         (2 * PI)

float Q_rsqrt(float number);

float FL_ABS(float x);

void Limit(int16_t *data,int16_t limit);

void Limitf(float *data,float limit);

void Limit_t2(int16_t *data,int16_t limit_low,int16_t limit_up);

float Constraint_f(float a , float lower , float upper);

int32_t Constraint_int(int32_t a , int32_t lower , int32_t upper);

float DLPF(float new_data, float old_data, float a, float dt);

float Warp_ToPI(float a);

float max(float a, float b);

float min(float a, float b);

#endif
