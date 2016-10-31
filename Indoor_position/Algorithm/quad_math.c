#include "quad_math.h"
/**
  * @brief  fast rsqrt
  * @param  number
  * @retval y
  */     
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   
	return y;
} 
/**
  * @brief  fast abs
  * @param  None
  * @retval None
  */
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
void Limit(int16_t *data,int16_t limit)
{
	if(*data<-limit)
	{
		*data=-limit;
		return;
	}
	if(*data>limit)
	{
		*data=limit;
		return;
	}
}

void Limitf(float *data,float limit)
{
	if(*data<-limit)
	{
		*data=-limit;
		return;
	}
	if(*data>limit)
	{
		*data=limit;
		return;
	}
}

void Limit_t2(int16_t *data,int16_t limit_low,int16_t limit_up)
{
	if(*data<limit_low)
	{
		*data=limit_low;
		return;
	}
	if(*data>limit_up)
	{
		*data=limit_up;
		return;
	}
}
/**
  * @brief  Constraint float data
  * @param  a lower upper
  * @retval a result
  */
float Constraint_f(float a , float lower , float upper)
{
  if (a >= upper)
    a = upper;
  else if (a <= lower)
    a = lower;
  return (a);
}
/**
  * @brief  Constraint int data
  * @param  a lower upper
  * @retval a result
  */
int32_t Constraint_int(int32_t a , int32_t lower , int32_t upper)
{
  if (a >= upper)
    return upper;
  else if (a <= lower)
    return lower;
  else
    return a;
}
/**
  * @brief  Low pass filter cutoff frequency 'a' (rad/s)
  * @param  new_data old_data a dt
  * @retval result
  */
float DLPF(float new_data, float old_data, float a, float dt)
{
  float   aT = a * dt;
  aT = Constraint_f(aT, 0.0f, 0.999f);
  return ((1.0f - aT) * old_data + aT * new_data);
}
/**
  * @brief  Constraint a (-2PI,2PI)
  * @param  a
  * @retval a
  */
float Warp_ToPI(float a)
{
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

float max(float a, float b)
{
  if (a > b)
    return a;
  else
    return b;
}

float min(float a, float b)
{
  if (a < b)
    return a;
  else
    return b;
}
