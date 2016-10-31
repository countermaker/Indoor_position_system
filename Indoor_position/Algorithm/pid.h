#ifndef __PID_H
#define	__PID_H 
#include "mcu.h"
#include "pwm.h"
#include "data.h"
#include "imu.h"
#include "monitor.h"
#include "quad_math.h"
#include "global.h"

/*limit*/
#define MOTOR_ROLL_PITCH_LIMIT 20000
#define MOTOR_YAW_LIMIT 200
#define INTEGRAL_LIMIT 20
#define METER_LIMIT 50
#define MOTOR_MAX 7999
#define ERROR_RATE_I_LIMIT   (20.0f)

void Eular_Control(Eular demand_angle, Eular current_angle,Vector *gyro,Vector *acc,float dt,Ctrl_DataType*control_data,float command_throttle);
void Position_Control(Vector* position,Vector* target_position,Ctrl_DataType*control_data,float dt,uint8_t mode);

#endif
