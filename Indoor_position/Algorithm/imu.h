#ifndef __IMU_H
#define	__IMU_H

#include "mpu6050.h"
#include "mcu.h"
#include "data.h"
#include "math.h"
#include "pid.h"
#include "quad_math.h"
#include "navigation.h"
#include "rotate.h"
#include "global.h"

void Matrix_EstimateEuler(Vector acc,Vector gyro,float dt);

void Quat_EstimateEuler(Vector acc,Vector gyro,float dt);

void Estimate_Status(Vector *acc,Vector sensor_measure,float dt,Eular angle);

#endif
