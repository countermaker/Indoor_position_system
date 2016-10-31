/**
  ******************************************************************************
  * @file    imu.c
  * @author  counter
  * @version V3.1
  * @date    2016-6-23
  * @brief   This file provides the estimation of the status of quad
  ******************************************************************************
  * @attention none
*/
/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "mcu.h"
#include "data.h"
#include "math.h"
#include "pid.h"
#include "quad_math.h"
#include "navigation.h"
#include "rotate.h"
#include "global.h"
#include "math.h"

#define Kp 0.8f                       
#define Ki 0.000f                     

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   
float exInt = 0, eyInt = 0, ezInt = 0;    
/**
  * @brief  Estimate the Eular angle
  * @param  ax,ay,az,gx,gy,gz
            dt the IMU process period
  * @retval None
  */
void Quat_EstimateEuler(Vector acc,Vector gyro,float dt)
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;	
	
	if(acc.x * acc.y * acc.z==0)
 		return;
		
  norm = Q_rsqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);       //one
  acc.x = acc.x *norm;
  acc.y = acc.y * norm;
  acc.z = acc.z * norm;

  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (-acc.y*vz + acc.z*vy) ;                       //error calculate
  ey = (-acc.z*vx + acc.x*vz) ;
  ez = (-acc.x*vy + acc.y*vx) ;

  exInt = exInt + ex * Ki;								   //error integral
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gyro.x = gyro.x + Kp *  ex + exInt;	               //error compensate
	gyro.y = gyro.y + Kp *  ey + eyInt;	
	gyro.z = gyro.z + Kp *  ez + ezInt;	
  														
  q0 = q0 + (-q1*gyro.x - q2*gyro.y - q3*gyro.z)*dt/2.0f; 
  q1 = q1 + (q0*gyro.x + q2*gyro.z - q3*gyro.y)*dt/2.0f;
  q2 = q2 + (q0*gyro.y - q1*gyro.z + q3*gyro.x)*dt/2.0f;
  q3 = q3 + (q0*gyro.z + q1*gyro.y - q2*gyro.x)*dt/2.0f;

  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  g_Eular.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);// roll
	g_Eular.pitch = asin(-2 * q1 * q3 + 2 * q0* q2);
  g_Eular.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1); // yaw 
}
void Matrix_EstimateEuler(Vector acc,Vector gyro,float dt)
{
	float prior_yaw,prior_pitch,prior_roll;
	float acc_pitch,acc_roll;
	float yaw_d,pitch_d,roll_d;
	float tan_pitch,sin_roll,cos_roll,sin_pitch,cos_pitch,sec_pitch;
	float pitch_error,roll_error,yaw_error;
	
	fast_SinCos(g_Eular.roll,&sin_roll,&cos_roll);
	fast_SinCos(g_Eular.pitch,&sin_pitch,&cos_pitch);
	sec_pitch = 1.0 / cos_pitch;
	tan_pitch = sin_pitch / cos_pitch;
	
	roll_d =  1.0 * gyro.x + tan_pitch * sin_roll * gyro.y + tan_pitch * cos_roll * gyro.z;
	pitch_d = 0.0 * gyro.x + cos_roll             * gyro.y - sin_roll             * gyro.z;
	yaw_d =   0.0 * gyro.x + sec_pitch * sin_roll * gyro.y + sec_pitch * cos_roll * gyro.z;
	
	prior_yaw = g_Eular.yaw + yaw_d * dt;
	prior_pitch = g_Eular.pitch + pitch_d * dt;
	prior_roll = g_Eular.roll + roll_d * dt;
	
	pitch_error = g_CamEuler.pitch - prior_pitch;
	roll_error = g_CamEuler.roll - prior_roll;
	yaw_error = g_CamEuler.yaw - prior_yaw;
	
	acc_roll = atan2f(-acc.y,-acc.z);
	acc_pitch = atan2f(acc.x,sqrtf(acc.y * acc.y + acc.z * acc.z));

	if(g_FlagComUpdateEuler == SET && FL_ABS(pitch_error) <= 10* AtR && FL_ABS(roll_error) <= 10* AtR && FL_ABS(yaw_error) <= 10* AtR)
	{
		g_Eular.pitch = Warp_ToPI(Constraint_f(prior_pitch + KALMAN_POS_K1_PITCH * (acc_pitch - prior_pitch) + KALMAN_POS_K2_PITCH * (g_CamEuler.pitch - prior_pitch),-85.0f * AtR,85.0f * AtR));
		g_Eular.roll = Warp_ToPI(prior_roll + KALMAN_POS_K1_ROLL * (acc_roll - prior_roll) + KALMAN_POS_K2_ROLL * (g_CamEuler.roll - prior_roll));
		g_Eular.yaw = Warp_ToPI(prior_yaw + KALMAN_POS_K1_YAW * (g_CamEuler.yaw - prior_yaw));
		g_FlagComUpdateEuler = RESET;
	}
	else
	{
		g_Eular.pitch = Warp_ToPI(Constraint_f(prior_pitch + KALMAN_POS_K1_PITCH * (acc_pitch - prior_pitch),-85 * AtR,85 * AtR));
		g_Eular.roll = Warp_ToPI(prior_roll + KALMAN_POS_K1_ROLL * (acc_roll - prior_roll));
		g_Eular.yaw = Warp_ToPI(prior_yaw);
	}
}
/**
  * @brief  Estimate the position(run in the interrupt)
  * @param  Vector *acc,Vector *sensor_measure,Vector*position,float dt,Angle*angle
  * @retval None
  */
void Estimate_Status(Vector *acc,Vector sensor_measure,float dt,Eular angle)
{
	static float dt_halfsqure;
	static Vector acci;
	float tmp;
	static uint8_t update_flag = 0;
	dt_halfsqure = 0.5f * dt * dt;
	/*Acc*/
	Eular_RoateVect(acc,&acci,&angle);
	Vect_Scale(&acci,-1);
	Vect_Sub(&acci,&acci,&g_AccRefOffset);
	
	g_AccRef.x = DLPF(acci.x, g_AccRef.x, 10 * HZ2RAD, dt);
  g_AccRef.y = DLPF(acci.y, g_AccRef.y, 10 * HZ2RAD, dt);
  g_AccRef.z = DLPF(acci.z, g_AccRef.z, 10 * HZ2RAD, dt);

	if(g_FlagComUpdate == SET)
	{
		g_FlagComUpdate = RESET;
		update_flag = SET;	
	}
	/* position z estimate */
	g_VelRef.z = g_VelRef.z + g_AccRef.z * dt;
  g_PosRef.z = Constraint_f(g_PosRef.z + g_VelRef.z * dt + g_AccRef.z * dt_halfsqure, 0, 3);
	if(update_flag == SET)
	{
		tmp = sensor_measure.z - g_PosRef.z;
		g_PosRef.z = g_PosRef.z + KALMAN_POS_K1_Z * tmp;
    g_VelRef.z = g_VelRef.z + KALMAN_POS_K2_Z * tmp;
		g_AccRefOffset.z = g_AccRefOffset.z + KALMAN_POS_K3_Z * tmp;
	}
	/* position x estimate */
	g_VelRef.x = g_VelRef.x + g_AccRef.x * dt;
	g_PosRef.x = g_PosRef.x + g_VelRef.x * dt + g_AccRef.x * dt_halfsqure;
	if (update_flag == SET) 
	{
    tmp = (sensor_measure.x - g_PosRef.x); /* position error */
    g_PosRef.x = g_PosRef.x + KALMAN_POS_K1_X * tmp;
		g_VelRef.x = g_VelRef.x + KALMAN_POS_K2_X * tmp;
		g_AccRefOffset.x = g_AccRefOffset.x + KALMAN_POS_K3_X * tmp;
  }	
	/* position y estimate */
		g_VelRef.y = g_VelRef.y + g_AccRef.y * dt;
		g_PosRef.y = g_PosRef.y + g_VelRef.y * dt + g_AccRef.y * dt_halfsqure;
		if (update_flag == SET) 
		{
      tmp = (sensor_measure.y - g_PosRef.y); /* position error */
      g_PosRef.y = g_PosRef.y + KALMAN_POS_K1_Y * tmp;
			g_VelRef.y = g_VelRef.y + KALMAN_POS_K2_Y * tmp;
			g_AccRefOffset.y = g_AccRefOffset.y + KALMAN_POS_K3_Y * tmp;
    }	
	update_flag = RESET;	
}
