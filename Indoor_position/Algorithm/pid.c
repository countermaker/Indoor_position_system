/**
  ******************************************************************************
  * @file    pid.c
  * @author  counter
  * @version V3.1
  * @date    2016-6-23
  * @brief   This file provides the controler of the quad
  ******************************************************************************
  * @attention none
*/
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "math.h"

static Vector KpAngle = {11, 11, 16};
static Vector KpRate  = {0.012, 0.012,0.11}; //0.020
static Vector KiRate  = {0.003, 0.003, 0.01}; //0.005
static Vector KdRate  = {0.00130, 0.00130, 0.008}; //0.002
static Vector CONTROLER_KP = {3.5,3.5,2.4};
static Vector CONTROLER_KD = {2.0,2.0,0.45};

static Vector CONTROLER_KI = {1.0,1.0,0.005};
// float PID_POS_P_Z = 3.0;   //3.0
// float PID_VEL_P_Z = 0.6;  //0.55
// float PID_VEL_I_Z = 1.0;   //1.0
// float PID_VEL_D_Z = 0.065; //0.065

// float PID_POS_P_Y = 2.35;
// float PID_VEL_P_Y = 0.45;
// float PID_VEL_I_Y = 0.07;
// float PID_VEL_D_Y = 0.04;

// float PID_POS_P_X = 2.35;
// float PID_VEL_P_X = 0.2;
// float PID_VEL_I_X = 0.1;
// float PID_VEL_D_X = 0.015;

//float PID_POS_P_Y = 0.0;
//float PID_VEL_P_Y = 0.45;
//float PID_VEL_I_Y = 0.02;
//float PID_VEL_D_Y = 0.04;

//float PID_POS_P_X = 0.0;
//float PID_VEL_P_X = 0.25;
//float PID_VEL_I_X = 0.03;
//float PID_VEL_D_X = 0.05;
/**
  * @brief  Eular control
  * @param  Command Current_Angle,gx,gy,gz,ax,ay,az,dt
  * @retval None
  */
void Eular_Control(Eular demand_angle, Eular current_angle,Vector *gyro,Vector *acc,float dt,
Ctrl_DataType*control_data,float command_throttle)
{
	Vector ErrorRate;
	float throttle;
	//float cos_abg;
	float temp_errorx,temp_errory,temp_errorz;
	Eular Angle_Error;
	
	static Vector LastErrorRate = {0};
	static Vector P_ErrorRate = {0};
  static Vector D_ErrorRate = {0};
  static Vector I_ErrorRate = {0};
	
	throttle = command_throttle;
	
	temp_errorx	= Warp_ToPI(demand_angle.roll - current_angle.roll);
	temp_errory	= Warp_ToPI(demand_angle.pitch - current_angle.pitch);
	temp_errorz	= Warp_ToPI(demand_angle.yaw	- current_angle.yaw);
	/*Limit*/
	Angle_Error.roll  = Constraint_f(temp_errorx, -45.0f * DEG2RAD, +45.0f * DEG2RAD);
	Angle_Error.pitch =	Constraint_f(temp_errory, -45.0f * DEG2RAD, +45.0f * DEG2RAD);
	Angle_Error.yaw   =	Constraint_f(temp_errorz, -45.0f * DEG2RAD, +45.0f * DEG2RAD);
	
	ErrorRate.x = KpAngle.x * Angle_Error.roll - gyro->x;
	ErrorRate.y = KpAngle.y * Angle_Error.pitch - gyro->y;
	ErrorRate.z = KpAngle.z * Angle_Error.yaw - gyro->z;
	
	P_ErrorRate.x = DLPF(ErrorRate.x, P_ErrorRate.x, 30.0f * HZ2RAD, dt);
  P_ErrorRate.y = DLPF(ErrorRate.y, P_ErrorRate.y, 30.0f * HZ2RAD, dt);
  P_ErrorRate.z = DLPF(ErrorRate.z, P_ErrorRate.z, 30.0f * HZ2RAD, dt);
	
	D_ErrorRate.x = DLPF((ErrorRate.x - LastErrorRate.x) / dt, D_ErrorRate.x, 15.0f * HZ2RAD, dt);
  D_ErrorRate.y = DLPF((ErrorRate.y - LastErrorRate.y) / dt, D_ErrorRate.y, 15.0f * HZ2RAD, dt);
  D_ErrorRate.z = DLPF((ErrorRate.z - LastErrorRate.z) / dt, D_ErrorRate.z, 15.0f * HZ2RAD, dt);
	
  LastErrorRate.x = ErrorRate.x;
  LastErrorRate.y = ErrorRate.y;
  LastErrorRate.z = ErrorRate.z;
	
	I_ErrorRate.x = Constraint_f(I_ErrorRate.x + dt * ErrorRate.x, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);
  I_ErrorRate.y = Constraint_f(I_ErrorRate.y + dt * ErrorRate.y, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);
  I_ErrorRate.z = Constraint_f(I_ErrorRate.z + dt * ErrorRate.z, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);
	
	/* when a_bg is smaller than 60 degree */
  /* Thrust compensate .  Notice: too big Thrust will make output saturated*/
	if(throttle>0.05)
	{
		control_data->Mx = KpRate.x * P_ErrorRate.x + KiRate.x * I_ErrorRate.x + KdRate.x * D_ErrorRate.x;
		control_data->My = KpRate.y * P_ErrorRate.y + KiRate.y * I_ErrorRate.y + KdRate.y * D_ErrorRate.y;
		control_data->Mz = KpRate.z * P_ErrorRate.z + KiRate.z * I_ErrorRate.z + KdRate.z * D_ErrorRate.z;
		control_data->Thrust = Constraint_f(throttle + g_AdjustThrrote,0,0.8);
	}
	else
	{
		control_data->Mx = 0;
		control_data->My = 0;
		control_data->Mz = 0;
		I_ErrorRate.x = I_ErrorRate.y = I_ErrorRate.z = 0;
		control_data->Thrust = Constraint_f(throttle + g_AdjustThrrote,0,0.8);
	}
}
/**
  * @brief  Position control
  * @param  Vector* position,Vector* target_position,Ctrl_DataType*control_data,float dt
  * @retval None
  */
void Position_Control(Vector* position,Vector* target_position,Ctrl_DataType*control_data,float dt,uint8_t mode)
{
	static Vector t_position_old,t_position_dold;
	static Vector t_position_d,t_position_dd;
	
	static Vector error_pos_p,error_pos_d,error_pos_i;
	static Vector des_acc,des_acc_b;
	/*update t_position_d*/
	t_position_d.x = (target_position->x - t_position_old.x) / dt;
	t_position_d.y = (target_position->y - t_position_old.y) / dt;
	t_position_d.z = (target_position->z - t_position_old.z) / dt;
	/*update t_position_dd*/
	t_position_dd.x = (t_position_d.x - t_position_dold.x) / dt;
	t_position_dd.y = (t_position_d.y - t_position_dold.y) / dt;
	t_position_dd.z = (t_position_d.z - t_position_dold.z) / dt;

	t_position_old.x = target_position->x;
	t_position_old.y = target_position->y;
	t_position_old.z = target_position->z;

	t_position_dold.x = t_position_d.x;
	t_position_dold.y = t_position_d.y;
	t_position_dold.z = t_position_d.z;

	error_pos_p.x = target_position->x - position->x;
	error_pos_p.y = target_position->y - position->y;
	error_pos_p.z = target_position->z - position->z;

	error_pos_d.x = - g_VelRef.x;
	error_pos_d.y = - g_VelRef.y;
	error_pos_d.z = - g_VelRef.z;

	error_pos_i.x = Constraint_f((error_pos_p.x + error_pos_i.x) * dt,-20 * DEG2RAD,20 * DEG2RAD);
	error_pos_i.y = Constraint_f((error_pos_p.y + error_pos_i.y) * dt,-20 * DEG2RAD,20 * DEG2RAD);
	error_pos_i.z = Constraint_f((error_pos_p.z + error_pos_i.z) * dt,-0.1,0.1);

	des_acc.x = CONTROLER_KD.x * error_pos_d.x + CONTROLER_KP.x * error_pos_p.x + CONTROLER_KI.x * error_pos_i.x;
	des_acc.y = CONTROLER_KD.y * error_pos_d.y + CONTROLER_KP.y * error_pos_p.y + CONTROLER_KI.y * error_pos_i.y;
	des_acc.z = CONTROLER_KD.z * error_pos_d.z + CONTROLER_KP.z * error_pos_p.z + CONTROLER_KI.z * error_pos_i.z;

	des_acc_b.x = cosf(g_Eular.yaw) * des_acc.x + sinf(g_Eular.yaw) * des_acc.y;
	des_acc_b.y = -sinf(g_Eular.yaw) * des_acc.x + cosf(g_Eular.yaw) * des_acc.y;
	des_acc_b.z = des_acc.z;
	
	if(mode == Status_Original)
	{
		g_NavEuler.roll =0;
		g_NavEuler.pitch = 0;
		g_AdjustThrrote = 0;
		error_pos_i.x = error_pos_i.y = error_pos_i.z = 0;
	}
	if(mode == Status_Hovering)
	{
		g_AdjustThrrote = des_acc_b.z;
		g_NavEuler.roll = Constraint_f(asinf(-des_acc_b.y / 9.8f),-20 * DEG2RAD,20 * DEG2RAD);
		g_NavEuler.pitch = Constraint_f(asinf(des_acc_b.x / (9.8f * cosf(g_Eular.pitch))),-20 * DEG2RAD,20 * DEG2RAD);
	}
}

