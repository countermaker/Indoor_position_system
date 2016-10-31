/**
  ******************************************************************************
  * @file    app.c
  * @author  counter
  * @version V3.1
  * @date    2016-6-24
  * @brief   This file provides the api function for quad
  ******************************************************************************
  * @attention none
*/
/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "mcu.h"
#include "global.h"
#include "data.h"
#include "quad_math.h"
#include "math.h"
#include "bsp.h"
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>
int32_t ulock_time,lock_time;
/**
  * @brief  Update the data(background)
  * @param  None
  * @retval None
  */
void Data_Update()
{
	static float data_dt;
	static uint16_t update_loop;
	static float print_period,led_loopcnt,command_loopcnt,uart_loopcnt;

	data_dt = G_GetDt(&update_loop);
	
	print_period += data_dt;
	led_loopcnt += data_dt;
	command_loopcnt += data_dt;
	uart_loopcnt += data_dt;
	/*task*/
	/*task 1*/
	if(command_loopcnt >= 0.02)
	{
		Remote_CommandGet(&Command);
		command_loopcnt = 0;
	}
	/*task 2*/
	if(led_loopcnt >= 0.2f)
	{
		Led_Transpose0();
		led_loopcnt = 0;
	}
	/*task 3*/
	if(print_period >= 0.02f)
	{
		//printf("%f,%f,%f,%f,\r\n",Command.Yaw,Command.Pitch,Command.Roll,Command.Throttle);
		//printf("%f,%f,%f,\r\n",g_NavEuler.yaw * 57.3,g_NavEuler.pitch* 57.3,g_NavEuler.roll* 57.3);
		//printf("%f,%f,%f,\r\n",g_PosRef.x,g_PosRef.y,g_PosRef.z);
		printf("%f,%f,\r\n",g_PosRef.z,g_SensorUpdate.z);
		//printf("%f,%f,%f,\r\n",g_Acc.x,g_Acc.y,g_Acc.z);
		//printf("%f,%f,%f,\r\n",g_SensorUpdate.x,g_SensorUpdate.y,g_SensorUpdate.z);
		//printf("%f,%f,%f,\r\n",g_Gyro.x,g_Gyro.y,g_Gyro.z);
		//printf("%f,\r\n",g_SensorUpdate.z);
		//printf("%f,%f,\r\n",data_dt,sonar_loopcnt);
		//printf("%f,%f,%d,\r\n",g_PosRef.z,g_SensorUpdate.z,Image_qual);
		//printf("%f,\r\n",sonar_data);
		//printf("%f,%f,%d,\r\n",g_VelRef.x,g_PosRef.x,Image_qual);
		//printf("%f,\r\n",g_Sensor.x);
		//printf("%f,%f,\r\n",x7,g_Sensor.x);
		//printf("%f,%f,%f,\r\n",g_TargetEuler.pitch,g_Eular.pitch,g_VelRef.x);
		//printf("%d,\r\n",Image_qual);
		//printf("%f,%f,%f,\r\n",g_PosRef.x,g_VelRef.x,g_SensorUpdate.x);
		//printf("%f,%f,%f,%f,%f,\r\n",current_flowx,x1,current_flowy,x2,g_PosRef.z);
		//printf("%f,%f,\r\n",g_TargetEuler.pitch,g_TargetEuler.roll);
		//printf("%f,%f,%f,%d,\r\n",g_TargetEuler.pitch,g_Eular.pitch,g_VelRef.x,Image_qual);
		//printf("%f,\r\n",x1);
		//printf("%f,%f,%f,\r\n",g_SensorUpdate.z,g_PosRef.z,sonar_data);
		//printf("%d,\r\n",Image_qual);
		//printf("%f,\r\n",current_sonar2);
		//printf("%f,\r\n",x6);
		//printf("%f,%f,\r\n",g_SensorUpdate.x,g_SensorUpdate.y);
		print_period=0;
	}
	/*judgment*/
	if(!lock)
	{
		Led_On1();
	}
	else
	{
		Led_Off1();
	}
	if(Flag_UartRx == 1)
	{
		Flag_UartRx = 0;
		Com_AttPosGet();
	}
}
/**
  * @brief  System startup
  * @param  None
  * @retval None
  */
void Startup(void)
{
	Hardware_Init();
}
/**
  * @brief  Control quadrotor
  * @param  control_dt mode acc gyro sensor_data TargetEuler commmand
  * @retval None
  */
void Quadrotor_Control(float control_dt,int16_t mode,Vector*acc,Vector*gyro,Vector sensor_data,Eular targeteuler,Remote_Command commmand)
{
	Imu_DataGet(acc,gyro);
	Matrix_EstimateEuler(*acc,*gyro,control_dt);
	Estimate_Status(acc,sensor_data,control_dt,g_Eular);
	Position_Control(&g_PosRef,&g_TargetPos,&Control_Data,control_dt,mode);
	if(System_Status != Status_Auto)
	{
		g_TargetEuler.pitch = commmand.Pitch + g_NavEuler.pitch;
		g_TargetEuler.roll = commmand.Roll + g_NavEuler.roll;
		g_TargetEuler.yaw = commmand.Yaw;
		g_BaseThrrote = commmand.Throttle; /*0-1*/
	}
	Eular_Control(targeteuler,g_Eular,gyro,acc,control_dt,&Control_Data,g_BaseThrrote);
	Moto_SetPwm(&Control_Data);
}
/**
  * @brief  Lock_Control
  * @param  Command
  * @retval None
  */
void Lock_Control(Remote_Command *command)
{
	if(command->Pitch<=0.1&&command->Roll<=0.1&&command->Yaw>=0.9&&command->Throttle<=0.1)
	{
		ulock_time++;
		if(command->Pitch<=0.1&&command->Roll<=0.1&&command->Yaw>=0.9&&command->Throttle<=0.1&&ulock_time>800)
		{
			lock=0;
			Command.Yaw = g_Eular.yaw;
		}
	}
	else
		ulock_time=0;
	if(command->Pitch<=0.1&&command->Roll<=0.1&&command->Yaw<=0.1&&command->Throttle<=0.1)
	{
			lock_time++;
		if(command->Pitch<=0.1&&command->Roll<=0.1&&command->Yaw<=0.1&&command->Throttle<=0.1&&lock_time>800)
			lock=1;
	}
	else
		lock_time=0;
}
/**
  * @brief  Get the attitude and position data from computer
  * @param  None
  * @retval None
  */
void Com_AttPosGet(void)
{
	static uint16_t timestamp_old = 0;
	static float dt;
	static uint8_t yaw_init = 0;
	
	uint8_t buf[COMM_PACKET_BUF_LEN];
	
	sw2b_t packet_x, packet_y,packet_z,packet_yaw,packet_pitch,packet_roll;
	sw2b_t packet_target_x,packet_target_y,packet_target_z;
	Vector marker_in_body = {-0.002,-0.00,0.055};
	
	Vector marker_in_b1;
	Vector marker_in_ref;
	
	static Vector cam_pos = {0,0,0};
  static Vector cam_vel = {0,0,0};
  static Vector last_cam_pos = {0,0,0};
	
	Eular euler_att;
	
	euler_att.yaw = g_Eular.yaw;
	euler_att.pitch = g_Eular.pitch;
	euler_att.roll = g_Eular.roll;
	
	dt = G_GetDt(&timestamp_old);
	
	memcpy((void*)buf, (void*)g_packet_buf, COMM_PACKET_BUF_LEN);
	
	packet_x.b[0] = buf[1];
  packet_x.b[1] = buf[2];

  packet_y.b[0] = buf[3];
  packet_y.b[1] = buf[4];
			
	packet_z.b[0] = buf[5];
  packet_z.b[1] = buf[6];
			
	packet_yaw.b[0] = buf[7];
  packet_yaw.b[1] = buf[8];
			
	packet_pitch.b[0] = buf[9];
  packet_pitch.b[1] = buf[10];
			
	packet_roll.b[0] = buf[11];
  packet_roll.b[1] = buf[12];
			
	packet_target_x.b[0] = buf[13];
	packet_target_x.b[1] = buf[14];
			
	packet_target_y.b[0] = buf[15];
	packet_target_y.b[1] = buf[16];
			
	packet_target_z.b[0] = buf[17];
	packet_target_z.b[1] = buf[18];
	if (checksum(&buf[1], COMM_PACKET_BUF_LEN-3) == buf[COMM_PACKET_BUF_LEN-2])
	{
		Eular_RoateVect(&marker_in_body, &marker_in_b1, &euler_att);
		
		marker_in_ref.x = packet_x.w/1000.0f;
    marker_in_ref.y = packet_y.w/1000.0f;
		marker_in_ref.z = packet_z.w/1000.0f;
		
		cam_pos.x = DLPF(marker_in_ref.x - marker_in_b1.x,cam_pos.x,10*HZ2RAD,dt);
    cam_pos.y = DLPF(marker_in_ref.y - marker_in_b1.y,cam_pos.y,10*HZ2RAD,dt);
		cam_pos.z = DLPF(marker_in_ref.z - marker_in_b1.z,cam_pos.z,10*HZ2RAD,dt);
					
		g_CamEuler.yaw = DLPF(packet_yaw.w/1000.0f,g_CamEuler.yaw,10*HZ2RAD,dt);
		g_CamEuler.pitch = DLPF(packet_pitch.w/1000.0f,g_CamEuler.pitch,10*HZ2RAD,dt);
		g_CamEuler.roll = DLPF(packet_roll.w/1000.0f,g_CamEuler.roll,10*HZ2RAD,dt);
				
		cam_vel.x = (cam_pos.x - last_cam_pos.x)/dt;
    cam_vel.y = (cam_pos.y - last_cam_pos.y)/dt;
    cam_vel.z = (cam_pos.z - last_cam_pos.z)/dt;
	
	  last_cam_pos.x = cam_pos.x;
    last_cam_pos.y = cam_pos.y;
	  last_cam_pos.z = cam_pos.z;
					
		g_CamPos.x = cam_pos.x;
		g_CamPos.y = cam_pos.y;
		g_CamPos.z = cam_pos.z;
					
		g_CamVel.x = cam_vel.x;
		g_CamVel.y = cam_vel.y;
		g_CamVel.z = cam_vel.z;
					
		g_CamTargetPos.x = DLPF(packet_target_x.w/1000.0f,g_CamTargetPos.x,1.5f*HZ2RAD,dt);
		g_CamTargetPos.y = DLPF(packet_target_y.w/1000.0f,g_CamTargetPos.y,1.5f*HZ2RAD,dt);
		g_CamTargetPos.z = DLPF(packet_target_z.w/1000.0f,g_CamTargetPos.z,1.5f*HZ2RAD,dt);
		
		g_SensorUpdate.x = cam_pos.x;
		g_SensorUpdate.y = cam_pos.y;
		g_SensorUpdate.z = cam_pos.z * 0.65;
		
		g_FlagComUpdate = SET;
		g_FlagComUpdateEuler = SET;
		
		if(yaw_init == 0)
		{
			yaw_init = 1;
			g_Eular.yaw = packet_yaw.w/1000.0f;
		}
	}
}
