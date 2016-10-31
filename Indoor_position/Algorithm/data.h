#ifndef DATA_H_
#define DATA_H_

#include "mcu.h"
#include "mpu6050.h"
#include "delay.h"
#include "math.h"
#include "receive.h"

/*basic data*/
#define RtA 		57.2957795f				
#define AtR    	0.017453293f		        	
#define Acc_G 	0.0011963			
#define G 			-9.8f
#define G2      -19.6f
#define G_2			16384.0f
#define G_FAB   9.8f
typedef struct
{
	float x;
	float y;
	float z;
}Vector;

typedef struct 
{
	float P;
	float I;
	float D;
	float DD;
}PID_Factor;

typedef struct 
{
	float Throttle;
	u16 Extra;
	float Roll;
	float Pitch;
	float Yaw;
	float meter;
}Remote_Command;

void Imu_DataGet(Vector*acc,Vector*gyro);

void Mpu_AccOffest(void);

void Mpu_GyroOffest(void);

void ReceiveData_Norm(Remote_Command *Command,Chx_Width *p);

void Remote_ChxGet(void);

void ReceiveData_Convert(Remote_Command *Command_Uint,Remote_Command *Command);

void Remote_CommandGet(Remote_Command *Command);

void Remote_StatusGet(void);

void Sonar_DataGet(void);

void Sonar_DistanceGet(float*sensor);

void Data_Init(void);

void Sonar_DataDrive(void);

void Px4flow_DataGet(float* px4_x,float* px4_y);

uint8_t checksum(uint8_t *data, uint16_t len);

#endif

