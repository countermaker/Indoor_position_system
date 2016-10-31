#ifndef _GLOBAL_H_
#define _GLOBAL_H_
#include "mcu.h"
#include "data.h"

#define SET    1
#define RESET  0
#define SUCESS 1
#define FAIL   0
#define KALMAN_POS_K1_Z           (0.2) 
#define KALMAN_POS_K2_Z           (0.9)
#define KALMAN_POS_K3_Z           (-0.02)

#define KALMAN_POS_K1_X           (0.2)
#define KALMAN_POS_K2_X           (0.9)
#define KALMAN_POS_K3_X           (-0.02)

#define KALMAN_POS_K1_Y           (0.2)
#define KALMAN_POS_K2_Y           (0.9)
#define KALMAN_POS_K3_Y           (-0.02)

#define KALMAN_POS_K1_PITCH       (0.001)
#define KALMAN_POS_K2_PITCH       (0.006)

#define KALMAN_POS_K1_ROLL        (0.001)
#define KALMAN_POS_K2_ROLL        (0.006)

#define KALMAN_POS_K1_YAW       (0.06)


#define COMM_PACKET_BUF_LEN  			21 

#define Hover_Height             	(0.50f)

#define Status_Original  0
#define Status_Setheight  1
#define Status_Hovering  2
#define Status_Auto  3
#define Status_Land  4
#define Status_Flameout  5
typedef struct 
{
  float w;
  float x;
  float y;
  float z;
}	Quaternion;

typedef struct 
{
  float yaw;
  float pitch;
  float roll;
}	Eular;

typedef struct
{
	float Thrust;
	float Mx;
	float My;
	float Mz;
}Ctrl_DataType;

typedef union
{
  int16_t w;
	uint8_t b[2];
}sw2b_t;

typedef int16_t Status;

float G_GetDt(uint16_t* oldtime);

void G_TimeStart(void);
	
void G_TimeStop(void);

extern Vector g_AccRef;

extern Vector g_AccRefOffset;

extern uint8_t g_FlagAttInit;

extern Vector g_PosRef;

extern Vector g_VelRef;

extern Vector g_TargetPos;

extern Vector g_Acc;

extern Vector g_Gyro;

extern Vector g_Sensor;

extern Vector g_SensorUpdate;

extern Eular g_NavEuler;

extern Vector g_CamPos;

extern Vector g_CamVel;

extern Vector g_CamTargetPos;

extern Eular g_Eular;

extern Eular g_TargetEuler;

extern Eular g_CamEuler;

extern uint8_t g_FlagSonarUpdate;

extern uint8_t g_FlagPx4Updatex;

extern uint8_t g_FlagPx4Updatey;

extern uint8_t g_FlagEnableControlX;

extern uint8_t g_FlagEnableControlY;

extern uint8_t g_FlagComUpdate;

extern uint8_t g_FlagComUpdateEuler;

extern Status System_Status;

extern int16_t y1,y2,y3,y4,y5;

extern int16_t Image_qual;

extern Ctrl_DataType Control_Data;

extern float g_BaseThrrote;

extern float g_AdjustThrrote;

extern Remote_Command Command_Uint,Command; 

extern int16_t lock;

extern int16_t p1,p2,p3,p4;

extern int16_t y1,y2,y3,y4,y5;

extern float x1,x2,x3,x4,x5,x6,x7,x8,x9;

extern volatile uint8_t g_packet_buf[COMM_PACKET_BUF_LEN];

extern uint8_t Flag_UartRx;

#endif
