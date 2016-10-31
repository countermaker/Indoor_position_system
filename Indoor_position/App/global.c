#include "global.h"
#include "rotate.h"

Vector g_AccRef = {0,0,0};         
Vector g_AccRefOffset = {0,0,9.8};  
Vector g_PosRef = {0,0,0};          
Vector g_VelRef = {0,0,0};				
Vector g_TargetPos = {0.0f,0.0f,0.0f};
Eular g_NavEuler = {0,0,0};
Vector g_Acc = {0,0,0};
Vector g_Gyro = {0,0,0};
Vector g_Sensor = {0,0,0};
Vector g_SensorUpdate = {0,0,0};

Vector g_CamPos = {0,0,0};
Vector g_CamVel = {0,0,0};
Vector g_CamTargetPos = {0,0,0};

Eular  g_TargetEuler = {0,0,0};
Eular g_Eular = {0,0,0};
Eular g_CamEuler = {0,0,0};
float g_BaseThrrote = 0;
float g_AdjustThrrote = 0;
int16_t Image_qual;
int16_t lock = 1;

Remote_Command Command_Uint,Command; 

int16_t y1,y2,y3,y4,y5;
int16_t p1,p2,p3,p4;
float x1,x2,x3,x4,x5,x6,x7,x8,x9;

uint8_t g_FlagSonarUpdate = RESET;
uint8_t g_FlagAttInit = RESET;
uint8_t g_FlagPx4Updatex = RESET;
uint8_t g_FlagPx4Updatey = RESET;
uint8_t g_FlagComUpdate = RESET;
uint8_t g_FlagComUpdateEuler = RESET;
uint8_t g_FlagEnableControlX = SET;
uint8_t g_FlagEnableControlY = SET;

volatile uint8_t g_packet_buf[COMM_PACKET_BUF_LEN];
uint8_t Flag_UartRx = 0;

Status System_Status = 0;
Ctrl_DataType Control_Data;

/**
  * @brief  Get Dt
  * @param  uint16_t* oldtime
  * @retval Dt
  */
float G_GetDt(uint16_t* oldtime)
{
	uint16_t New,Old;
	New = TIM_GetCounter(TIM1);
	Old = *oldtime;
	*oldtime = New;
	if(New > Old)
	{
		return (float)((New-Old)/1000000.0f);
	}
	else
	{
		return (float)((New-Old+0xffff+1)/1000000.0f);
	}
}

void G_TimeStart(void)
{
	Measure_TimeStart();
}

void G_TimeStop(void)
{
	Measure_TimeStop();
}
