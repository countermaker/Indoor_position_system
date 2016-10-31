#ifndef APP_H_
#define APP_H_
#include "mcu.h"
#include "global.h"
void Data_Update(void);

void Startup(void);

void Quadrotor_Control(float control_dt,int16_t mode,Vector*acc,Vector*gyro,Vector sensor_data,Eular targeteuler,Remote_Command commmand);

void Lock_Control(Remote_Command *Command);

void Com_AttPosGet(void);

#endif
