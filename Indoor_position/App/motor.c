#include "motor.h"

/**
  * @brief  Change the PWM 
  * @param  None
  * @retval None
  */
void Moto_PwmRflash(Moto_Pwm* MOTO_PWM)
{		
	if(MOTO_PWM->MOTO1>MOTO_MAX)	MOTO_PWM->MOTO1 = MOTO_MAX;
	if(MOTO_PWM->MOTO2>MOTO_MAX)	MOTO_PWM->MOTO2 = MOTO_MAX;
	if(MOTO_PWM->MOTO3>MOTO_MAX)	MOTO_PWM->MOTO3 = MOTO_MAX;
	if(MOTO_PWM->MOTO4>MOTO_MAX)	MOTO_PWM->MOTO4 = MOTO_MAX;
	
	if(MOTO_PWM->MOTO1<0)	MOTO_PWM->MOTO1 = 0;
	if(MOTO_PWM->MOTO2<0)	MOTO_PWM->MOTO2 = 0;
	if(MOTO_PWM->MOTO3<0)	MOTO_PWM->MOTO3 = 0;
	if(MOTO_PWM->MOTO4<0)	MOTO_PWM->MOTO4 = 0;
	/*1ms-2ms*/
	if(!lock && System_Status != Status_Flameout)
	{
		Pwm_Set1(MOTO_PWM->MOTO1 + MOTO_BASE); //4 
		Pwm_Set2(MOTO_PWM->MOTO2 + MOTO_BASE); //1
		Pwm_Set3(MOTO_PWM->MOTO3 + MOTO_BASE); //2
		Pwm_Set4(MOTO_PWM->MOTO4 + MOTO_BASE); //3
		Led_On0();
	}
	else
	{
		Pwm_Set1(MOTO_BASE); //1 
		Pwm_Set2(MOTO_BASE); //2
		Pwm_Set3(MOTO_BASE); //3
		Pwm_Set4(MOTO_BASE); //4
		Led_Off0();
	}
}
/**
  * @brief  Set the PWM 
  * @param  control_data
  * @retval None
  */
void Moto_SetPwm(Ctrl_DataType*control_data)
{
	float w1,w2,w3,w4;
	static Moto_Pwm Result;
	
	control_data->Thrust=Constraint_f(control_data->Thrust,0.0f,0.60f);
	
	w1= Constraint_f(control_data->Thrust + control_data->Mx - control_data->My - control_data->Mz,0,1);
	w2= Constraint_f(control_data->Thrust + control_data->Mx + control_data->My + control_data->Mz,0,1);
	w3= Constraint_f(control_data->Thrust - control_data->Mx + control_data->My - control_data->Mz,0,1);
	w4= Constraint_f(control_data->Thrust - control_data->Mx - control_data->My + control_data->Mz,0,1);
	
	Result.MOTO1=(int16_t)(w1 * PWM_WITH + 0.5f);
	Result.MOTO2=(int16_t)(w2 * PWM_WITH + 0.5f);
	Result.MOTO3=(int16_t)(w3 * PWM_WITH + 0.5f);
	Result.MOTO4=(int16_t)(w4 * PWM_WITH + 0.5f);
	
	Moto_PwmRflash(&Result);
}
/**
  * @brief  Clear the Moto
  * @param  None
  * @retval None
  */
void Moto_Clear(void)
{
	Pwm_Set1(MOTO_BASE); //1 
	Pwm_Set2(MOTO_BASE); //2
	Pwm_Set3(MOTO_BASE); //3
	Pwm_Set4(MOTO_BASE); //4
}
/**
  * @brief  Full the Moto
  * @param  None
  * @retval None
  */
void Moto_Full(void)
{
	Pwm_Set1(MOTO_BASE+MOTO_MAX); //1 
	Pwm_Set2(MOTO_BASE+MOTO_MAX); //2
	Pwm_Set3(MOTO_BASE+MOTO_MAX); //3
	Pwm_Set4(MOTO_BASE+MOTO_MAX); //4
}
/**
  * @brief  Moto Smooth
  * @param  None
  * @retval None
  */
void Moto_Smooth(Moto_Pwm *motor)
{
	static uint16_t last_motor1 = MOTO_BASE;
	static uint16_t last_motor2 = MOTO_BASE;
	static uint16_t last_motor3 = MOTO_BASE;
	static uint16_t last_motor4 = MOTO_BASE;
	
	if((motor->MOTO1) > last_motor1)
	{
		motor->MOTO1 = (last_motor1 + 1 * (motor->MOTO1)) / 2;
	}
	else
	{
		motor->MOTO1 = (motor->MOTO1) - (last_motor1 - (motor->MOTO1)) * 1;
	}
	
	if((motor->MOTO2) > last_motor2)
	{
		motor->MOTO2 = (last_motor2 + 1 * (motor->MOTO2)) / 2;
	}
	else
	{
		motor->MOTO2 = (motor->MOTO2) - (last_motor2 - (motor->MOTO2)) * 1;
	}
	
	if((motor->MOTO3) > last_motor3)
	{
		motor->MOTO3 = (last_motor3 + 1 * (motor->MOTO3)) / 2;
	}
	else
	{
		motor->MOTO3 = (motor->MOTO3) - (last_motor3 - (motor->MOTO3)) * 1;
	}
	
	if((motor->MOTO4) > last_motor4)
	{
		motor->MOTO4 = (last_motor4 + 1 * (motor->MOTO4)) / 2;
	}
	else
	{
		motor->MOTO4 = (motor->MOTO4) - (last_motor4 - (motor->MOTO4)) * 1;
	}
	
}
