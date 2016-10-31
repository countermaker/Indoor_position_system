#include "bsp.h"

/**
  * @brief  Init the system
  * @param  None
  * @retval None
  */
void Hardware_Init(void)
{
	Uart1_Init(115200);
	printf("UART OK!\r\n");
	
	Led_Init(); 
	printf("LED OK!\r\n");
	
	Timer_Init(0xffff);
	printf("TIMER OK!\r\n");
	
  TimerCapture_Init();
	printf("REMOTE CONTROL OK!\r\n");
	
  Pwm_init(); 
	Moto_Clear();
	printf("MOTOR OK!\r\n");
	
	
//	IIC2_Init();
//	printf("Px4 OK!\r\n");
	
	IIC_Init();
	MPU_Init();
	printf("MPU OK!\r\n");
	
	Data_Init();
	printf("GYRO OFFECT OK!\r\n");
	
	Exti_Init();
	printf("FOREGROUND TASK OK!\r\n");
	
	//Measure_TimeInit();
	CORDIC_Init();
	
	Sonar_GpioInit();
	printf("SONAR OK!\r\n");
	
	printf("HARDWARE INITAILIZE OK!");
	Timer_Control(1);
}

/**
  * @brief  Init the ESC
  * @param  Command
  * @retval None
  */
void Esc_Init(Remote_Command*Command)
{
	if(Command->Throttle<0.5)
	{
		Moto_Clear();
	}
	else
	{
		Moto_Full();
	}
}
