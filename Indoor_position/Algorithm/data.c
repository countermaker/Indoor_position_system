#include "data.h"  
#include "rotate.h"
#include "global.h"
#include "mpu6050.h"
#include "receive.h"
#include "sonar.h"
#include "px4.h"

int16_t Acc_Offsetx=-263,Acc_Offsety=-193,Acc_Offsetz=4631;
float Gyro_Offsetx,Gyro_Offsety,Gyro_Offsetz;
float Kaccx,Kaccy,Kaccz;

float Sonar_Width,Sonar2_Width;;

Chx_Width Chx;

float Flow_x,Flow_y;
Mpu6050_Type Mpu6050_Data;

/**
  * @brief  mpu6050 original data handle
  * @param  Mpu6050_Data ax ay az gx gy gz
  * @retval None
  */
void Mpu_DataHandle(Mpu6050_Type*Mpu6050_Data,Vector*acc,Vector*gyro)
{
	float temp_gx,temp_gy,temp_gz;
	temp_gx=Mpu6050_Data->gyro_x;	//radian
	temp_gy=Mpu6050_Data->gyro_y;	//radian
	temp_gz=Mpu6050_Data->gyro_z;	//radian
	
	gyro->x=  (temp_gy/GYRO_SENSITIVITY*AtR-Gyro_Offsety);
	gyro->y= - (temp_gx/GYRO_SENSITIVITY*AtR-Gyro_Offsetx);
	gyro->z=temp_gz/GYRO_SENSITIVITY*AtR-Gyro_Offsetz;
	
	acc->y= - (float)(Mpu6050_Data->acc_x-Acc_Offsetx)/ACC_SENSITIVITY *G_FAB;
	acc->x=(float)(Mpu6050_Data->acc_y-Acc_Offsety)/ACC_SENSITIVITY *G_FAB;
	acc->z=(float)(Mpu6050_Data->acc_z-Acc_Offsetz)/ACC_SENSITIVITY *G_FAB;
}

/**
  * @brief  Offest the Mpu_Acc
  * @param  None
  * @retval None
  */
void Mpu_AccOffest(void)
{
	int16_t cnt_g=1000;
	Mpu6050_Type Mpu6050_Data;
	static double offset_x1,offset_x2,offset_y1,offset_y2,offset_z1,offset_z2;
	Delay_ms(4000);
//	int16_t Mpu_Temp[3]; 
	printf("please correct the z axis.\r\n");
	Delay_ms(6000);
	printf("Correcting ... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_z1 += Mpu6050_Data.acc_z;
		Delay_ms(10);
	}
	offset_z1/=1000.0f;
	
	printf("Z1 = %f,\r\n",offset_z1);
	
	cnt_g=1000;
	printf("please overturn.\r\n");
	Delay_ms(6000);
	printf("Correcting... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_z2+=Mpu6050_Data.acc_z;
		Delay_ms(10);
	}
	offset_z2/=1000.0f;
	
	printf("Z2 = %f,\r\n",offset_z2);
	
	cnt_g=1000;
	printf("please correct the y axis.\r\n");
	Delay_ms(6000);
	printf("Correcting... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_y1+=Mpu6050_Data.acc_y;
		Delay_ms(10);
	}
	offset_y1/=1000.0f;
	
	printf("Y1 = %f,\r\n",offset_y1);
	
	cnt_g=1000;
	printf("please overturn\r\n");
	Delay_ms(6000);
	printf("Correcting... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_y2+=Mpu6050_Data.acc_y;
		Delay_ms(10);
	}
	offset_y2/=1000.0f;
	
	printf("Y2 = %f,\r\n",offset_y2);
	
	cnt_g=1000;
	printf("please correct the x axis.\r\n");
	Delay_ms(6000);
	printf("Correcting... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_x1+=Mpu6050_Data.acc_x;
		Delay_ms(10);
	}
	offset_x1/=1000.0f;
	
	printf("X1 = %f,\r\n",offset_x1);
	
	cnt_g=1000;
	printf("please overturn.\r\n");
	Delay_ms(6000);
	printf("Correcting... ...\r\n");
	while(cnt_g--)
	{
		MPU_Getaccelerometergyroscope(&Mpu6050_Data);
		offset_x2+=Mpu6050_Data.acc_x;
		Delay_ms(10);
	}
	offset_x2/=1000.0f;
	
	printf("X2 = %f,\r\n",offset_x2);
	
	Kaccx=(float)fabs(offset_x2-offset_x1)/G_2;
	Kaccy=(float)fabs(offset_y2-offset_y1)/G_2;
	Kaccz=(float)fabs(offset_z2-offset_z1)/G_2;
	
	Acc_Offsetx=(offset_x2+offset_x1)/2.0f;
	Acc_Offsety=(offset_y2+offset_y1)/2.0f;
	Acc_Offsetz=(offset_z2+offset_z1)/2.0f;
	
	//Mpu_Temp[0]=Acc_Offsetz;
	//Mpu_Temp[1]=Acc_Offsety;
	//Mpu_Temp[2]=Acc_Offsetx;
	printf("Offsetx:%d,Offsety:%d,Offsetz:%d\r\n",Acc_Offsetx,Acc_Offsety,Acc_Offsetz);//-292,132,4666
	printf("Kaccx:%f,Kaccy:%f,Kaccz:%f\r\n",Kaccx,Kaccy,Kaccz);//0.999045,0.958949,1.006267
}
/**
  * @brief  Offest the Mpu_Gyo
  * @param  None
  * @retval None
  */
void Mpu_GyroOffest(void)
{
	const int32_t cnt_g=500;
	int16_t i;
	float offset_temp1,offset_temp2,offset_temp3;
	Mpu6050_Type Mpu6050Data;
	
	do
	{
		offset_temp1 = offset_temp2 = offset_temp3 =0;
		Gyro_Offsetx = Gyro_Offsety = Gyro_Offsetz = 0;
		for(i=0;i<cnt_g;i++){
			MPU_Getaccelerometergyroscope(&Mpu6050Data);
			offset_temp1=(float)Mpu6050Data.gyro_x/GYRO_SENSITIVITY*AtR;
			offset_temp2=(float)Mpu6050Data.gyro_y/GYRO_SENSITIVITY*AtR;
			offset_temp3=(float)Mpu6050Data.gyro_z/GYRO_SENSITIVITY*AtR;
		
			Gyro_Offsetx+=offset_temp1;
			Gyro_Offsety+=offset_temp2;
			Gyro_Offsetz+=offset_temp3;	
			Delay_ms(5);
		}
    Gyro_Offsetx/=(float)cnt_g;
	  Gyro_Offsety/=(float)cnt_g;
	  Gyro_Offsetz/=(float)cnt_g;
		
		offset_temp1 = offset_temp2 = offset_temp3 =0;
		
	  for(i=0;i<cnt_g;i++){
			MPU_Getaccelerometergyroscope(&Mpu6050Data);
			offset_temp1 +=(float)Mpu6050Data.gyro_x/GYRO_SENSITIVITY*AtR - Gyro_Offsetx;
			offset_temp2 +=(float)Mpu6050Data.gyro_y/GYRO_SENSITIVITY*AtR - Gyro_Offsety;
			offset_temp3 +=(float)Mpu6050Data.gyro_z/GYRO_SENSITIVITY*AtR - Gyro_Offsetz;
			Delay_ms(5);
		}
	}while((fabs(offset_temp1) > 0.15f) || (fabs(offset_temp2) > 0.15f) || (fabs(offset_temp3) > 0.15f));

}
/**
  * @brief  ReceiveData Normzation
  * @param  Remote_Command *Command,Chx_Width *p
  * @retval None
  */
void ReceiveData_Norm(Remote_Command *Command,Chx_Width *p)
{
	Command->Roll= 			Constraint_f((p->Ch1-999)/1000.0f- 0.017f,0.0,1.0);//1015
	Command->Pitch=			Constraint_f((p->Ch2-999)/1000.0f- 0.017f,0.0,1.0);
	Command->Throttle=	Constraint_f((p->Ch3-999)/1000.0f- 0.017f,0.0,1.0);
	Command->Yaw=				Constraint_f((p->Ch4-999)/1000.0f- 0.017f,0.0,1.0);
	
}
/**
  * @brief  ReceiveData transform
  * @param  Remote_Command *Command_Uint,Remote_Command *Command
  * @retval None
  */
void ReceiveData_Convert(Remote_Command *Command_Uint,Remote_Command *Command)
{
	Command->Throttle=	(Command_Uint->Throttle);	
	Command->Pitch   =	(Command_Uint->Pitch - 0.5f) * PI / 6.0 * 2.0f;
	Command->Roll    =	(Command_Uint->Roll - 0.5f) * PI / 6.0 * 2.0f;
	
	Command->Yaw = Constraint_f(Command->Yaw + 0.005*(Command_Uint->Yaw-0.499f) * PI * 2.0f,-PI,PI);
}

/**
  * @brief  Get control data from remote control
  * @param  Remote_Command *Command
  * @retval None
  */
void Remote_CommandGet(Remote_Command *Command)
{
	ReceiveData_Norm(&Command_Uint,&Chx);					//chx normalize
	ReceiveData_Convert(&Command_Uint,Command);		//Command_Uint -> Command 
}
void Remote_ChxGet(void)
{
	Reseave_IrqTIM2(&Chx);												//get the width of pulse
}
/**
  * @brief  Get imu data from sensor
  * @param  Vector*acc,Vector*gyro
  * @retval None
  */
void Imu_DataGet(Vector*acc,Vector*gyro)
{
	MPU_Getaccelerometergyroscope(&Mpu6050_Data);
	Mpu_DataHandle(&Mpu6050_Data,acc,gyro);
	g_Acc = *acc;
	g_Gyro = *gyro;
}
/**
  * @brief  Get Status data from remote control
  * @param  None
  * @retval None
  */
void Remote_StatusGet(void)
{
	static int16_t Thrrote;
	static int16_t flag; //reset flag
	Reseave_IrqTIM3(&Chx);
	switch(System_Status)
	{
		/*fly*/
		case Status_Original:
			Thrrote = Chx.Ch3;
		  g_TargetPos.x = 0;
			g_TargetPos.y = 0;
		  g_TargetPos.z = g_PosRef.z;
			if(Chx.Ch7 < 2000 && Chx.Ch7 > 1300 && flag == 0)
			{
				System_Status = Status_Hovering;
			}
			if(Chx.Ch7 > 2000)
			{
				flag = 0;
			}
			break;
		/*hover*/
		case Status_Hovering:
			if(Chx.Ch7 > 2000)
			{
				System_Status = Status_Original;
			}
			if(fabs(Thrrote - Chx.Ch3) >= 10)
			{
				System_Status = Status_Original;
				flag = 1;
			}
			break;
			default: break;
	}
}
/**
  * @brief  Get data from sonar
  * @param  float *Sonar_DistanceRaw
  * @retval None
  */
void Sonar_DataGet(void)
{
	Sonar_Getwidth(&Sonar_Width,&Chx);
}

void Sonar_DistanceGet(float*sensor)
{
	Sonar_GetDist(&Sonar_Width,sensor);
}
/**
  * @brief  Get data from Px4flow
  * @param  float *Sonar_DistanceRaw
  * @retval None
  */
void Px4flow_DataGet(float* px4_x,float* px4_y)
{
	PX4_FLOW_I2C_FRAME Px4_Data;
	PX4_FrameUpdate(&Px4_Data);
	
	Image_qual = Px4_Data.qual;
	
	Flow_y = -Px4_Data.pixel_flow_y_sum/10.0f;
	Flow_x = Px4_Data.pixel_flow_x_sum/10.0f;
	g_Sensor.x = 0.31f * -g_PosRef.z * (Flow_x / FOCUS - 3.5f * g_Gyro.y);
	g_Sensor.y = 0.31f * -g_PosRef.z * (Flow_y / FOCUS + 3.5f * g_Gyro.x);	

	*px4_x = g_Sensor.x;
	*px4_y = g_Sensor.y;
}
/**
  * @brief  Init the sensor data
  * @param  None
  * @retval None
  */
void Data_Init(void)
{
	Mpu_GyroOffest();
}
/**
  * @brief  Drive the sonar
  * @param  None
  * @retval None
  */
void Sonar_DataDrive(void)
{
	Sonar_Encourage();
}
/**
  * @brief  Check the sum
  * @param  None
  * @retval None
  */
uint8_t checksum(uint8_t *data, uint16_t len)
{
  uint32_t sum = 0;
  uint8_t i = 0;

  for (; len > 1; len -= 2)
  {
    sum += *data++;
    if (sum & 0x80000000)
      sum = (sum & 0xffff) + (sum >> 16);
  }

  if (len == 1)
  {
    *(uint8_t *)(&i) = *(uint8_t *)data;
    sum += i;
  }

  while (sum >> 16)
    sum = (sum & 0xffff) + (sum >> 16);

  return ((sum == 0xffff) ? sum : (~sum));
}
