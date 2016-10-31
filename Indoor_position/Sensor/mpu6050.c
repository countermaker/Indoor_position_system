#include "mpu6050.h"

	/**
  * @brief  Mpu6050 Writelen
  * @param  addr Mpu_reg Mpu_len Mpu_buf
  * @retval flat
  */
u8 MPU_Writelen(u8 Mpu_addr,u8 Mpu_reg,u8 Mpu_len,u8 *Mpu_buf)
{
	u8 i; 
  IIC_Start(); 
	IIC_Sendbyte((Mpu_addr<<1)|0);	
	if(IIC_Waitack())					
	{
		IIC_Stop();		 
		return 1;		
	}
  IIC_Sendbyte(Mpu_reg);				
  IIC_Waitack();						
	for(i=0;i<Mpu_len;i++)
	{
		IIC_Sendbyte(Mpu_buf[i]);		
		if(IIC_Waitack())				
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
  IIC_Stop();	 
	return 0;	
} 
/**
  * @brief  Mpu6050 Readlen
  * @param  addr Mpu_reg Mpu_len Mpu_buf
  * @retval flat
  */
u8 MPU_Readlen(u8 Mpu_addr,u8 Mpu_reg,u8 Mpu_len,u8 *Mpu_buf)
{ 
 	IIC_Start(); 
	IIC_Sendbyte((Mpu_addr<<1)|0);					
	if(IIC_Waitack())								
	{
		IIC_Stop();		 
		return 1;		
	}
  IIC_Sendbyte(Mpu_reg);							
  IIC_Waitack();									
  IIC_Start();
	IIC_Sendbyte((Mpu_addr<<1)|1);				
  IIC_Waitack();									
	while(Mpu_len)
	{
		if(Mpu_len==1)*Mpu_buf=IIC_Readbyte(0);
		else *Mpu_buf=IIC_Readbyte(1);			
		Mpu_len--;
		Mpu_buf++; 
	}    
  IIC_Stop();											
	return 0;	
}
/**
  * @brief  Mpu6050 write a byte
  * @param  addr Mpu_reg Mpu_len Mpu_buf
  * @retval flat
  */
u8 MPU_Writebyte(u8 Mpu_Reg,u8 Mpu_Data) 				 
{ 
  IIC_Start(); 
	IIC_Sendbyte((MPU_ADDR<<1)|0);	
	if(IIC_Waitack())							
	{
		IIC_Stop();		 
		return 1;		
	}
  IIC_Sendbyte(Mpu_Reg);						
  IIC_Waitack();								
	IIC_Sendbyte(Mpu_Data);						
	if(IIC_Waitack())							
	{
		IIC_Stop();	 
		return 1;		 
	}		 
  IIC_Stop();	 
	return 0;
}
/**
  * @brief  Mpu6050 read a byte
  * @param  addr Mpu_reg Mpu_len Mpu_buf
  * @retval flat
  */
u8 MPU_Readbyte(u8 Mpu_Reg)
{
	u8 Mpu_Res;
	IIC_Start(); 
	IIC_Sendbyte((MPU_ADDR<<1)|0);	
	IIC_Waitack();									
	IIC_Sendbyte(Mpu_Reg);						
	IIC_Waitack();								
	IIC_Start();
	IIC_Sendbyte((MPU_ADDR<<1)|1);		
	IIC_Waitack();								
	Mpu_Res=IIC_Readbyte(0);						
	IIC_Stop();										
	return Mpu_Res;		
}

/**
  * @brief  initialise Mpu6050
  * @param  None
  * @retval None
  */
u8 MPU_Init(void)
{ 
	u8 i = 0;
  u8 MPU6500_Init_Data[10][2] = 
		{
    {0x80, 	MPU_PWR_MGMT1_REG	},		// Reset Device
    {0x01, 	MPU_PWR_MGMT1_REG	},  	// Clock Source
		{0x01, 	MPU_USER_CTRL_REG},   	//reset the signal path
		{0x02,	MPU_SAMPLE_RATE_REG}, 	// 1khz/(1+SMPLRT_DIV)
    {0x00, 	MPU_PWR_MGMT2_REG	},  	// Enable Acc & Gyro
    {0x03, 	MPU_CFG_REG			},    	//0x06(5Hz) (0x04 20Hz) (0x03 42Hz) (0x05 10Hz) (0x02 98Hz)
		{0x02, 	MPU_INT_STA_REG	},			// Config INT Pin PP,active high, the INT pin emits a 50us long pulse 
		{0x01, 	MPU_INT_EN_REG	},			// Enable data ready interrupt 
    {0x18, 	MPU_GYRO_CFG_REG		},  // +-2000dps
    {0x08, 	MPU_ACCEL_CFG_REG   },  // +-4G
		};
		
  for(i=0; i<10; i++) 
	{
    MPU_Writebyte(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
    Delay_ms(10);
	}
	return SUCCESS;
}
/**
  * @brief  Mpu6050 get acceleromete and gyroscope
  * @param  Mpu6050_Value
  * @retval None
  */
u8 MPU_Getaccelerometergyroscope(Mpu6050_Type* Mpu6050_Value)
{
	u8 buf[14];  
  int16_t acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
	MPU_Readlen(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
	/*acc data*/
	acc_x = (buf[0]<<8) |buf[1];
	acc_y = (buf[2]<<8) |buf[3];
	acc_z = ((buf[4]<<8)|buf[5]);
	
	/*gyro data*/
	gyro_x=((buf[8]<<8) |buf[9]);
	gyro_y=((buf[10]<<8)|buf[11]);
	gyro_z=((buf[12]<<8)|buf[13]);
	
	Mpu6050_Value->temperature = (buf[6] << 8) + buf[7];
	
	Mpu6050_Value->acc_x = -acc_x;
	Mpu6050_Value->acc_y = -acc_y;
	Mpu6050_Value->acc_z = -acc_z;
		
	Mpu6050_Value->gyro_x = gyro_x; 
	Mpu6050_Value->gyro_y = gyro_y;
	Mpu6050_Value->gyro_z = gyro_z;

	return 1;
}
