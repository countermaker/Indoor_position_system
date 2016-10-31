#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f10x.h"
#include "delay.h"
#include "IIC.h"

typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;  //rad/s
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temperature;
} Mpu6050_Type;

//#define ACC_SENSITIVITY  16384.0f /* +- 2g */
#define ACC_SENSITIVITY  8192.0f /* +- 4g */
// #define GYRO_SENSITIVITY 32.8f    /* +- 1000 degree/s */
//#define GYRO_SENSITIVITY 65.5f    /* +- 500 degree/s */
#define GYRO_SENSITIVITY 16.384f    /* +- 2000 degree/s */

#define MPU_SELF_TESTX_REG		0X0D	
#define MPU_SELF_TESTY_REG		0X0E	
#define MPU_SELF_TESTZ_REG		0X0F	
#define MPU_SELF_TESTA_REG		0X10	
#define MPU_SAMPLE_RATE_REG		0X19	
#define MPU_CFG_REG				0X1A	
#define MPU_GYRO_CFG_REG		0X1B	
#define MPU_ACCEL_CFG_REG		0X1C	
#define MPU_ACCEL_CFG_REG2		0X1D	
#define MPU_MOTION_DET_REG		0X1F	
#define MPU_FIFO_EN_REG			0X23	
#define MPU_I2CMST_CTRL_REG		0X24	
#define MPU_I2CSLV0_ADDR_REG	0X25	
#define MPU_I2CSLV0_REG			0X26	
#define MPU_I2CSLV0_CTRL_REG	0X27	
#define MPU_I2CSLV1_ADDR_REG	0X28	
#define MPU_I2CSLV1_REG			0X29	
#define MPU_I2CSLV1_CTRL_REG	0X2A	
#define MPU_I2CSLV2_ADDR_REG	0X2B	
#define MPU_I2CSLV2_REG			0X2C	
#define MPU_I2CSLV2_CTRL_REG	0X2D	
#define MPU_I2CSLV3_ADDR_REG	0X2E	
#define MPU_I2CSLV3_REG			0X2F	
#define MPU_I2CSLV3_CTRL_REG	0X30	
#define MPU_I2CSLV4_ADDR_REG	0X31	
#define MPU_I2CSLV4_REG			0X32	
#define MPU_I2CSLV4_DO_REG		0X33	
#define MPU_I2CSLV4_CTRL_REG	0X34	
#define MPU_I2CSLV4_DI_REG		0X35	
#define MPU_I2CMST_STA_REG		0X36	
#define MPU_INTBP_CFG_REG		0X37	
#define MPU_INT_EN_REG			0X38	
#define MPU_INT_STA_REG			0X3A	
#define MPU_ACCEL_XOUTH_REG		0X3B	
#define MPU_ACCEL_XOUTL_REG		0X3C	
#define MPU_ACCEL_YOUTH_REG		0X3D	
#define MPU_ACCEL_YOUTL_REG		0X3E	
#define MPU_ACCEL_ZOUTH_REG		0X3F	
#define MPU_ACCEL_ZOUTL_REG		0X40	
#define MPU_TEMP_OUTH_REG		0X41	
#define MPU_TEMP_OUTL_REG		0X42	
#define MPU_GYRO_XOUTH_REG		0X43	
#define MPU_GYRO_XOUTL_REG		0X44	
#define MPU_GYRO_YOUTH_REG		0X45	
#define MPU_GYRO_YOUTL_REG		0X46	
#define MPU_GYRO_ZOUTH_REG		0X47	
#define MPU_GYRO_ZOUTL_REG		0X48	
#define MPU6500_EXT_SENS_DATA_00    ((u8)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((u8)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((u8)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((u8)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((u8)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((u8)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((u8)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((u8)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((u8)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((u8)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((u8)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((u8)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((u8)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((u8)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((u8)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((u8)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((u8)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((u8)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((u8)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((u8)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((u8)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((u8)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((u8)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((u8)0x60)
#define MPU_USER_CTRL         	0x6A
#define MPU_I2CSLV0_DO_REG		0X63	
#define MPU_I2CSLV1_DO_REG		0X64	
#define MPU_I2CSLV2_DO_REG		0X65	
#define MPU_I2CSLV3_DO_REG		0X66	
#define MPU_I2CMST_DELAY_REG	0X67	
#define MPU_SIGPATH_RST_REG		0X68	
#define MPU_MDETECT_CTRL_REG	0X69	
#define MPU_USER_CTRL_REG		0X6A	
#define MPU_PWR_MGMT1_REG		0X6B	
#define MPU_PWR_MGMT2_REG		0X6C	
#define MPU_FIFO_CNTH_REG		0X72	
#define MPU_FIFO_CNTL_REG		0X73	
#define MPU_FIFO_RW_REG			0X74	
#define MPU_DEVICE_ID_REG		0X75	

#define MPU_ADDR				    0X68

#define AK8963_Device_ID            0x48
#define MPU6500_Device_ID           0x71

u8 MPU_Init(void); 									

u8 MPU_Getaccelerometergyroscope(Mpu6050_Type* Mpu6050_Value);

#endif

