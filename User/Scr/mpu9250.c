#include "NUC1xx.h"
#include <stdint.h>
#include "mpu9250.h"
#include "DrvI2C.h"

void I2CWrite(uint8_t MPU_Adr,uint8_t INDEX,uint8_t DATA)
{
	SystemCoreClock = DrvSYS_GetHCLKFreq();
	//Open I2C0 and set clock = 50Kbps
	DrvI2C_Open(I2C_PORT0, 650000);

   	//send i2c start
	DrvI2C_Ctrl(I2C_PORT0, 1, 0, 1, 0);	//set start
	while (I2C0->I2CON.SI == 0);		//poll si flag

	//send to Write port
	I2C0->I2CDAT = MPU_Adr;				//send writer command
    DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0); //clr si flag
    while( I2C0->I2CON.SI == 0 );		//poll si flag

	//send write address
	I2C0->I2CDAT = INDEX;
	DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0); //clr si
	while( I2C0->I2CON.SI == 0 );		//poll si flag

	//send write data
	I2C0->I2CDAT = DATA;
	DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0); //clr si
	while( I2C0->I2CON.SI == 0 );		//poll si flag

   	//send i2c stop
	DrvI2C_Ctrl(I2C_PORT0, 0, 1, 1, 0); //send stop
	while( I2C0->I2CON.STO);
	//while( I2C0->CON.SI == 0 );

	DrvI2C_Close(I2C_PORT0);
}

uint8_t I2CRead(uint8_t MPU_Adr,uint8_t INDEX)
{
    uint8_t TEMP;
	//Open I2C0 and set clock = 50Kbps
	SystemCoreClock = DrvSYS_GetHCLKFreq();
	DrvI2C_Open(I2C_PORT0, 650000);
	//send i2c start
    DrvI2C_Ctrl(I2C_PORT0, 1, 0, 1, 0);	 	//set start
	while (I2C0->I2CON.SI == 0);			//poll si flag

   	//send to Write port
	I2C0->I2CDAT = MPU_Adr;
    DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);	   //clr si
    while( I2C0->I2CON.SI == 0 );		   //poll si flag

	//send INDEX
	I2C0->I2CDAT = INDEX;
    DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);	   //clr si
    while( I2C0->I2CON.SI == 0 );		   //poll si flag

	//send i2c start
    DrvI2C_Ctrl(I2C_PORT0, 1, 0, 1, 0);	 	//set start
	while (I2C0->I2CON.SI == 0);			//poll si flag

   	//send to Read port
	I2C0->I2CDAT = (MPU_Adr+1);
    DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);	   //clr si
    while( I2C0->I2CON.SI == 0 );		   //poll si flag

	//receive data
	I2C0->I2CDAT = 0xFF;
	DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);    //clr si
	while( I2C0->I2CON.SI == 0 );		   //poll si flag
	TEMP = I2C0->I2CDAT;

	//send i2c stop
 	DrvI2C_Ctrl(I2C_PORT0, 0, 1, 1, 0);    //clr si and set stop
	while( I2C0->I2CON.STO);
	DrvI2C_Close(I2C_PORT0);

	return TEMP;
}

void Init_MPU6050()
{
	//I2CWrite_MPU6050(PWR_MGMT_1, 0x80);
	I2CWrite(I2C_ADDR,PWR_MGMT_1, 0x00);// PWR_MGMT_1 MPU9250電源管理寄存器解除休眠
	DrvSYS_Delay(335000);
	i2c_dev = I2CRead(I2C_ADDR,EXT_SENS_DATA_00);//These Reg store data read from external sensors by the Slave 0
	if(i2c_dev==0x71)
	{
		DrvSYS_Delay(335000);
		I2CWrite(I2C_ADDR,PWR_MGMT_2, 0x00);//使能寄存加速度X,Y,Z加速度
		I2CWrite(I2C_ADDR,SMPLRT_DIV, 0x07);//SMPLRT_DIV 採樣率分頻寄存器,輸入採樣時鐘為SMPLRT_DIV kHz,1000/(SMPLRT_DIV+1)
		I2CWrite(I2C_ADDR,USER_CTRL, 0x00);// 初始化I2C
		I2CWrite(I2C_ADDR,CONFIG, 0x06);
		I2CWrite(I2C_ADDR,GYRO_CONFIG, 0x10);//0x00=250dps,0x08=500dps,0x10=1000dps,0x18=2000dps
		I2CWrite(I2C_ADDR,ACCEL_CONFIG, 0x08);//0x00=2g,0x08=4g,0x10=8g,0x18=16g
		I2CWrite(I2C_ADDR,INT_PIN_CFG,0x02);//進入Bypass模式，用於控制電子指南針
    	return 0;
	}
	return 1;
}

void Init_AK8963()
{
	//I2CWrite(AK8963_I2C_ADDR,AK8963_CNTL1,0x02);
	//I2CWrite(AK8963_I2C_ADDR,AK8963_CNTL1,0x00);// Power down magnetometer
	//DrvSYS_Delay(10000);
	//I2CWrite(AK8963_I2C_ADDR,AK8963_CNTL1,0x0F);// Enter Fuse ROM access mode
	//DrvSYS_Delay(10000);
	//I2CWrite(AK8963_I2C_ADDR,I2C_SLV0_ADDR,0x98);//MAG address 0x18，and Transfer is a read
	//I2CWrite(AK8963_I2C_ADDR,I2C_SLV0_CTRL,0x81);//Enable reading data from this slave at the sample rate && 1 Byte length

    //return 0;

	I2CWrite(I2C_ADDR,PWR_MGMT_1, 0x00); //
	I2CWrite(I2C_ADDR,SMPLRT_DIV, 0x00); //1khz
	//I2CWrite_MPU6050(I2C_ADDR,SMPLRT_DIV, 0x07);
	I2CWrite(I2C_ADDR,CONFIG, 0x02);
	I2CWrite(AK8963_I2C_ADDR,AK8963_CNTL1,0x02);// continute mode
	I2CWrite(I2C_ADDR,INT_PIN_CFG,0x02);

}

void MPU6050_value()
{
	tmpL = I2CRead(I2C_ADDR,ACCEL_XOUT_L); // read Accelerometer X_Low  value
    tmpH = I2CRead(I2C_ADDR,ACCEL_XOUT_H); // read Accelerometer X_High value
		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp;
		accX  = -(int)temp;
//		if(accX>20000)
//			accX=65536-accX;

    tmpL = I2CRead(I2C_ADDR,ACCEL_YOUT_L); // read Accelerometer Y_Low  value
    tmpH = I2CRead(I2C_ADDR,ACCEL_YOUT_H); // read Accelerometer Y_High value
 		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp;
		accY  = -(int)temp;
//		if(accY>20000)
//			accY=65536-accY;

    tmpL = I2CRead(I2C_ADDR,ACCEL_ZOUT_L); // read Accelerometer Z_Low  value
    tmpH = I2CRead(I2C_ADDR,ACCEL_ZOUT_H); // read Accelerometer Z_High value
 		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp;
		accZ  = -(int)temp;
//		if(accZ>20000)
//			accZ=65536-accZ;

		//gyroX0=gyroX;
    tmpL = I2CRead(I2C_ADDR,GYRO_XOUT_L); // read Gyroscope X_Low  value
    tmpH = I2CRead(I2C_ADDR,GYRO_XOUT_H); // read Gyroscope X_High value
		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp*8.06f;//*1000*0.033
		gyroX  = (int)temp;
		//if(gyroX>60000)
		//	gyroX=gyroX0;

		//gyroY0=gyroY;
    tmpL = I2CRead(I2C_ADDR,GYRO_YOUT_L); // read Gyroscope Y_Low  value
    tmpH = I2CRead(I2C_ADDR,GYRO_YOUT_H); // read Gyroscope Y_High value
 		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp*8.06f;//*1000*0.033
		gyroY  = (int)temp;
		//if(gyroY>60000)
		//	gyroY=gyroY0;

		//gyroZ0=gyroZ;
    tmpL = I2CRead(I2C_ADDR,GYRO_ZOUT_L); // read Gyroscope Z_Low  value
    tmpH = I2CRead(I2C_ADDR,GYRO_ZOUT_H); // read Gyroscope Z_High value
 		tmp = (tmpH<<8)+tmpL;
		temp = (float)tmp*8.06f;//*1000*0.033
		gyroZ  = (int)temp;
		//if(gyroZ>60000)
		//	gyroZ=gyroZ0;
}

void AK8963_value()
{
	/*此處放置大量乖乖 保佑磁力計*/
	I2CWrite(AK8963_I2C_ADDR,AK8963_CNTL1,0x02);
	if(I2CRead(AK8963_I2C_ADDR,AK8963_ST1)&0x01)
	{
		m[0] = I2CRead(AK8963_I2C_ADDR,AK8963_XOUT_L); // read Gyroscope X_Low  value
		m[1] = I2CRead(AK8963_I2C_ADDR,AK8963_XOUT_H); // read Gyroscope X_High value

		m[2] = I2CRead(AK8963_I2C_ADDR,AK8963_YOUT_L); // read Gyroscope Y_Low  value
		m[3] = I2CRead(AK8963_I2C_ADDR,AK8963_YOUT_H); // read Gyroscope Y_High value

		m[4] = I2CRead(AK8963_I2C_ADDR,AK8963_ZOUT_L); // read Gyroscope Z_Low  value
		m[5] = I2CRead(AK8963_I2C_ADDR,AK8963_ZOUT_H); // read Gyroscope Z_High value
	    m[6] = I2CRead(AK8963_I2C_ADDR,AK8963_ST2);

	    if(!(m[6]&0x08))
	    {
	    	magX=((m[3]<<8)+m[2]);
	    	if(magX>32768)
	    		magX=magX-65536;
	    	magX *= 1000;
	    	magX -= 72100;
	    	magY=((m[1]<<8)+m[0]);
	    	if(magY>32768)
	    		magY=magY-65536;
	    	magY *= 1000;
	    	magY += 21100;
	    	magZ=((m[5]<<8)+m[4]);
	    	if(magZ>32768)
	    		magZ=65536-magZ;
	    	magZ *= 1000;
	    	magZ -= 53000;
	    }
	}
	/*此處放置大量乖乖 保佑磁力計*/
}
