#include "NUC1xx.h"
#include <stdint.h>
#include "mpu9250.h"
#include "DrvI2C.h"
#include "MS5611.h"
#include "HMC5883L.h"
#include "DrvGPIO.h"

/*函式宣告----------------------------------------------------------------*/
 void I2CWriteNoAddr(uint8_t MPU_Adr,uint8_t DATA);
 uint16_t I2C_Read_2Bytes(uint8_t MPU_Adr,uint8_t INDEX);
 uint32_t I2C_Read_3Bytes(uint8_t MPU_Adr,uint8_t INDEX);
 void MS5611_Reset(void);
 void MS5611_readPROM(void);
 uint32_t MS5611_DO_CONVERSION(uint8_t command);
 void MS5611_WriteTemperature();
 void MS5611_GetTemperature();
 void MS5611_WritePressure();
 void MS5611_GetPressure();
 void MS5611_Init(void);
 void SampleANDExchange(void);
 /************************************************************/
 /*void I2CWrite01(uint8_t MPU_Adr,uint8_t INDEX,uint8_t DATA)
{
	SystemCoreClock = DrvSYS_GetHCLKFreq();
  //Open I2C1 and set clock = 50Kbps
	DrvI2C_Open(I2C_PORT1, 650000);

   	//send i2c start
	DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	//set start
	while (I2C1->I2CON.SI == 0);		//poll si flag

	//send to Write port
	I2C1->I2CDAT = MPU_Adr;				//send writer command
    DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si flag
    while( I2C1->I2CON.SI == 0 );		//poll si flag

	//send write address
	I2C1->I2CDAT = INDEX;
	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si
	while( I2C1->I2CON.SI == 0 );		//poll si flag

	//send write data
	I2C1->I2CDAT = DATA;
	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si
	while( I2C1->I2CON.SI == 0 );		//poll si flag

   	//send i2c stop
	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0); //send stop
	while( I2C1->I2CON.STO);
//	while( I2C1->CON.SI == 0 );

	DrvI2C_Close(I2C_PORT1);
}

uint8_t I2CRead01(uint8_t MPU_Adr,uint8_t INDEX)
{
    uint8_t TEMP;
  //Open I2C1 and set clock = 50Kbps
	SystemCoreClock = DrvSYS_GetHCLKFreq();
	DrvI2C_Open(I2C_PORT1, 650000);
	//send i2c start
    DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
	while (I2C1->I2CON.SI == 0);			//poll si flag

   	//send to Write port
	I2C1->I2CDAT = MPU_Adr;
    DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);	   //clr si
    while( I2C1->I2CON.SI == 0 );		   //poll si flag

	//send INDEX
	I2C1->I2CDAT = INDEX;
    DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);	   //clr si
    while( I2C1->I2CON.SI == 0 );		   //poll si flag

	//send i2c start
    DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
	while (I2C1->I2CON.SI == 0);			//poll si flag

   	//send to Read port
	I2C1->I2CDAT = (MPU_Adr+1);
    DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);	   //clr si
    while( I2C1->I2CON.SI == 0 );		   //poll si flag

	//receive data
	I2C1->I2CDAT = 0xFF;
	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);    //clr si
	while( I2C1->I2CON.SI == 0 );		   //poll si flag
	TEMP = I2C1->I2CDAT;

	//send i2c stop
 	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0);    //clr si and set stop
	while( I2C1->I2CON.STO);
	DrvI2C_Close(I2C_PORT1);

	return TEMP;
}
 }*/

 void I2CWriteNoAddr(uint8_t MPU_Adr,uint8_t DATA)
 {
 	SystemCoreClock = DrvSYS_GetHCLKFreq();
   //Open I2C1 and set clock = 50Kbps
 	DrvI2C_Open(I2C_PORT1, 650000);

    	//send i2c start
 	DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	//set start
 	while (I2C1->I2CON.SI == 0);		//poll si flag

 	//send to Write port
 	I2C1->I2CDAT = MPU_Adr;				//send writer command
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si flag
     while( I2C1->I2CON.SI == 0 );		//poll si flag
/*
 	//send write address
 	I2C1->I2CDAT = INDEX;
 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si
 	while( I2C1->I2CON.SI == 0 );		//poll si flag
*/
 	//send write data
 	I2C1->I2CDAT = DATA;
 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); //clr si
 	while( I2C1->I2CON.SI == 0 );		//poll si flag

    	//send i2c stop
 	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0); //send stop
 	while( I2C1->I2CON.STO);
 //	while( I2C1->CON.SI == 0 );

 	DrvI2C_Close(I2C_PORT1);
 }

uint16_t I2C_Read_2Bytes(uint8_t MPU_Adr,uint8_t INDEX)
 {
	unsigned char data_temp1,data_temp2;
	uint16_t TEMP;

   //Open I2C1 and set clock = 50Kbps
 	SystemCoreClock = DrvSYS_GetHCLKFreq();
 	DrvI2C_Open(I2C_PORT1, 650000);
 	//send i2c start
     DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
 	while (I2C1->I2CON.SI == 0);			//poll si flag

    	//send to Write port
 	I2C1->I2CDAT = MPU_Adr;
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//send INDEX
 	I2C1->I2CDAT = INDEX;
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//send i2c start
     DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
 	while (I2C1->I2CON.SI == 0);			//poll si flag

    	//send to Read port
 	I2C1->I2CDAT = (MPU_Adr+1);
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//receive data
 	I2C1->I2CDAT = 0xFF;
 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);    //clr si
 	while( I2C1->I2CON.SI == 0 );		   //poll si flag
 	data_temp1 = I2C1->I2CDAT;

 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);    //clr si
 	while( I2C1->I2CON.SI == 0 );		   //poll si flag
 	data_temp2 = I2C1->I2CDAT;

 	//send i2c stop
  	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0);    //clr si and set stop
 	while( I2C1->I2CON.STO);
 	DrvI2C_Close(I2C_PORT1);

 	TEMP=(data_temp1<<8)|data_temp2;

 	return TEMP;
 }

uint32_t I2C_Read_3Bytes(uint8_t MPU_Adr,uint8_t INDEX)
 {
	uint8_t data_temp1,data_temp2,data_temp3;
     uint32_t TEMP;
   //Open I2C1 and set clock = 50Kbps
 	SystemCoreClock = DrvSYS_GetHCLKFreq();
 	DrvI2C_Open(I2C_PORT1, 650000);
 	//send i2c start
     DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
 	while (I2C1->I2CON.SI == 0);			//poll si flag

    	//send to Write port
 	I2C1->I2CDAT = MPU_Adr;
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//send INDEX
 	I2C1->I2CDAT = INDEX;
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//send i2c start
     DrvI2C_Ctrl(I2C_PORT1, 1, 0, 1, 0);	 	//set start
 	while (I2C1->I2CON.SI == 0);			//poll si flag

    	//send to Read port
 	I2C1->I2CDAT = (MPU_Adr+1);
     DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);	   //clr si(0010)
     while( I2C1->I2CON.SI == 0 );		   //poll si flag

 	//receive data
 	I2C1->I2CDAT = 0xFF;
 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);    //clr si
 	while( I2C1->I2CON.SI == 0 );		   //poll si flag
 	data_temp1 = I2C1->I2CDAT;

 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1);    //clr si
 	while( I2C1->I2CON.SI == 0 );		   //poll si flag
 	data_temp2 = I2C1->I2CDAT;

 	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0);    //clr si
 	while( I2C1->I2CON.SI == 0 );		   //poll si flag
 	data_temp3 = I2C1->I2CDAT;

 	//send i2c stop
  	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0);    //clr si and set stop
 	while( I2C1->I2CON.STO);
 	DrvI2C_Close(I2C_PORT1);

 	TEMP = (data_temp1<<16)|(data_temp2<<8)|data_temp3;

 	return TEMP;
 }

void MS5611_Reset(void)
{
      //  I2C_NoAddr_WriteByte(MS5611_ADDR,MS561101BA_RESET);
	I2CWriteNoAddr(MS5611_ADDR,0x1e);
}

void MS5611_readPROM(void)
{
	uint16_t value=0;uint8_t temp1[2]={0};
	uint8_t i/*,sig[2]*/;
      for (i=0;i<=MS561101BA_PROM_REG_COUNT;i++)
     {
       /*I2C_Read_MultiBytes(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2,temp1);
       I2C_Read_XBytes(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2,temp1);

       value=temp1[0]<<8|temp1[1];
       Cal_C[i]=value;*/
       Cal_C[i]=I2C_Read_2Bytes(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
       //Cal_C[i]=I2CRead(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    /*sig[0] = I2CRead(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    sig[1] = I2CRead(MS5611_ADDR,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE+1));
    Cal_C[i]= (sig[0]<<8) + sig[1];*/
     }
    DrvSYS_Delay(10000);
    //printf("\n The MS561101BA is reading PROM : \r\n");
    //printf("\r\nC1 = %d\r\nC2 = %d\r\nC3 = %d\r\nC4 = %d\r\nC5 = %d\r\nC6 = %d\r\n",Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);
}

/*uint32_t MS5611_DO_CONVERSION(uint8_t command)
{
        uint32_t conversion;
        //uint8_t sig[3];

    // I2C_NoAddr_WriteByte(MS5611_ADDR,command);
        I2CWriteNoAddr(MS5611_ADDR,command);

     //delay_ms(10);//延時,去掉資料錯誤
     DrvSYS_Delay(8000);
    conversion = I2C_Read_3Bytes(MS5611_ADDR,0x00);
    //sig[0] = I2CRead(MS5611_ADDR,0x00);
     //sig[1] = I2CRead(MS5611_ADDR,0x01);
     //sig[2] = I2CRead(MS5611_ADDR,0x02);
     //conversion = sig[0]<<16+sig[1]<<8+sig[2];

   return conversion;

}*/
void MS5611_WriteTemperature()
{
    I2CWriteNoAddr(MS5611_ADDR,0x58);
    I2CWriteNoAddr(MS5611_ADDR,MS561101BA_OSR_4096);
}

void MS5611_GetTemperature()
{
    D2_Temp = I2C_Read_3Bytes(MS5611_ADDR,0x00);
    //delay_ms(100);
    //DrvSYS_Delay(10000);
    dT = D2_Temp - (((uint32_t)Cal_C[5])<<8);
    Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;   //算出溫度值的100倍，2001表示20.01°


}

void MS5611_WritePressure()
{
    I2CWriteNoAddr(MS5611_ADDR,0x48);
    I2CWriteNoAddr(MS5611_ADDR,MS561101BA_OSR_4096);
}

void MS5611_GetPressure()
{
    D1_Pres= I2C_Read_3Bytes(MS5611_ADDR,0x00);

    //delay_ms(100);
    //DrvSYS_Delay(10000);
    OFF = (uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT)/128.0;
    SENS = (uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0;
    //溫度補償
    if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
    {
        Temperature2 = (dT*dT) / 0x80000000;
        Aux = (Temperature-2000)*(Temperature-2000);
        OFF2 = 2.5*Aux;
        SENS2 = 1.25*Aux;
        if(Temperature < -1500)
        {
            Aux = (Temperature+1500)*(Temperature+1500);
            OFF2 = OFF2 + 7*Aux;
            SENS2 = SENS + 5.5*Aux;
        }
    }else  //(Temperature > 2000)
    {
        Temperature2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    Temperature = Temperature - Temperature2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    Pressure=(D1_Pres*SENS/2097152.0f-OFF)/32768.0f;

}

void MS5611_Init(void)
{
    MS5611_Reset();

    //delay_ms(100);
    DrvSYS_Delay(100000);
    MS5611_readPROM();
    //delay_ms(100);
    DrvSYS_Delay(100000);
}

/*void SampleANDExchange(void)
{
   uint8_t i=0;
    MS5611_GetTemperature(MS561101BA_OSR_4096);//0x58
    MS5611_GetPressure(MS561101BA_OSR_4096);     //0x48
    ex_Pressure=(long)(Pressure);

    //printf("%d\n",(int)());
    if(Pressure<0)
    {
        ex_Pressure=-ex_Pressure;
        exchange_num[0]='-';
    }
    else exchange_num[0]='\0';

    exchange_num[1]=ex_Pressure/100000+0x30;
    ex_Pressure=ex_Pressure%100000;

    exchange_num[2]=ex_Pressure/10000+0x30;
    ex_Pressure=ex_Pressure%10000;

    exchange_num[3]=ex_Pressure/1000+0x30;
    ex_Pressure=ex_Pressure%1000;

    exchange_num[4]=ex_Pressure/100+0x30;
    ex_Pressure=ex_Pressure%100;

    exchange_num[5]='.';

    exchange_num[6]=ex_Pressure/10+0x30;
    ex_Pressure=ex_Pressure%10;

    exchange_num[7]=ex_Pressure+0x30;

     //printf("\nP : %c%c%c%c%c%c%c%c\r\n",exchange_num[0],exchange_num[1],exchange_num[2],exchange_num[3],exchange_num[4],exchange_num[5],exchange_num[6],exchange_num[7]);
    // printf("\nP : %lu mbar\r\n",ex_Pressure);
    //   for(i=0;i<8;i++)
 // {
 //   printf("%c",exchange_num[i]);
 // }
  //  printf(" mbar   \r\n");
    //printf("T : %4.3f\r\n ",Temperature/100);

    printf("P : %d\n",(int)Pressure);
    printf("T : %d\n",(int)Temperature);

}*/
/************************************************************/
/*void HMC5883L_Init()
{
	I2CWrite(HMC5883L_ADDRESS,HMC5883L_CONFIG_A,0x14);
	I2CWrite(HMC5883L_ADDRESS,HMC5883L_CONFIG_B,0x20);
	I2CWrite(HMC5883L_ADDRESS,HMC5883L_MODE,0x00);
}

void HMC5883L_value()
{

	I2CRead(HMC5883L_ADDRESS,HMC5883L_CONFIG_A);

	HMC5883L_H = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_X_H);
	HMC5883L_L = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_X_L);
	HMC5883L_16 = (HMC5883L_H<<8)+HMC5883L_L;
	magX_5883 = (int)(HMC5883L_16);

	HMC5883L_H = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_Z_H);
	HMC5883L_L = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_Z_L);
	HMC5883L_16 = (HMC5883L_H<<8)+HMC5883L_L;
	magZ_5883 = (int)(HMC5883L_16);

	HMC5883L_H = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_Y_H);
	HMC5883L_L = I2CRead(HMC5883L_ADDRESS,HMC5883L_DATA_Y_L);
	HMC5883L_16 = (HMC5883L_H<<8)+HMC5883L_L;
	magY_5883 = (int)(HMC5883L_16);


}*/
