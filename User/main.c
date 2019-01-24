//
// Smpl_I2C_MPU6050
//
// MPU6050 : 3-axis Gyroscope + 3-axis accelerometer + 3-axis magnetometer + temperature
// Interface: I2C
// pin1: Vcc to Vcc (+5V)
// pin2: Gnd to Gnd
// pin3: SCL to I2C0_SCL/GPA9
// pin4: SDA to I2C0_SDA/GPA8
// pin5: XDA -- N.C.
// pin6: XCL -- N.C.
// pin7: AD0 -- N.C.
// pin8: INT -- N.C.

#include <stdio.h>
#include "DrvSYS.h"
#include "DrvUART.h"
#include "DrvI2C.h"
#include "DrvGPIO.h"
#include "math.h"
#include "DrvSPI.h"
#include "DrvPWM.h"
#include "DrvTIMER.h"
#include "mpu9250.h"
#include "MS5611.h"
#include "HMC5883L.h"
#include "fusion.h"
#include "Control.h"
#include "cc1101.h"

void Init_PWM();
void Init_MPU6050();
//void Init_MPU60501();
void Init_AK8963();

//uint8_t Init_HMC5883L();
//

//
//void MPU6050_value();
void MPU6050_value1();
//void AK8963_value();
void HMC5883L_value();
void accel_gyxo_mag_filter();
void MS5611_filter();
void gyro_offset();
void SPI_Init();
void Init(void);
void Strobe(unsigned char strobe);
//void mag_center_correction();
/*void mag_filter_all( float Ans[5], int magX, int magY, unsigned char Num )//橢圓擬合*/
void IMUupdata();
void Controller();
void Quadrotor_Control();
void motor_PWMcontrol();

volatile uint32_t u32Timer0Cnt = 20;
unsigned char rxlength, i;
unsigned char RxBuf[3]={0/*x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00*/};
short c=0,keystep=0,MS5611step,STOPstep=0;
int workstep;
int number,irow;
int32_t A3x3,B3x3,C3x3;

int PID_Base_1k,PID_Roll_1k,PID_Pitch_1k;
int PID_Kp_Roll_1k,PID_Ki_Roll_1k,PID_Kd_Roll_1k;
int PID_Kp_Pitch_1k,PID_Ki_Pitch_1k,PID_Kd_Pitch_1k;
//int Pressure_1k,Pressure_f_1k,Temperature_1k,Temperature_f_1k;
int g_Roll_1k,g_Pitch_1k,g_Yaw_1k;
int g_Roll_1_1k,g_Pitch_1_1k,g_Yaw_1_1k,IMUupdatastep=0;
int PID_Kp_1k,PID_Ki_1k,PID_Kd_1k;
int standard_Height_1k;

/*--------------------*/
void OpenKeyPad(void)
{
	uint8_t i;
	/* Initial key pad */
	for(i=0;i<8;i++)
	DrvGPIO_Open(E_GPA, i, E_IO_QUASI);
}

/*uint8_t Scankey(void) //0xfe=>1111 1110
{                     //0xfd=>1111 1101
                      //0xfb=>1111 1011
    				  //0xf7=>1111 0111
	uint8_t act[4]={0xfe, 0xfd, 0xfb, 0xf7};
	uint8_t i,temp,pin;

	for(i=0;i<4;i++)
	{
	   temp=act[i];
	   for(pin=0;pin<8;pin++)
	  {
	      if((temp&0x01)==0x01)
		DrvGPIO_SetBit(E_GPA,pin);
	      else
		DrvGPIO_ClrBit(E_GPA,pin);
	       temp>>=1;
	    }
    if(DrvGPIO_GetBit(E_GPA,4)==0)
		return(i+1);
    else if(DrvGPIO_GetBit(E_GPA,5)==0)
		return(i+5);
    else if(DrvGPIO_GetBit(E_GPA,6)==0)
		return(i+9);
    else if(DrvGPIO_GetBit(E_GPA,7)==0)
		return(i+13);
	}
	return 0;//未按送出0
}*/

void Controller2()
{
	//日本手		//4x4鍵盤
				//遙控器
	PID_BaseR = 0;
	if(number==1)
	{
		//停機鈕
		controllerdateleft = 4;
	}
	if(number==2)
	{
		//PID_Kd -= 0.0002;
		controllerdateleft = 1;
	}
	if(number==3)
	{
		//controllerdateright = 2;
		controllerdateleft = 2;
	}
	if(number==4)
	{
		//controllerdateright = 4;
		controllerdateleft = 3;
	}
	if(number==5)
	{
		//PID_Kp -= 0.002;
		//PID_Ki -= 0.0002;
		PID_Kp_Roll -= 0.002;
	}
	if(number==6)
	{
		//PID_Ki -= 0.0002;	//PID_BaseR = -1;
		PID_Kp_Pitch += 0.002;
	}
	if(number==7)
	{
		//controllerdateright = 3;
		//PID_Kd -= 0.0002;'
		PID_Kp_Pitch -= 0.002;
	}
	if(number==8)
	{
		//controllerdateright = 1;
		//PID_Kp -= 0.002;
		PID_Kp_Roll += 0.002;
	}
	if(number==9)
	{
		//controllerdateleft = 2;
		//停機鈕
	}
	if(number==10)
	{
		//controllerdateleft = 4;
		PID_BaseR = 1;//PID_Kp += 0.002;
	}
	if(number==11)
	{
		//PID_Ki += 0.0002;	//PID_BaseR = 1;
		PID_BaseR = -1;//PID_Ki += 0.0002;
	}
	if(number==12)
	{
		//PID_Kd += 0.0002;
		//PID_Kd += 0.0002;
	}
	if(number==13)
	{
		//controllerdateleft = 3;
		controllerdateright = 4;
	}
	if(number==14)
	{
		//controllerdateleft = 1;
		controllerdateright = 1;
	}
	if(number==15)
	{
		//PID_Kp += 0.002;
		controllerdateright = 2;
	}
	if(number==16)
	{
		controllerdateright = 3;
	}
}
/*--------------------*/

/*void Controller_test()
{
	if(RxBuf[3]==0x01){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=1,GPA_6=1,GPA_7=0;}
	if(RxBuf[3]==0x02){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=1,GPA_6=0,GPA_7=1;}
	if(RxBuf[3]==0x03){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=1,GPA_6=0,GPA_7=0;}
	if(RxBuf[3]==0x04){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=0,GPA_6=1,GPA_7=1;}
	if(RxBuf[3]==0x05){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=0,GPA_6=1,GPA_7=0;}
	if(RxBuf[3]==0x06){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=0,GPA_6=0,GPA_7=1;}
	if(RxBuf[3]==0x07){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=1,GPA_5=0,GPA_6=0,GPA_7=0;}
	if(RxBuf[3]==0x08){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=1,GPA_6=1,GPA_7=1;}
	if(RxBuf[3]==0x09){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=1,GPA_6=1,GPA_7=0;}
	if(RxBuf[3]==0x0a){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=1,GPA_6=0,GPA_7=1;}
	if(RxBuf[3]==0x0b){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=1,GPA_6=0,GPA_7=0;}
	if(RxBuf[3]==0x0c){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=0,GPA_6=1,GPA_7=1;}
	if(RxBuf[3]==0x0d){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=0,GPA_6=1,GPA_7=0;}
	if(RxBuf[3]==0x0e){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=0,GPA_6=0,GPA_7=1;}
	if(RxBuf[3]==0x0f){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=1,GPA_4=0,GPA_5=0,GPA_6=0,GPA_7=0;}
	if(RxBuf[3]==0x10){GPA_0=1,GPA_1=1,GPA_2=1,GPA_3=0,GPA_4=1,GPA_5=1,GPA_6=1,GPA_7=1;}
}*/

void CC1101_NUC140()
{
	int sugnal0x01,sugnal0x02,sugnal0x03,sugnal0x04,sugnal0x05;
	sugnal0x01=0;
	sugnal0x02=0;
	sugnal0x03=0;
	sugnal0x04=0;
	sugnal0x05=0;
	if(DrvGPIO_GetBit(E_GPA,4)==0)	{number += 16;sugnal0x05=1;}
	if(DrvGPIO_GetBit(E_GPA,3)==0)	{number += 8;sugnal0x04=1;}
	if(DrvGPIO_GetBit(E_GPA,2)==0)	{number += 4;sugnal0x03=1;}
	if(DrvGPIO_GetBit(E_GPA,1)==0)	{number += 2;sugnal0x02=1;}
	if(DrvGPIO_GetBit(E_GPA,0)==0)	{number += 1;sugnal0x01=1;}
	//printf("%d,%d,%d,%d,%d\n",sugnal0x05,sugnal0x04,sugnal0x03,sugnal0x02,sugnal0x01);
}

void TMR0_Callback()
{
	u32Timer0Cnt=1;
	c++;
}

int32_t main (void)
{
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, 1);
	DrvSYS_Delay(5000);					// Waiting for 12M Xtal stalble
	DrvSYS_SelectHCLKSource(0);
	LOCKREG();
	DrvSYS_SetClockDivider(E_SYS_HCLK_DIV, 0);

	Initial_pannel();
	clr_all_pannal();
   // print_lcd(0,"Smpl_I2C_MPU6050");

	DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC, 0x00);
	DrvSYS_SetIPClock(E_SYS_TMR0_CLK, 1);
	DrvSYS_LockProtectedReg();

	DrvTIMER_Init();
	DrvTIMER_Open(E_TMR0, 10000, E_PERIODIC_MODE);//計算頻率
	DrvTIMER_SetTimerEvent(E_TMR0, 200, TMR0_Callback, 1);//計數器
	DrvTIMER_EnableInt(E_TMR0);
	DrvTIMER_Start(E_TMR0);
	//DrvTIMER_Delay(E_TMR0,0);
	//DrvTIMER_Close(E_TMR0);

	STR_UART_T       UartParam;
	E_DRVGPIO_FUNC   FuncNum = E_FUNC_UART0;
	E_UART_PORT      UartNum = UART_PORT0;

	{
		DrvGPIO_InitFunction(FuncNum);
		UartParam.u32BaudRate        = 115200;
		UartParam.u8cDataBits        = DRVUART_DATABITS_8;
		UartParam.u8cStopBits        = DRVUART_STOPBITS_1;
		UartParam.u8cParity          = DRVUART_PARITY_NONE;
		UartParam.u8cRxTriggerLevel  = DRVUART_FIFO_1BYTES;
		UartParam.u8TimeOut          = 0;
		DrvUART_Open(UartNum, &UartParam);
		//printf(" \n Uart example, Please input: \n ");
		DrvGPIO_InitFunction(E_FUNC_I2C0);  // Initialize I2C
		DrvGPIO_InitFunction(E_FUNC_I2C1);  // Initialize I2C
		DrvGPIO_InitFunction(E_FUNC_SPI0);
	}

	DrvSYS_Delay(100000);
	Init_PWM();
	Init_MPU6050();                     // Initialize MPU6050
	Init_AK8963();
	MS5611_Init();
	//HMC5883L_Init();
	//Init_MPU60501();
	DrvSYS_Delay(100000);

	OpenKeyPad();

	/*DrvGPIO_Open(E_GPE,5,E_IO_INPUT);
	DrvGPIO_Open(E_GPE,6,E_IO_INPUT);
	SPI_Init();
	Init();
	Strobe(CCxxx0_SRX);*/

	g_q0 = 1.0f;g_q1 = 0.0f;g_q2 = 0.0f;g_q3 = 0.0f;// 四元數
	CLStep = 0;
	PID_Base = 2500;//4500開始轉
	PID_Base_S = 4800;//5300;//初始轉速//5250起飛
	InitMotor = 0;//啟動ESC
	MS5611_readPROM();
						//初始	//Roll	//Pitch
	PID_Kp_Roll = 7.500f;//2.000f//4.600f//5.789f	// PID控制比例係數
	PID_Ki_Roll = 0.000f;//0.020f//0.045f//0.060f	// PID控制積分係數
	PID_Kd_Roll = 0.000f;//0.152f//0.036f//0.152f	// PID控制微分係數

	PID_Kp_Pitch = 21.500f;//2.000f//4.600f//5.789f	// PID控制比例係數
	PID_Ki_Pitch = 0.000f;//0.020f//0.045f//0.060f	// PID控制積分係數
	PID_Kd_Pitch = 0.000f;//0.152f//0.036f//0.152f	// PID控制微分係數

	/*==調整區域==*/
	//float gyro_roll, gyro_pitch, gyro_yaw;
	//workstep=0;
	/*===========*/

	while(1)
	{
		if(u32Timer0Cnt==1)
		{
			/*迴圈初始值*/
			u32Timer0Cnt=0;
			controllerdateright=0;
			controllerdateleft=0;
			number=0;

			//感測
			MS5611step++;
			if(MS5611step==1)	MS5611_WritePressure();
			if(MS5611step==2)	MS5611_GetPressure();
			if(MS5611step==3)	MS5611_WriteTemperature();
			if(MS5611step==4){	MS5611_GetTemperature();	MS5611step = 0;}
			MPU6050_value();
				//MPU6050_value1();XXXX
			AK8963_value();
				//HMC5883L_value();XXXX

			//校正
			accel_gyxo_mag_filter();
			MS5611_filter();
			gyro_offset();
				//mag_center_correction();XXXX

			//姿態轉換
			IMUupdata();
			if(IMUupdatastep>=500)
			{
				IMUupdata_1();
			}
			else
			{
				g_q0_1 = g_q0;g_q1_1 = g_q1;g_q2_1 = g_q2;g_q3_1 = g_q3;// 四元數
				IMUupdatastep++;
			}
			Altimeter();

			//通訊
			CC1101_NUC140();
			/*keystep++;
			if(keystep==1)
			{
				ReceivePacket(RxBuf,&rxlength);
				number = RxBuf[3];
				keystep = 0;
				//printf("%d\n",(int)number);
			}*/
			//Controller_test();

			//控制
			Controller2();
			MotorBase();
			Controller();
			Quadrotor_Control();
			/*if(number==2)
			{
				motor1_PWM = 7000;
				motor2_PWM = 7000;
				motor3_PWM = 7000;
				motor4_PWM = 7000;
			}
			if(number==3)
			{
				motor1_PWM = 4000;
				motor2_PWM = 4000;
				motor3_PWM = 4000;
				motor4_PWM = 4000;
			}*/
			motor_PWMcontrol();

			/*if(workstep>=100)
			{
				gyro_roll = gyro_roll + gyroX_f*0.01;
				gyro_pitch = gyro_pitch + gyroY_f*0.01;
				gyro_yaw = gyro_yaw + gyroZ_f*0.01;
				printf("%d,%d,%d\n",(int)gyro_roll,(int)gyro_pitch,(int)gyro_yaw);
			}
			workstep++;*/
		/*感測數值+濾波+校正*/
		//printf("%d,%d,%d\n",accX,accY,accZ);
		//printf("%d,%d,%d\n",gyroX,gyroY,gyroZ);
		//printf("%d,%d,%d\n",magX,magY,magZ);
		//printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ);
		//printf("%d,%d\n",accX,accX_f);
		//printf("%d,%d\n",accY,accY_f);
		//printf("%d,%d\n",accZ,accZ_f);
		//printf("%d,%d,%d\n",accX_f,accY_f,accZ_f);
		//printf("%d,%d\n",gyroX,gyroX_f);
		//printf("%d,%d\n",gyroY,gyroY_f);
		//printf("%d,%d\n",gyroZ,gyroZ_f);
		//printf("%d,%d,%d\n",gyroX_f,gyroY_f,gyroZ_f);
		//printf("%d,%d,%d\n",gyroX_offset,gyroY_offset,gyroZ_offset)
		//printf("%d,%d\n",magX,magX_f);
		//printf("%d,%d\n",magY,magY_f);
		//printf("%d,%d\n",magZ,magZ_f);
		//printf("%d,%d,%d\n",(int)magX_f,(int)magY_f,(int)magZ_f);
		//printf("%d,%d,%d\n",(int)magX_5883,(int)magY_5883,(int)magZ_5883);
		//printf("%d,%d\n",(int)Pressure,(int)Temperature);
		//printf("%d,%d\n",(int)Pressure_1k,(int)Pressure_f_1k);
		//printf("%d,%d\n",(int)Temperature_1k,(int)Temperature_f_1k);
		//printf("%d,%d\n",(int)Pressure_f,(int)Temperature_f);

		//standard_Height_1k = standard_Height * 1000.0f;MS5611_height_PT *=1000.0f;//MS5611_height_P *=1000.0f;
		//printf("%d,%d\n",(int)MS5611_height_PT,(int)MS5611_height_P);

		//printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",accX_f,accY_f,accZ_f,gyroX_f,gyroY_f,gyroZ_f,magX_f,magY_f,magZ_f);
		/*角度轉換*/
		//printf("%d,%d,%d,%d\n",(int)g_q0,(int)g_q1,(int)g_q2,(int)g_q3);
		//printf("%d,%d,%d\n",(int)g_Roll,(int)g_Pitch,(int)g_Yaw);
		g_Roll_1k = g_Roll * 1000,g_Pitch_1k = g_Pitch * 1000,g_Yaw_1k = g_Yaw * 1000;
		g_Roll_1_1k = g_Roll_1 * 1000,g_Pitch_1_1k = g_Pitch_1 * 1000,g_Yaw_1_1k = g_Yaw_1 * 1000;
		//printf("%d,%d,%d\n",(int)g_Roll_1k,(int)g_Pitch_1k,(int)g_Yaw_1k);
		//printf("%d,%d,%d\n",(int)g_Roll_1_1k,(int)g_Pitch_1_1k,(int)g_Yaw_1_1k);
		//printf("%d,%d,%d,%d\n",(int)g_Roll_1k,(int)g_Pitch_1k,(int)g_Roll_1_1k,(int)g_Pitch_1_1k);
		//printf("%d,%d\n",(int)g_Roll_1k,(int)g_Roll_1_1k);
		//printf("%d,%d\n",(int)g_Pitch_1k,g_Pitch_1_1k);
		//printf("%d,%d\n",(int)g_Yaw_1k,(int)g_Yaw_1_1k);
		/*通訊測試*/

		/*電機輸出*/
		//printf("%d,%d,%d,%d\n",(int)motor1_PWM,(int)motor2_PWM,(int)motor3_PWM,(int)motor4_PWM);
		/*參數調試*/
		//printf("%d,%d,%d\n",Roll_1k,Pitch_1k,Yow_1k);

		//printf("%d,%d,%d,%d\n",(int)PID_Base,(int)error_Height,(int)standard_Height,(int)CLStep);

		PID_Base_1k = PID_Base * 1000;PID_Roll_1k = PID_Roll * 1000;PID_Pitch_1k = PID_Pitch * 1000;
		//printf("%d,%d,%d,%d\n",(int)PID_Base_1k,(int)PID_Roll_1k,(int)PID_Pitch_1k,(int)number);

		//PID_Kp_1k = PID_Kp * 1000;PID_Ki_1k = PID_Ki * 1000;PID_Kd_1k = PID_Kd * 1000;standard_Roll *= 1000;
		PID_Kp_Roll_1k = PID_Kp_Roll * 1000;PID_Ki_Roll_1k = PID_Ki_Roll * 1000;PID_Kd_Roll_1k = PID_Kd_Roll * 1000;
		PID_Kp_Pitch_1k = PID_Kp_Pitch * 1000;PID_Ki_Pitch_1k = PID_Ki_Pitch * 1000;PID_Kd_Pitch_1k = PID_Kd_Pitch * 1000;
		//printf("%d,%d,%d,%d,%d,%d\n",(int)MS5611_height_PT,(int)standard_Height_1k,(int)PID_Kp_1k,(int)PID_Ki_1k,(int)PID_Kd_1k,(int)number);

		//printf("%d,%d,%d,%d,%d\n",(int)MS5611_height_PT,(int)Roll_1k,(int)Pitch_1k,(int)PID_Base,(int)number);

		//printf("%d,%d,%d,%d,%d,%d\n",(int)Roll_1k,(int)Pitch_1k,(int)PID_Base,(int)errorI_Roll,(int)errorI_Pitch,(int)number);
		printf("%d,%d,%d,%d,%d,%d\n",(int)g_Roll_1k,(int)g_Pitch_1k,(int)PID_Base,(int)PID_Kp_Roll_1k,(int)PID_Kp_Pitch_1k,(int)number);
		/*執行時間*/
		//printf("%d\n",(int)c);
		//printf("%d,%d\n",(int)c,(int)number);
		c=0;

		//printf("%d,%d\n",(int)number,(int)HPratio0);

		/*sprintf(TEXT1,"x:%d X:%d",accX, gyroX);
		sprintf(TEXT2,"y:%d Y:%d",accY, gyroY);
		sprintf(TEXT3,"z:%d Z:%d",accZ, gyroZ);
	    print_lcd(1,TEXT1);
		print_lcd(2,TEXT2);
		print_lcd(3,TEXT3);*/
		}
	if(number==9)	break;
	if((fabs(g_Roll)>=30)||(fabs(g_Pitch)>=30))	break;
	}
	while(1)//停機設置
	{
		if(u32Timer0Cnt==1)
		{
			u32Timer0Cnt=0;
			//Quadrotor_Stop();

			motor1_PWM = 3500 - 100 * STOPstep;
			motor2_PWM = 3500 - 100 * STOPstep;
			motor3_PWM = 3500 - 100 * STOPstep;
			motor4_PWM = 3500 - 100 * STOPstep;
			if(motor1_PWM <= 0)	motor1_PWM = 0;
			if(motor2_PWM <= 0)	motor2_PWM = 0;
			if(motor3_PWM <= 0)	motor3_PWM = 0;
			if(motor4_PWM <= 0)	motor4_PWM = 0;

			motor_PWMcontrol();
			//printf("%d,%d,%d,%d\n",(int)motor1_PWM,(int)motor2_PWM,(int)motor3_PWM,(int)motor4_PWM);
			STOPstep++;
			if((motor1_PWM<=100)&&(motor2_PWM<=100)&&(motor3_PWM<=100)&&(motor4_PWM<=100))	break;
		}
	}
}
