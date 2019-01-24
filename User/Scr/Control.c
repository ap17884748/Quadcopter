#include <stdio.h>
#include <stdint.h>
#include "mpu9250.h"
#include "fusion.h"
#include "DrvPWM.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "fusion.h"
#include "Control.h"
#include "math.h"
/*----------------------------------------

           motor_1(CCW)  motor_4(CW)
               --X--     --X--
          x       \       /
          |        \     /
          |         | = |
      y---Z         |   |
                    | = |
                   /     \
                  /       \
               --X--     --X--
           motor_2(CW)  motor_3(CCW)

-----------------------------------------*/
void MotorBase()
{
	if(InitMotor == 0)
	{
		if(PID_Base < 2800)	PID_Base += 10;
		else if(PID_Base < 3200)	PID_Base += 0.5;
		else if(PID_Base < PID_Base_S)	PID_Base += 20;
	}
	if((PID_Base >= PID_Base_S)&&(InitMotor == 0)) InitMotor = 1;
	if(InitMotor ==1)	PID_Base_S += PID_BaseR;
}

void Controller()
{
	PID_Yaw = 0;
	if(controllerdateright == 1)//forward
	{
		//motor_1;//-
		//motor_2;//+
		//motor_3;//+
		//motor_4;//-
		standard_Roll = 0;
		standard_Pitch = 10;
	}
	else if(controllerdateright == 2)//backward
	{
		//motor_1;//+
		//motor_2;//-
		//motor_3;//-
		//motor_4;//+
		standard_Roll = 0;
		standard_Pitch = -10;
	}
	else if(controllerdateright == 3)//left shift
	{
		//motor_1;//-
		//motor_2;//-
		//motor_3;//+
		//motor_4;//+
		standard_Roll = -10;
		standard_Pitch = 0;
	}
	else if(controllerdateright == 4)//right shift
	{
		//motor_1;//+
		//motor_2;//+
		//motor_3;//-
		//motor_4;//-
		standard_Roll = 10;
		standard_Pitch = 0;
	}
	else
	{
		standard_Roll = 0;
		standard_Pitch = 0;
	}
	
	if(controllerdateleft == 1&&CLStep == 0)//up
	{
		//motor_1;//+
		//motor_2;//+
		//motor_3;//+
		//motor_4;//+
		CLStep = 0;
		PID_Base = PID_Base_S + 100;
	}
	else if(controllerdateleft == 2&&CLStep == 0)//down
	{
		//motor_1;//-
		//motor_2;//-
		//motor_3;//-
		//motor_4;//-
		CLStep = 0;
		PID_Base = PID_Base_S - 100;
	}
	else if(controllerdateleft == 3)//turn left
	{
		//motor_1;//+
		//motor_2;//-
		//motor_3;//+
		//motor_4;//-
		PID_Yaw = -100;
	}
	else if(controllerdateleft == 4)//turn right
	{
		//motor_1;//-
		//motor_2;//+
		//motor_3;//-
		//motor_4;//+
		PID_Yaw = 100;
	}
	return 0;
}

void Quadrotor_Control()
{
	if(InitMotor == 1)
	{
		/*----Roll&Pitch----*/
		//防止翻落角度範圍
		if(fabs(g_Roll)>20)
			standard_Roll = 0;
		if(fabs(g_Pitch)>20)
			standard_Pitch = 0;

		//calculatePID
		//計算姿態角度當前誤差(當前姿態角度 - 期望姿態角度)
		error_Roll = g_Roll - standard_Roll;
		error_Pitch = g_Pitch - standard_Pitch;
		//error_Yaw = g_Yaw - standard_Yaw;

		//積分運算與限幅,積分分離-->在姿態誤差角小於20°時引入積分
		if (fabs(error_Roll) <= 20)	// Roll
		{
			//累加誤差
			errorI_Roll += error_Roll;
			//積分限幅
			if (errorI_Roll >= Roll_I_MAX)
				errorI_Roll = Roll_I_MAX;
			else if (errorI_Roll <= -Roll_I_MAX)
				errorI_Roll = -Roll_I_MAX;
		}
		if (fabs(error_Pitch) <= 20)// Pitch
		{
			//累加誤差
			errorI_Pitch += error_Pitch;
			//積分限幅
			if (errorI_Pitch >= Pitch_I_MAX)
				errorI_Pitch = Pitch_I_MAX;
			else if (errorI_Pitch <= -Pitch_I_MAX)
				errorI_Pitch = -Pitch_I_MAX;
		}

		//微分運算
		errorD_Roll = error_Roll - error_Roll0;
		errorD_Pitch = error_Pitch - error_Pitch0;
		error_Roll0 = error_Roll;
		error_Pitch0 = error_Pitch;

		/*----height----*/
		if(controllerdateleft == 0 && CLStep == 0)
		{
			standard_Height = MS5611_height_PT;
			CLStep = 1;//升降跳懸停
		}
		if((controllerdateleft == 1 || controllerdateleft == 2) && CLStep == 1)
		{
			PID_Base_S = PID_Base;
			CLStep = 0;//懸停跳升降
		}
		if(controllerdateleft == 0 && CLStep == 1)
		{
			//calculatePID
			//計算姿態高度當前誤差(當前姿態高度 - 期望姿態高度)
			error_Height = MS5611_height_PT - standard_Height;
			//積分運算與限幅,積分分離
			if (fabs(error_Height) <= 100)// Hright
			{
				//累加誤差
				errorI_Height += error_Height;
				//積分限幅
				if (errorI_Height >= Height_I_MAX)
					errorI_Height = Height_I_MAX;
				else if (errorI_Height <= -Height_I_MAX)
					errorI_Height = -Height_I_MAX;
			}
			//微分運算
			errorD_Height = error_Height - error_Height0;
			error_Height0 = error_Height;

			PID_Base = PID_Base_S /*- (PID_Kp * error_Height + PID_Ki * errorI_Height + PID_Kd * (float)errorD_Height)*/;
		}

		/*----PID&PWM----*/
		// P + D control (roll,pitch) and P control (yaw)
		PID_Roll  = PID_Kp_Roll * error_Roll  + PID_Ki_Roll * errorI_Roll  + PID_Kd_Roll * (float)errorD_Roll;
		PID_Pitch = PID_Kp_Pitch * error_Pitch + PID_Ki_Pitch * errorI_Pitch + PID_Kd_Pitch * (float)errorD_Pitch;
		//PID_Yaw = PID_Kp_Yaw * error_Yaw;

		//PWM參數設定
		motor1_PWM = PID_Base - PID_Roll + PID_Pitch - PID_Yaw;
		motor2_PWM = PID_Base - PID_Roll - PID_Pitch + PID_Yaw;
		motor3_PWM = PID_Base + PID_Roll - PID_Pitch - PID_Yaw;
		motor4_PWM = PID_Base + PID_Roll + PID_Pitch + PID_Yaw;
		//PWM反向清零
		if(motor1_PWM < 0)	motor1_PWM = 0;
		if(motor2_PWM < 0)	motor2_PWM = 0;
		if(motor3_PWM < 0)	motor3_PWM = 0;
		if(motor4_PWM < 0)	motor4_PWM = 0;
		//PWM限幅
		if(motor1_PWM > 9000)	motor1_PWM = 9000;
		if(motor2_PWM > 9000)	motor2_PWM = 9000;
		if(motor3_PWM > 9000)	motor3_PWM = 9000;
		if(motor4_PWM > 9000)	motor4_PWM = 9000;
	}
	//啟動ESC
	if(InitMotor == 0)
	{
		motor1_PWM = PID_Base;
		motor2_PWM = PID_Base;
		motor3_PWM = PID_Base;
		motor4_PWM = PID_Base;
	}

	//printf("%d,%d,%d,%d,%d,%d\n",(int)error_Roll,(int)errorI_Roll,(int)errorD_Roll,(int)error_Pitch,(int)errorI_Pitch,(int)errorD_Pitch);
	//printf("%d,%d\n",(int)PID_Base, (int)PID_Base_S);
	//printf("%d,%d,%d,%d\n",(int)PID_Roll,(int)PID_Pitch,(int)PID_Yaw,(int)PID_Base);
	//printf("%d,%d\n",(int)PID_Base,(int)InitMotor);
}

void Init_PWM()
{
	DrvPWM_Open();//使能PWM時鐘並且復位PWM
	DrvGPIO_InitFunction(E_FUNC_PWM01);//啟動腳位PWM功能GPA12,GPA13
	DrvGPIO_InitFunction(E_FUNC_PWM23);//啟動腳位PWM功能GPA14,GPA15
	DrvGPIO_Open(E_GPA, 12, E_IO_OUTPUT);//設置motor1輸出腳位
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT);//設置motor2輸出腳位
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT);//設置motor3輸出腳位
	DrvGPIO_Open(E_GPA, 15, E_IO_OUTPUT);//設置motor4輸出腳位

	//motor1
	DrvPWM_SelectClockSource(DRVPWM_TIMER0, DRVPWM_EXT_12M);//定時器DRVPWM_TIMER(0/1,2/3,4/5,6/7),時鐘外接12MHz或內部22MHz
	DrvPWM_SetTimerIO(DRVPWM_TIMER0,1);//使IO能輸出
	DrvPWM_Enable(DRVPWM_TIMER0,1);//使PWM0能工作
	//motor2
	DrvPWM_SelectClockSource(DRVPWM_TIMER1, DRVPWM_EXT_12M);
	DrvPWM_SetTimerIO(DRVPWM_TIMER1,1);
	DrvPWM_Enable(DRVPWM_TIMER1,1);
	//motor3
	DrvPWM_SelectClockSource(DRVPWM_TIMER2, DRVPWM_EXT_12M);
	DrvPWM_SetTimerIO(DRVPWM_TIMER2,1);
	DrvPWM_Enable(DRVPWM_TIMER2,1);
	//motor4
	DrvPWM_SelectClockSource(DRVPWM_TIMER3, DRVPWM_EXT_12M);
	DrvPWM_SetTimerIO(DRVPWM_TIMER3,1);
	DrvPWM_Enable(DRVPWM_TIMER3,1);
}

void motor_PWMcontrol()
{
	S_DRVPWM_TIME_DATA_T sPt_0;
	sPt_0.u8Mode= DRVPWM_AUTO_RELOAD_MODE;//自動裝載模式
	sPt_0.i32Inverter= 0;//反轉輸出開啟/關閉
	sPt_0.u32Frequency= 0;//PWM頻率(Hz)//u32Frequency=0時,Output frequency = HCLK freq / sPt.u8ClockSelector / sPt.u8PreScale / sPt.u32Duty
	sPt_0.u8ClockSelector= DRVPWM_CLOCK_DIV_1;//時鐘源選擇,輸入時鐘除以1,2,4,8,16//u32Frequency=0時生效
	sPt_0.u8PreScale= 2;//8位預分頻值(1~255)//u32Frequency=0時生效
	sPt_0.u32Duty= 10000;//脈衝佔空比(0x1~0x10000),捕捉定時用的計數值//u32Frequency=0,或工作在捕獲模式時有效//cnr
	sPt_0.u32HighPulse= motor1_PWM;//高脈衝0~u32Duty//cmr
	DrvPWM_SetTimerClk16bit(DRVPWM_TIMER0, &sPt_0);//初始化TIMER0時鐘
	//sPt_0.u8HighPulseRatio= (motor1_PWM/100);
	//DrvPWM_SetTimerClk(DRVPWM_TIMER0, &sPt_0);

	S_DRVPWM_TIME_DATA_T sPt_1;
	sPt_1.u8Mode= DRVPWM_AUTO_RELOAD_MODE;
	sPt_1.i32Inverter= 0;
	sPt_1.u32Frequency= 0;
	sPt_1.u8ClockSelector= DRVPWM_CLOCK_DIV_1;
	sPt_1.u8PreScale= 2;
	sPt_1.u32Duty= 10000;
	sPt_1.u32HighPulse= motor2_PWM;
	DrvPWM_SetTimerClk16bit(DRVPWM_TIMER1, &sPt_1);

	S_DRVPWM_TIME_DATA_T sPt_2;
	sPt_2.u8Mode= DRVPWM_AUTO_RELOAD_MODE;
	sPt_2.i32Inverter= 0;
	sPt_2.u32Frequency= 0;
	sPt_2.u8ClockSelector= DRVPWM_CLOCK_DIV_1;
	sPt_2.u8PreScale= 2;
	sPt_2.u32Duty= 10000;
	sPt_2.u32HighPulse= motor3_PWM;
	DrvPWM_SetTimerClk16bit(DRVPWM_TIMER2, &sPt_2);

	S_DRVPWM_TIME_DATA_T sPt_3;
	sPt_3.u8Mode= DRVPWM_AUTO_RELOAD_MODE;
	sPt_3.i32Inverter= 0;
	sPt_3.u32Frequency= 0;
	sPt_3.u8ClockSelector= DRVPWM_CLOCK_DIV_1;
	sPt_3.u8PreScale= 2;
	sPt_3.u32Duty= 10000;
	sPt_3.u32HighPulse= motor4_PWM;
	DrvPWM_SetTimerClk16bit(DRVPWM_TIMER3, &sPt_3);
}

/*void Quadrotor_Stop()// 停機
{
	motor1_PWM = 0;
	motor2_PWM = 0;
	motor3_PWM = 0;
	motor4_PWM = 0;
	if(motor1_PWM < 0)	motor1_PWM = 0;
	if(motor2_PWM < 0)	motor2_PWM = 0;
	if(motor3_PWM < 0)	motor3_PWM = 0;
	if(motor4_PWM < 0)	motor4_PWM = 0;
}*/
