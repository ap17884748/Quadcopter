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
		//����½�����׽d��
		if(fabs(g_Roll)>20)
			standard_Roll = 0;
		if(fabs(g_Pitch)>20)
			standard_Pitch = 0;

		//calculatePID
		//�p�⫺�A���׷�e�~�t(��e���A���� - ���櫺�A����)
		error_Roll = g_Roll - standard_Roll;
		error_Pitch = g_Pitch - standard_Pitch;
		//error_Yaw = g_Yaw - standard_Yaw;

		//�n���B��P���T,�n������-->�b���A�~�t���p��20�X�ɤޤJ�n��
		if (fabs(error_Roll) <= 20)	// Roll
		{
			//�֥[�~�t
			errorI_Roll += error_Roll;
			//�n�����T
			if (errorI_Roll >= Roll_I_MAX)
				errorI_Roll = Roll_I_MAX;
			else if (errorI_Roll <= -Roll_I_MAX)
				errorI_Roll = -Roll_I_MAX;
		}
		if (fabs(error_Pitch) <= 20)// Pitch
		{
			//�֥[�~�t
			errorI_Pitch += error_Pitch;
			//�n�����T
			if (errorI_Pitch >= Pitch_I_MAX)
				errorI_Pitch = Pitch_I_MAX;
			else if (errorI_Pitch <= -Pitch_I_MAX)
				errorI_Pitch = -Pitch_I_MAX;
		}

		//�L���B��
		errorD_Roll = error_Roll - error_Roll0;
		errorD_Pitch = error_Pitch - error_Pitch0;
		error_Roll0 = error_Roll;
		error_Pitch0 = error_Pitch;

		/*----height----*/
		if(controllerdateleft == 0 && CLStep == 0)
		{
			standard_Height = MS5611_height_PT;
			CLStep = 1;//�ɭ����a��
		}
		if((controllerdateleft == 1 || controllerdateleft == 2) && CLStep == 1)
		{
			PID_Base_S = PID_Base;
			CLStep = 0;//�a�����ɭ�
		}
		if(controllerdateleft == 0 && CLStep == 1)
		{
			//calculatePID
			//�p�⫺�A���׷�e�~�t(��e���A���� - ���櫺�A����)
			error_Height = MS5611_height_PT - standard_Height;
			//�n���B��P���T,�n������
			if (fabs(error_Height) <= 100)// Hright
			{
				//�֥[�~�t
				errorI_Height += error_Height;
				//�n�����T
				if (errorI_Height >= Height_I_MAX)
					errorI_Height = Height_I_MAX;
				else if (errorI_Height <= -Height_I_MAX)
					errorI_Height = -Height_I_MAX;
			}
			//�L���B��
			errorD_Height = error_Height - error_Height0;
			error_Height0 = error_Height;

			PID_Base = PID_Base_S /*- (PID_Kp * error_Height + PID_Ki * errorI_Height + PID_Kd * (float)errorD_Height)*/;
		}

		/*----PID&PWM----*/
		// P + D control (roll,pitch) and P control (yaw)
		PID_Roll  = PID_Kp_Roll * error_Roll  + PID_Ki_Roll * errorI_Roll  + PID_Kd_Roll * (float)errorD_Roll;
		PID_Pitch = PID_Kp_Pitch * error_Pitch + PID_Ki_Pitch * errorI_Pitch + PID_Kd_Pitch * (float)errorD_Pitch;
		//PID_Yaw = PID_Kp_Yaw * error_Yaw;

		//PWM�ѼƳ]�w
		motor1_PWM = PID_Base - PID_Roll + PID_Pitch - PID_Yaw;
		motor2_PWM = PID_Base - PID_Roll - PID_Pitch + PID_Yaw;
		motor3_PWM = PID_Base + PID_Roll - PID_Pitch - PID_Yaw;
		motor4_PWM = PID_Base + PID_Roll + PID_Pitch + PID_Yaw;
		//PWM�ϦV�M�s
		if(motor1_PWM < 0)	motor1_PWM = 0;
		if(motor2_PWM < 0)	motor2_PWM = 0;
		if(motor3_PWM < 0)	motor3_PWM = 0;
		if(motor4_PWM < 0)	motor4_PWM = 0;
		//PWM���T
		if(motor1_PWM > 9000)	motor1_PWM = 9000;
		if(motor2_PWM > 9000)	motor2_PWM = 9000;
		if(motor3_PWM > 9000)	motor3_PWM = 9000;
		if(motor4_PWM > 9000)	motor4_PWM = 9000;
	}
	//�Ұ�ESC
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
	DrvPWM_Open();//�ϯ�PWM�����åB�_��PWM
	DrvGPIO_InitFunction(E_FUNC_PWM01);//�Ұʸ}��PWM�\��GPA12,GPA13
	DrvGPIO_InitFunction(E_FUNC_PWM23);//�Ұʸ}��PWM�\��GPA14,GPA15
	DrvGPIO_Open(E_GPA, 12, E_IO_OUTPUT);//�]�mmotor1��X�}��
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT);//�]�mmotor2��X�}��
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT);//�]�mmotor3��X�}��
	DrvGPIO_Open(E_GPA, 15, E_IO_OUTPUT);//�]�mmotor4��X�}��

	//motor1
	DrvPWM_SelectClockSource(DRVPWM_TIMER0, DRVPWM_EXT_12M);//�w�ɾ�DRVPWM_TIMER(0/1,2/3,4/5,6/7),�����~��12MHz�Τ���22MHz
	DrvPWM_SetTimerIO(DRVPWM_TIMER0,1);//��IO���X
	DrvPWM_Enable(DRVPWM_TIMER0,1);//��PWM0��u�@
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
	sPt_0.u8Mode= DRVPWM_AUTO_RELOAD_MODE;//�۰ʸ˸��Ҧ�
	sPt_0.i32Inverter= 0;//�����X�}��/����
	sPt_0.u32Frequency= 0;//PWM�W�v(Hz)//u32Frequency=0��,Output frequency = HCLK freq / sPt.u8ClockSelector / sPt.u8PreScale / sPt.u32Duty
	sPt_0.u8ClockSelector= DRVPWM_CLOCK_DIV_1;//���������,��J�������H1,2,4,8,16//u32Frequency=0�ɥͮ�
	sPt_0.u8PreScale= 2;//8��w���W��(1~255)//u32Frequency=0�ɥͮ�
	sPt_0.u32Duty= 10000;//�߽Ħ��Ť�(0x1~0x10000),�����w�ɥΪ��p�ƭ�//u32Frequency=0,�Τu�@�b����Ҧ��ɦ���//cnr
	sPt_0.u32HighPulse= motor1_PWM;//���߽�0~u32Duty//cmr
	DrvPWM_SetTimerClk16bit(DRVPWM_TIMER0, &sPt_0);//��l��TIMER0����
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

/*void Quadrotor_Stop()// ����
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
