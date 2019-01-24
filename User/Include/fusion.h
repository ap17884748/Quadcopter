#ifndef __FUSION_H
#define __FUSION_H

#ifdef __cplusplus
 extern "C" {
#endif
/*----------�o�i----------*/
int accX_f,accY_f,accZ_f,gyroX_f,gyroY_f,gyroZ_f,magX_f,magY_f,magZ_f;
float Pressure_f,Temperature_f;

/*----------�ե�----------*/
int gyroX_offset,gyroY_offset,gyroZ_offset;
float magX_Max,magX_min,magY_Max,magY_min,magZ_Max,magZ_min;
/*----------�ĦX----------*/
#define CNTLCYCLE 0.0200f				// ����P��
#define PI 3.1415926f					// ��P�v
#define radius 57.295779f			//�����ഫ
/*==�Ѽƽվ�==*/
#define g_Kp 0.080f //
#define g_Ki 0.00003f //
#define g_Kd 0.024f //

//float gyro_angleX,gyro_angleY,gyro_angleZ;
float g_Pitch;		// �d��-180�X~+180�X	-->�W���U�t
float g_Roll;		// �d�� -90�X~+90�X	-->�k�����t
float g_Yaw;		// �d��-180�X~+180�X	-->�f�����t

float MS5611_height_PT,MS5611_height_P;

float Stm,Ctm,accR,magR;

float g_integralFBx;
float g_integralFBy;
float g_integralFBz;
float halfex0;
float halfey0;
float halfez0;
float g_differentialFBx;
float g_differentialFBy;
float g_differentialFBz;

float g_q0,g_q1,g_q2,g_q3;// �|���� PID �ĦX

float g_q0_1,g_q1_1,g_q2_1,g_q3_1;// �|���� ��l�ĦX
float g_Pitch_1;		// �d��-180�X~+180�X	-->�W���U�t
float g_Roll_1;		// �d�� -90�X~+90�X	-->�k�����t
float g_Yaw_1;		// �d��-180�X~+180�X	-->�f�����t

#ifdef __cplusplus
}
#endif

#endif
