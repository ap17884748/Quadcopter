#ifndef __FUSION_H
#define __FUSION_H

#ifdef __cplusplus
 extern "C" {
#endif
/*----------濾波----------*/
int accX_f,accY_f,accZ_f,gyroX_f,gyroY_f,gyroZ_f,magX_f,magY_f,magZ_f;
float Pressure_f,Temperature_f;

/*----------校正----------*/
int gyroX_offset,gyroY_offset,gyroZ_offset;
float magX_Max,magX_min,magY_Max,magY_min,magZ_Max,magZ_min;
/*----------融合----------*/
#define CNTLCYCLE 0.0200f				// 控制周期
#define PI 3.1415926f					// 圓周率
#define radius 57.295779f			//角度轉換
/*==參數調整==*/
#define g_Kp 0.080f //
#define g_Ki 0.00003f //
#define g_Kd 0.024f //

//float gyro_angleX,gyro_angleY,gyro_angleZ;
float g_Pitch;		// 範圍-180°~+180°	-->上正下負
float g_Roll;		// 範圍 -90°~+90°	-->右正左負
float g_Yaw;		// 範圍-180°~+180°	-->逆正順負

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

float g_q0,g_q1,g_q2,g_q3;// 四元數 PID 融合

float g_q0_1,g_q1_1,g_q2_1,g_q3_1;// 四元數 初始融合
float g_Pitch_1;		// 範圍-180°~+180°	-->上正下負
float g_Roll_1;		// 範圍 -90°~+90°	-->右正左負
float g_Yaw_1;		// 範圍-180°~+180°	-->逆正順負

#ifdef __cplusplus
}
#endif

#endif
