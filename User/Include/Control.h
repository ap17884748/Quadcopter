#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
/*========================*/
#define Roll_I_MAX		5000
#define Pitch_I_MAX		5000
#define Height_I_MAX	2000
//#define motor_PWM_MAX	10000
//-----PID把计-----//
//#define PID_Kp_Roll 4.999f		// Roll PID北ゑㄒ玒计
//#define PID_Ki_Roll 0.050f		// Roll PID北縩だ玒计
//#define PID_Kd_Roll 0.060f		// Roll PID北稬だ玒计
//#define PID_Kp_Pitch 5.789f		// Roll PID北ゑㄒ玒计
//#define PID_Ki_Pitch 0.060f		// Roll PID北縩だ玒计
//#define PID_Kd_Pitch 0.152f		// Roll PID北稬だ玒计
//#define PID_Kp_Yaw 8.0f			// YAW虫縒P把计
#define PID_Kp_h 0.1f			// PID北ゑㄒ玒计
#define PID_Ki_h 0.0f			// PID北縩だ玒计
#define PID_Kd_h 0.0f			// PID北稬だ玒计

float PID_Kp,PID_Ki,PID_Kd;


uint8_t controllerdateright, controllerdateleft, CLStep, CRStep, InitMotor;

float standard_Roll, standard_Pitch, standard_Yaw, standard_Height;
float error_Roll, error_Pitch, error_Yaw, error_Height;
float errorI_Roll, errorI_Pitch, errorI_Height;
float errorD_Roll, errorD_Pitch, errorD_Height, error_Roll0, error_Pitch0, error_Height0;
float PID_Roll, PID_Pitch, PID_Yaw, PID_Base, PID_Base_S ,PID_BaseR;
float motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;
float PID_Kp_Roll,PID_Ki_Roll,PID_Kd_Roll,PID_Kp_Pitch,PID_Ki_Pitch,PID_Kd_Pitch;

uint8_t g_u8PWMCount;
uint16_t g_u16Frequency;
uint32_t s_u32Pulse;

#ifdef __cplusplus
}
#endif

#endif
