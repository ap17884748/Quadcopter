#ifndef __HMC5883L_H
#define __HMC5883L_H

#ifdef __cplusplus
extern "C" {
#endif
//定義HMC588L的內部位址
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE     0x02
#define HMC5883L_DATA_X_H 0x03
#define HMC5883L_DATA_X_L 0x04
#define HMC5883L_DATA_Z_H 0x05
#define HMC5883L_DATA_Z_L 0x06
#define HMC5883L_DATA_Y_H 0x07
#define HMC5883L_DATA_Y_L 0x08
#define HMC5883L_STATUS   0x09
#define HMC5883L_IDENTIFICATION_A 0x0A
#define HMC5883L_IDENTIFICATION_B 0x0B
#define HMC5883L_IDENTIFICATION_C 0x0C
#define HMC5883L_ADDRESS 0x1E            //寫入：0x3C   讀出：0x3D
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C
//HMC5883L函數聲名
//uint8_t hmc5883l_init(void);		//初始化 (返回0成功)
//void Hmc5883_Read(void);	//讀原始資料
//void Get_Mag(void);         //將原始資料高地位元整合

//float Get_yaw(void);//獲得修正後的Yaw(yaw_hmc),並作為作為返回值返回

uint16_t HMC5883L_H,HMC5883L_L,HMC5883L_16;
int magX_5883,magY_5883,magZ_5883;

#ifdef __cplusplus
}
#endif

#endif
