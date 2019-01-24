#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
 extern "C" {
#endif
 /*========================*/
#define AK8963_I2C_ADDR 0x0C<<1
#define AK8963_DeviceID	0x48

#define	AK8963_WHO_AM_I	0x00
#define	AK8963_INFO		0x01
#define	AK8963_ST1		0x02
#define	AK8963_XOUT_L	0x03
#define	AK8963_XOUT_H	0x04
#define	AK8963_YOUT_L	0x05
#define	AK8963_YOUT_H	0x06
#define	AK8963_ZOUT_L	0x07
#define	AK8963_ZOUT_H	0x08
#define	AK8963_ST2		0x09
#define	AK8963_CNTL1	0x0A
#define	AK8963_CNTL2	0x0B
#define	AK8963_ASTC		0x0C
#define	AK8963_TS1		0x0D
#define	AK8963_TS2		0x0E
#define	AK8963_I2CDIS	0x0F
#define	AK8963_ASAX		0x10
#define	AK8963_ASAY		0x11
#define	AK8963_ASAZ		0x12

#define	AK8963_STATUS_DRDY	0x01
#define	AK8963_STATUS_DOR	0x02
#define	AK8963_STATUS_HOFL	0x08
/*========================*/
#define SELF_TEST_XG	0x00
#define SELF_TEST_YG	0x01
#define SELF_TEST_ZG	0x02
#define SELF_TEST_XA	0x0D
#define SELF_TEST_YA	0x0E
#define SELF_TEST_ZA	0x0F
#define XG_OFFSET_H		0x13
#define XG_OFFSET_L		0x14
#define YG_OFFSET_H		0x15
#define YG_OFFSET_L		0x16
#define ZG_OFFSET_H		0x17
#define ZG_OFFSET_L		0x18
#define SMPLRT_DIV		0x19	//(125Hz)
#define	CONFIG			0x1A	//(5Hz)
#define	GYRO_CONFIG		0x1B	//0x182000deg/s)
#define	ACCEL_CONFIG	0x1C	//(5Hz)
#define	ACCEL_CONFIG_2	0x1D
#define LP_ACCEL_ODR	0x1E
#define MOT_THR			0x1F
#define FIFO_EN			0x23
#define I2C_MST_CTRL	0x24
#define I2C_SLV0_ADDR	0x25
#define I2C_SLV0_REG	0x26
#define I2C_SLV0_CTRL	0x27
#define I2C_SLV1_ADDR	0x28
#define I2C_SLV1_REG	0x29
#define I2C_SLV1_CTRL	0x2A
#define I2C_SLV2_ADDR	0x2B
#define I2C_SLV2_REG	0x2C
#define I2C_SLV2_CTRL	0x2D
#define I2C_SLV3_ADDR	0x2E
#define I2C_SLV3_REG	0x2F
#define I2C_SLV3_CTRL	0x30
#define I2C_SLV4_ADDR	0x31
#define I2C_SLV4_REG	0x32
#define I2C_SLV4_DO		0x33
#define I2C_SLV4_CTRL	0x34
#define I2C_SLV4_DI		0x35
#define I2C_MST_STATUS	0x36
#define INT_PIN_CFG     0x37	//INT引腳配置和Bypass配置寄存器
#define INT_ENABLE		0x38
#define INT_STATUS		0x3A
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL		0x6A
#define	PWR_MGMT_1		0x6B	//?qˇ????z?A‥???-??G0x00(???`???I)
#define	PWR_MGMT_2		0x6C
#define DMP_BANK        0x6D	// Activates a specific bank in the DMP
#define DMP_RW_PNT      0x6E	// Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG         0x6F	// Register in DMP from which to read or to which to write
#define DMP_REG_1       0x70
#define DMP_REG_2       0x71
#define FIFO_COUNTH		0x72
#define FIFO_COUNTL		0x73
#define FIFO_R_W		0x74
#define	WHO_AM_I		0x75   /* ID = 0x71 In MPU9250 */
#define XA_OFFSET_H		0x77
#define XA_OFFSET_L		0x78
#define YA_OFFSET_H		0x7A
#define YA_OFFSET_L		0x7B
#define ZA_OFFSET_H		0x7D
#define ZA_OFFSET_L		0x7E

#define I2C_SLVx_EN		0x80
#define I2C_SLV4_DONE	0x40
#define I2C_SLV4_NACK	0x10

#define	I2C_ADDR    	0xD0	//IIC?g?J?E???a§}?r﹐`???U?A+1?°??‥u
#define DeviceID		0x71
 /*========================*/
//#define a		0.5
//#define dt 		0.01
//#define M_PI	3.14159265358979323846

 enum Mscale {
   MFS_14BITS = 0, // 0.6 mG per LSB
   MFS_16BITS      // 0.15 mG per LSB
 };

 /*
 |     |      ACCELEROMETER      |        GYROSCOPE        |
 | LPF | BandW | Delay  | Sample | BandW | Delay  | Sample |
 +-----+-------+--------+--------+-------+--------+--------+
 |  0  | 260Hz |    0ms |  1kHz  | 256Hz | 0.98ms |  8kHz  |
 |  1  | 184Hz |  2.0ms |  1kHz  | 188Hz |  1.9ms |  1kHz  |
 |  2  |  94Hz |  3.0ms |  1kHz  |  98Hz |  2.8ms |  1kHz  |
 |  3  |  44Hz |  4.9ms |  1kHz  |  42Hz |  4.8ms |  1kHz  |
 |  4  |  21Hz |  8.5ms |  1kHz  |  20Hz |  8.3ms |  1kHz  |
 |  5  |  10Hz | 13.8ms |  1kHz  |  10Hz | 13.4ms |  1kHz  |
 |  6  |   5Hz | 19.0ms |  1kHz  |   5Hz | 18.6ms |  1kHz  |
 |  7  | -- Reserved -- |  1kHz  | -- Reserved -- |  8kHz  |
 */
 typedef enum {
     MPU_GYRO_LPS_250HZ   = 0x00,
     MPU_GYRO_LPS_184HZ   = 0x01,
     MPU_GYRO_LPS_92HZ    = 0x02,
     MPU_GYRO_LPS_41HZ    = 0x03,
     MPU_GYRO_LPS_20HZ    = 0x04,
     MPU_GYRO_LPS_10HZ    = 0x05,
     MPU_GYRO_LPS_5HZ     = 0x06,
     MPU_GYRO_LPS_DISABLE = 0x07,
 } MPU_GYRO_LPF_TypeDef;

 typedef enum {
     MPU_ACCE_LPS_460HZ   = 0x00,
     MPU_ACCE_LPS_184HZ   = 0x01,
     MPU_ACCE_LPS_92HZ    = 0x02,
     MPU_ACCE_LPS_41HZ    = 0x03,
     MPU_ACCE_LPS_20HZ    = 0x04,
     MPU_ACCE_LPS_10HZ    = 0x05,
     MPU_ACCE_LPS_5HZ     = 0x06,
     MPU_ACCE_LPS_DISABLE = 0x08,
 } MPU_ACCE_LPF_TypeDef;

 typedef enum {
     MPU_GYRO_FS_250  = 0x00,
     MPU_GYRO_FS_500  = 0x08,
     MPU_GYRO_FS_1000 = 0x10,
     MPU_GYRO_FS_2000 = 0x18,
 } MPU_GYRO_FS_TypeDef;

 typedef enum {
     MPU_ACCE_FS_2G  = 0x00,
     MPU_ACCE_FS_4G  = 0x08,
     MPU_ACCE_FS_8G  = 0x10,
     MPU_ACCE_FS_16G = 0x18,
 } MPU_ACCE_FS_TypeDef;

 typedef enum {
     MPU_READ_ACCE = 1 << 0,
     MPU_READ_TEMP = 1 << 1,
     MPU_READ_GYRO = 1 << 2,
     MPU_READ_MAGN = 1 << 3,
     MPU_READ_ALL  = 0x0F,
 } MPU_READ_TypeDef;

 typedef enum {
     MPU_CORRECTION_PX = 0x01,
     MPU_CORRECTION_NX = 0x02,

     //三軸磁強寄存器
     MPU_CORRECTION_PY = 0x03,
     MPU_CORRECTION_NY = 0x04,
     MPU_CORRECTION_PZ = 0x05,
     MPU_CORRECTION_NZ = 0x06,
     MPU_CORRECTION_GYRO = 0x07,
     MPU_CORRECTION_CALCX = 0x08,
     MPU_CORRECTION_CALCY = 0x09,
     MPU_CORRECTION_CALCZ = 0x0A,
     MPU_CORRECTION_SAVE = 0x0B,
     MPU_CORRECTION_CIRCLE = 0x0C,
     MPU_CORRECTION_CIRCLEZ = 0x0D,
 } MPU_CORRECTION_TypeDef;

 //加速度帶寬設置和輸出速率配置寄存器
 /*
 +-------+----+----+----+
 |FCHOICE|DLPF|BW  |RATE|
 +-------+----+----+----+
 |0      |X   |1.13|4K  |
 |1      |0   |460 |1K  |
 |1      |1   |184 |1K  |
 |1      |2   |92  |1K  |
 |1      |3   |41  |1K  |
 |1      |4   |20  |1K  |
 |1      |5   |10  |1K  |
 |1      |6   |10  |1K  |
 |1      |7   |460 |1K  |
 +-------+----+----+----+
 */

//uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
//uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

char TEXT1[16], TEXT2[16], TEXT3[16];
uint8_t tmpL, tmpH,i2c_dev/*, i*/;
int16_t tmp, m[6]/*, buf[20]*/;
int temp;
int accX,accY,accZ,gyroX,gyroY,gyroZ,/*gyroX0,gyroY0,gyroZ0,*/magX,magY,magZ,temperature;
/*float anglex,angley,anglez,anglex1,angley1,anglez1,gyro_anglex1,gyro_angley1,gyro_anglez,gyro_anglez1,c;*/
//float pitch,roll,yaw,acc_phi,acc_theta,acc_psi,mag_phi,mag_theta,mag_psi;

#ifdef __cplusplus
}
#endif

#endif
