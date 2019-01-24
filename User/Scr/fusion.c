#include <stdio.h>
#include <stdint.h>
#include "math.h"
#include "mpu9250.h"
#include "MS5611.h"
#include "HMC5883L.h"
#include "fusion.h"

/*------------------------------*/
/*------------濾波--------------*/
/*------------------------------*/

void accel_gyxo_mag_filter()
{
//accel filter
	static int resultax = 0;
	static int resulttmpax[10] = {0};
	static int bufferCounterax = 0;
	static int totalax = 0;

	totalax -= resulttmpax[bufferCounterax];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpax[bufferCounterax] = accX;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalax += accX;		                    // 更新總和

    resultax = totalax / 10;					// 計算平均值，並輸入到一個固定變數中
    bufferCounterax++;		                    // 更新指針
    if (bufferCounterax == 10)		            // 到達隊列邊界
        bufferCounterax = 0;
    accX_f = resultax;

	static int resultay = 0;
	static int resulttmpay[10] = {0};
	static int bufferCounteray = 0;
	static int totalay = 0;

	totalay -= resulttmpay[bufferCounteray];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpay[bufferCounteray] = accY;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalay += accY;		                    // 更新總和

    resultay = totalay / 10;					// 計算平均值，並輸入到一個固定變數中
    bufferCounteray++;		                    // 更新指針
    if (bufferCounteray == 10)		            // 到達隊列邊界
        bufferCounteray = 0;
    accY_f = resultay;

	static int resultaz = 0;
	static int resulttmpaz[10] = {0};
	static int bufferCounteraz = 0;
	static int totalaz = 0;

	totalaz -= resulttmpaz[bufferCounteraz];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpaz[bufferCounteraz] = accZ;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalaz += accZ;		                    // 更新總和

    resultaz = totalaz / 10;					// 計算平均值，並輸入到一個固定變數中
    bufferCounteraz++;		                    // 更新指針
    if (bufferCounteraz == 10)		            // 到達隊列邊界
        bufferCounteraz = 0;
    accZ_f = resultaz;

//gyxo fliter
	static int resultgx = 0;
	static int resulttmpgx[2] = {0};
	static int bufferCountergx = 0;
	static int totalgx = 0;

	totalgx -= resulttmpgx[bufferCountergx];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpgx[bufferCountergx] = gyroX;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalgx += gyroX;		                    // 更新總和

    resultgx = totalgx / 2;						// 計算平均值，並輸入到一個固定變數中
    bufferCountergx++;		                    // 更新指針
    if (bufferCountergx == 2)		            // 到達隊列邊界
        bufferCountergx = 0;
    gyroX_f = resultgx;

	static int resultgy = 0;
	static int resulttmpgy[2] = {0};
	static int bufferCountergy = 0;
	static int totalgy = 0;

	totalgy -= resulttmpgy[bufferCountergy];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpgy[bufferCountergy] = gyroY;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalgy += gyroY;		                    // 更新總和

    resultgy = totalgy / 2;						// 計算平均值，並輸入到一個固定變數中
    bufferCountergy++;		                    // 更新指針
    if (bufferCountergy == 2)		            // 到達隊列邊界
        bufferCountergy = 0;
    gyroY_f = resultgy;

	static int resultgz = 0;
	static int resulttmpgz[2] = {0};
	static int bufferCountergz = 0;
	static int totalgz = 0;

	totalgz -= resulttmpgz[bufferCountergz];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpgz[bufferCountergz] = gyroZ;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalgz += gyroZ;		                    // 更新總和

    resultgz = totalgz / 2;						// 計算平均值，並輸入到一個固定變數中
    bufferCountergz++;		                    // 更新指針
    if (bufferCountergz == 2)		            // 到達隊列邊界
        bufferCountergz = 0;
    gyroZ_f = resultgz;

//mag filter
	static int resultmx = 0;
	static int resulttmpmx[5] = {0};
	static int bufferCountermx = 0;
	static int totalmx = 0;

	totalmx -= resulttmpmx[bufferCountermx];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpmx[bufferCountermx] = magX;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalmx += magX;		                    // 更新總和

    resultmx = totalmx / 5;						// 計算平均值，並輸入到一個固定變數中
    bufferCountermx++;		                    // 更新指針
    if (bufferCountermx == 5)		            // 到達隊列邊界
        bufferCountermx = 0;
    magX_f = resultmx;

	static int resultmy = 0;
	static int resulttmpmy[5] = {0};
	static int bufferCountermy = 0;
	static int totalmy = 0;

	totalmy -= resulttmpmy[bufferCountermy];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpmy[bufferCountermy] = magY;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalmy += magY;		                    // 更新總和

    resultmy = totalmy / 5;						// 計算平均值，並輸入到一個固定變數中
    bufferCountermy++;		                    // 更新指針
    if (bufferCountermy == 5)		            // 到達隊列邊界
        bufferCountermy = 0;
    magY_f = resultmy;

	static int resultmz = 0;
	static int resulttmpmz[5] = {0};
	static int bufferCountermz = 0;
	static int totalmz = 0;

	totalmz -= resulttmpmz[bufferCountermz];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpmz[bufferCountermz] = magZ;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalmz += magZ;		                    // 更新總和

    resultmz = totalmz / 5;						// 計算平均值，並輸入到一個固定變數中
    bufferCountermz++;		                    // 更新指針
    if (bufferCountermz == 5)		            // 到達隊列邊界
        bufferCountermz = 0;
    magZ_f = resultmz;
}

void MS5611_filter()
{
//Pressure
	static int resultPressure = 0;
	static int resulttmpPressure[10] = {0};
	static int bufferCounterPressure = 0;
	static int totalPressure = 0;

	Pressure *= 1000.0f;
	totalPressure -= resulttmpPressure[bufferCounterPressure];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpPressure[bufferCounterPressure] = (int)Pressure;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalPressure += (int)Pressure;		                    // 更新總和

    resultPressure = totalPressure / 10;						// 計算平均值，並輸入到一個固定變數中
    bufferCounterPressure++;		                    // 更新指針
    if (bufferCounterPressure == 10)		            // 到達隊列邊界
        bufferCounterPressure = 0;
    Pressure_f = ((float)resultPressure / 1000.0f);
    Pressure /= 1000.0f;

//Temperature
    Temperature *= 1000.0f;
	static int resultTemperature = 0;
	static int resulttmpTemperature[5] = {0};
	static int bufferCounterTemperature = 0;
	static int totalTemperature = 0;

	totalTemperature -= resulttmpTemperature[bufferCounterTemperature];	// 從總和中刪除頭部元素的值，履行頭部指針職責
    resulttmpTemperature[bufferCounterTemperature] = (int)Temperature;		// 將採樣值放到尾部指針處，履行尾部指針職責
    totalTemperature += (int)Temperature;		                    // 更新總和

    resultTemperature = totalTemperature / 5;						// 計算平均值，並輸入到一個固定變數中
    bufferCounterTemperature++;		                    // 更新指針
    if (bufferCounterTemperature == 5)		            // 到達隊列邊界
        bufferCounterTemperature = 0;
    Temperature_f = ((float)resultTemperature / 1000.0f);
    Temperature /= 1000.0f;
}

/*------------------------------*/
/*------------校正--------------*/
/*------------------------------*/
void gyro_offset()
{
	static short workstep;
	static int totalgyroX,totalgyroY,totalgyroZ;
	if(workstep<100)
	{
		totalgyroX += gyroX_f;
		totalgyroY += gyroY_f;
		totalgyroZ += gyroZ_f;
		workstep +=1;
	}
	else
	{
		gyroX_offset = totalgyroX / 100;
		gyroY_offset = totalgyroY / 100;
		gyroZ_offset = totalgyroZ / 100;
		//printf("%d,%d,%d\n",totalgyroX,totalgyroY,totalgyroZ);
		gyroX_f -= gyroX_offset;
		gyroY_f -= gyroY_offset;
		gyroZ_f -= gyroZ_offset;
	}
	//printf("%d,%d,%d\n",gyroX_f,gyroY_f,gyroZ_f);
}

/*void mag_center_correction()
{
	magR = sqrt((float)magX_f * (float)magX_f + (float)magY_f * (float)magY_f + (float)magZ_f * (float)magZ_f);
	if(((float)magX_f/magR) > magX_Max) magX_Max = magX_f;
	if(((float)magX_f/magR) < magX_min) magX_min = magX_f;
	magX_f = (magX_Max + magX_min)/2.0f;

	if(((float)magY_f/magR) > magY_Max) magY_Max = magY_f;
	if(((float)magY_f/magR) < magY_min) magY_min = magY_f;
	magY_f = (magY_Max + magY_min)/2.0f;

	if(((float)magZ_f/magR) > magZ_Max) magZ_Max = magZ_f;
	if(((float)magZ_f/magR) < magZ_min) magZ_min = magZ_f;
	magZ_f = (magZ_Max + magZ_min)/2.0f;
}
*/
/*------------------------------*/
/*------------轉換--------------*/
/*------------------------------*/
void Altimeter()
{
    MS5611_height_PT = 2927.0953f * (273.15f + Temperature_f / 100.0f) * log(101325.0f / Pressure_f) * 0.86f;
    MS5611_height_P = (101325.0f - Pressure_f) * 9.0f * 0.86f;
    //printf("%d,%d\n",(int)MS5611_height_PT,(int)MS5611_height_P);
}

void IMUupdata()
{
	//g_Pitch = 0.0;	// 範圍-180°~+180°	-->上正下負
	//g_Roll = 0.0;		// 範圍 -90°~+90°	-->右正左負
	//g_Yaw = 0.0;		// 範圍-180°~+180°	-->逆正順負

	float aX, aY, aZ, mX, mY, mZ;
	float recipNorm;
	float halfvx, halfvy, halfvz;	// 在當前載體座標系中，重力在三個軸上的分量
	float halfwx, halfwy, halfwz;   // 在當前載體座標系中，磁力在三個軸上的分量
	float halfex, halfey, halfez;	// 當前加速度計測得的重力加速度在三個軸上的分量與當前姿態在三個軸上的重力分量的誤差,這裡採用差的方式
	float hx, hy, hz, bx, bz;
	float inthalfex,inthalfey,inthalfez;
	float halfexmag,halfeymag,halfezmag;

	float gXf,gYf,gZf;
	float qa, qb, qc;

	//int a_Norm;

	gXf = (float)gyroX_f / 1000.0f / radius;
	gYf = (float)gyroY_f / 1000.0f / radius;
	gZf = (float)gyroZ_f / 1000.0f / radius;

	accR = sqrt((float)accX_f * (float)accX_f + (float)accY_f * (float)accY_f + (float)accZ_f * (float)accZ_f);
	magR = sqrt((float)magX_f * (float)magX_f + (float)magY_f * (float)magY_f + (float)magZ_f * (float)magZ_f);

	//加速度計及磁力計對陀螺儀調節
	aX = (float)accX_f / accR;
	aY = (float)accY_f / accR;
	aZ = (float)accZ_f / accR;

	mX = (float)magX_f / magR;
	mY = (float)magY_f / magR;
	mZ = (float)magZ_f / magR;

	Stm = aX * mX + aY * mY + aZ * mZ;
	Ctm = sqrt(1.0f - Stm * Stm);
	//Stm*=1000;
	//Ctm*=1000;
	//printf("%d,%d\n",(int)Ctm,(int)Stm);

	/*hx = 2.0f * mX * (0.5f - g_q2*g_q2 - g_q3*g_q3) + 2.0f * mY * (g_q1*g_q2 - g_q0*g_q3) + 2.0f * mZ * (g_q1*g_q3 + g_q0*g_q2);
	hy = 2.0f * mX * (g_q1*g_q2 + g_q0*g_q3) + 2.0f * mY * (0.5f - g_q1*g_q1 - g_q3*g_q3) + 2.0f * mZ * (g_q2*g_q3 - g_q0*g_q1);
	hz = 2.0f * mX * (g_q1*g_q3 - g_q0*g_q2) + 2.0f * mY * (g_q2*g_q3 + g_q0*g_q1) + 2.0f * mZ * (0.5f - g_q1*g_q1 - g_q2*g_q2);
	bx = sqrt(hx * hx + hy * hy);
	bz = hz;*/

	halfvx = -(g_q1 * g_q3 - g_q0 * g_q2);
	halfvy = -(g_q0 * g_q1 + g_q2 * g_q3);
	halfvz = -(g_q0 * g_q0 - 0.5f + g_q3 * g_q3);
	halfwx = Ctm/*sqrt(mX * mX + mY * mY)*/ * (0.5f - g_q2 * g_q2 - g_q3 * g_q3) + Stm/*mZ*/ * (g_q1 * g_q3 - g_q0 * g_q2);
	halfwy = Ctm/*sqrt(mX * mX + mY * mY)*/ * (g_q1 * g_q2 - g_q0 * g_q3) + Stm/*mZ*/ * (g_q0 * g_q1 + g_q2 * g_q3);
	halfwz = Ctm/*sqrt(mX * mX + mY * mY)*/ * (g_q0 * g_q2 + g_q1 * g_q3) + Stm/*mZ*/ * (0.5f - g_q1 * g_q1 - g_q2 * g_q2);

	//計算誤差
	halfex = 1.0f * (aY * halfvz - aZ * halfvy)/* + 0.0f * (mY * halfwz - mZ * halfwy)*/;
	halfey = 1.0f * (aZ * halfvx - aX * halfvz)/* + 0.0f * (mZ * halfwx - mX * halfwz)*/;
	halfez = /*0.0f * (aX * halfvy - aY * halfvx)*/ /*+ 0.5f * (mX * halfwy - mY * halfwx)*/0;
	//inthalfex = 1000.0f * (aY * halfvz - aZ * halfvy);
	//inthalfey = 1000.0f * (aZ * halfvx - aX * halfvz);
	//inthalfez = 1000.0f * (aX * halfvy - aY * halfvx);
	//halfexmag = 1000.0f * (mY * halfwy - mZ * halfwz);
	//halfeymag = 1000.0f * (mZ * halfwz - mX * halfwx);
	//halfezmag = 1000.0f * (mX * halfwx - mY * halfwy);
	//printf("%d,%d,%d\n",(int)halfex,(int)halfey,(int)halfez);
	//printf("%d,%d,%d\n",(int)inthalfex,(int)inthalfey,(int)inthalfez);
	//printf("%d,%d,%d\n",(int)halfexmag,(int)halfeymag,(int)halfezmag);

	//積分控制調整陀螺儀值
	g_integralFBx += g_Ki * halfex;
	g_integralFBy += g_Ki * halfey;
	g_integralFBz += g_Ki * halfez;
	gXf += g_integralFBx;
	gYf += g_integralFBy;
	gZf += g_integralFBz;
	//printf("%d,%d,%d\n",(int)g_integralFBx,(int)g_integralFBy,(int)g_integralFBz);

	//微分控制調整陀螺儀值
	g_differentialFBx = (halfex - halfex0) * g_Kd;
	g_differentialFBy = (halfey - halfey0) * g_Kd;
	g_differentialFBz = (halfez - halfez0) * g_Kd;
	gXf += g_differentialFBx;
	gYf += g_differentialFBy;
	gZf += g_differentialFBz;
	halfex0 = halfex;
	halfey0 = halfey;
	halfez0 = halfez;

	//比例控制調整陀螺儀值
	gXf += halfex * g_Kp;
	gYf += halfey * g_Kp;
	gZf += halfez * g_Kp;

	//陀螺儀姿態轉換
	gXf *= CNTLCYCLE * 0.5f;
	gYf *= CNTLCYCLE * 0.5f;
	gZf *= CNTLCYCLE * 0.5f;
	qa = g_q0;
	qb = g_q1;
	qc = g_q2;
	g_q0 += (-qb * gXf - qc * gYf - g_q3 * gZf);
	g_q1 += ( qa * gXf + qc * gZf - g_q3 * gYf);
	g_q2 += ( qa * gYf - qb * gZf + g_q3 * gXf);
	g_q3 += ( qa * gZf + qb * gYf -   qc * gXf);

	recipNorm = sqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
	g_q0 /= recipNorm;//g_q0 *= 1000;
	g_q1 /= recipNorm;//g_q1 *= 1000;
	g_q2 /= recipNorm;//g_q2 *= 1000;
	g_q3 /= recipNorm;//g_q3 *= 1000;

	g_Yaw   = atan2f(2 * g_q1 * g_q2 + 2 * g_q0 * g_q3, g_q1 * g_q1 + g_q0 * g_q0 - g_q3 * g_q3 - g_q2 * g_q2) * radius;// Yaw
	g_Pitch =  asinf(-2 * g_q1 * g_q3 + 2 * g_q0 * g_q2) * radius;// Roll
	g_Roll  = atan2f(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, g_q0 * g_q0 - g_q1 * g_q1 - g_q2 * g_q2 + g_q3 * g_q3) * radius;// Pitch
}

IMUupdata_1()//初始姿態融合
{
	//g_Pitch = 0.0;	// 範圍-180°~+180°	-->上正下負
	//g_Roll = 0.0;		// 範圍 -90°~+90°	-->右正左負
	//g_Yaw = 0.0;		// 範圍-180°~+180°	-->逆正順負

	float gXf,gYf,gZf;
	float qa, qb, qc;
	float recipNorm;

	gXf = (float)gyroX_f / 1000.0f / radius;
	gYf = (float)gyroY_f / 1000.0f / radius;
	gZf = (float)gyroZ_f / 1000.0f / radius;

	//陀螺儀姿態轉換
	gXf *= CNTLCYCLE * 0.5f;
	gYf *= CNTLCYCLE * 0.5f;
	gZf *= CNTLCYCLE * 0.5f;
	qa = g_q0_1;
	qb = g_q1_1;
	qc = g_q2_1;
	g_q0_1 += (-qb * gXf - qc * gYf - g_q3_1 * gZf);
	g_q1_1 += ( qa * gXf + qc * gZf - g_q3_1 * gYf);
	g_q2_1 += ( qa * gYf - qb * gZf + g_q3_1 * gXf);
	g_q3_1 += ( qa * gZf + qb * gYf -   qc * gXf);

	recipNorm = sqrt(g_q0_1 * g_q0_1 + g_q1_1 * g_q1_1 + g_q2_1 * g_q2_1 + g_q3_1 * g_q3_1);
	g_q0_1 /= recipNorm;//g_q0 *= 1000;
	g_q1_1 /= recipNorm;//g_q1 *= 1000;
	g_q2_1 /= recipNorm;//g_q2 *= 1000;
	g_q3_1 /= recipNorm;//g_q3 *= 1000;

	g_Yaw_1   = atan2f(2 * g_q1_1 * g_q2_1 + 2 * g_q0_1 * g_q3_1, g_q1_1 * g_q1_1 + g_q0_1 * g_q0_1 - g_q3_1 * g_q3_1 - g_q2_1 * g_q2_1) * radius;// Yaw
	g_Pitch_1 =  asinf(-2 * g_q1_1 * g_q3_1 + 2 * g_q0_1 * g_q2_1) * radius;// Roll
	g_Roll_1  = atan2f(2 * g_q2_1 * g_q3_1 + 2 * g_q0_1 * g_q1_1, g_q0_1 * g_q0_1 - g_q1_1 * g_q1_1 - g_q2_1 * g_q2_1 + g_q3_1 * g_q3_1) * radius;// Pitch
}

/*void IMUupdata_ex(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ)
{
		// 計算磁場的參考方向
		hx = 2 * magX * (0.5 - g_q2*g_q2 - g_q3*g_q3) + 2 * magY * (g_q1*g_q2 - g_q0*g_q3) + 2 * magZ * (g_q1*g_q3 + g_q0*g_q2);
		hy = 2 * magX * (g_q1*g_q2 + g_q0*g_q3) + 2 * magY * (0.5 - g_q1*g_q1 - g_q3*g_q3) + 2 * magZ * (g_q2*g_q3 - g_q0*g_q1);
		hz = 2 * magX * (g_q1*g_q3 - g_q0*g_q2) + 2 * magY * (g_q2*g_q3 + g_q0*g_q1) + 2 * magZ * (0.5 - g_q1*g_q1 - g_q2*g_q2);
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		// 將當前姿態的重力及磁力在三個軸上的分量分離出來
		// 就是方向余弦旋轉矩陣的第三列,注意是地理坐標系(n系)到載體坐標系(b系)的
		// 一切的量都要以飛機的坐標系來做參考,所以座標需要從地理坐標系上轉換到載體坐標系上
		//=================================================================//
		halfvx = 2 * (g_q1 * g_q3 - g_q0 * g_q2);
		halfvy = 2 * (g_q0 * g_q1 + g_q2 * g_q3);
		halfvz = g_q0 * g_q0 - 0.5f + g_q3 * g_q3;
		halfwx = 2 * bx * (0.5f - g_q2 * g_q2 - g_q3 * g_q3) + 2 * bz * (g_q1 * g_q3 - g_q0 * g_q2);
		halfwy = 2 * bx * (g_q1 * g_q2 - g_q0 * g_q3) + 2 * bz * (g_q0 * g_q1 + g_q2 * g_q3);
		halfwz = 2 * bx * (g_q0 * g_q2 + g_q1 * g_q3) + 2 * bz * (0.5f - g_q1 * g_q1 - g_q2 * g_q2);
		//=================================================================//

		// 計算由當前姿態的重力在三個軸上的分量與加速度計測得的重力在三個軸上的分量的差,這裡採用三維空間的差積(向量積)方法求差
		// 計算公式由矩陣運算推導而來
		halfex = (accY * halfvz - accZ * halfvy)+(magY * halfwy - magZ * halfwz);
		halfey = (accZ * halfvx - accX * halfvz)+(magZ * halfwz - magX * halfwx);
		halfez = (accX * halfvy - accY * halfvx)+(magX * halfwx - magY * halfwy);
}*/
