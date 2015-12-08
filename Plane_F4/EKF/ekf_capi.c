/*
 * ekf_capi.c
 *
 *  Created on: 2015年12月9日
 *      Author: kohill
 */




/*
 * atitude.c
 *
 *  Created on: 2015骞�8鏈�21鏃�
 *      Author: yangkeshan
 */

//PB10,PB11
//I2C 2
#include "ekf_capi.h"

static Quaternion cuQua = {0,0,0,0}; //当前四元数的值
#define AcWeightFactor 0

/*
 * @Function:取相反数
 */
inline static float o(float x) {
	return -1.0f * x;
}

/*
 * @Function:四元数到欧拉角
 */
static void quat2Angle(float quat[],float euler[]){
	float r[5];
	r[0] = -2 * (quat[1] * quat[2] - quat[0] * quat[3]);
	r[1] = quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2]- quat[3] * quat[3];
	r[2] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
	r[3] = -2 * (quat[1] * quat[3] - quat[0] * quat[2]);
	r[4] = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2]+ quat[3] * quat[3];
	euler[0] = atan2(r[0], r[1]) * 180 / PI;
	euler[1] = asin(r[2]) * 180 / PI;
	euler[2] = atan2(r[3], r[4]) * 180 / PI;
}

/*
 * @Function:由加速度传感器获得四元数
 */
static void getQuaterfromAccler(float quater[4]){
	s16 Acce[3] = { 0,0,0 };
	float Acceler[3] = {0,0,0};
	float sum = 0;
	MPU_Get_Accelerometer(&Acce[0],&Acce[1],&Acce[2]);
	for(int i = 0;i<3;i++){
		Acceler[i] = 1.0*Acce[i]/32767*2; //满量程4g，-32767-+32767
		sum+=Acceler[i]*Acceler[i];
	}
	sum = sqrt(sum);
	for(int i = 0;i<3;i++){
		Acceler[i] /= sum;
	}
	float pitch  = asin(Acceler[1]);	//求俯仰角
	float roll = atan(-Acceler[0]/Acceler[2]);		//求横滚角
	//由加速度传感器无法得到偏航角
	pitch /=2;
	roll /=2;
	//得到四元数
	quater[0] = cos(pitch)*cos(roll);
	quater[1] = sin(pitch)*cos(roll);
	quater[2] = cos(pitch)*sin(roll);
	quater[3] = sin(pitch)*sin(roll);
}
/*
 * @Function: 更新四元数的值
 * @Parameter: x,y,z为当前时刻角速度
 */
void updateQuat(float x, float y, float z) {
	float exqu[4];
	getQuaterfromAccler(exqu);

	float delteTheta[4][4];
	float temp0 = 0;

	//获得角增量
	x *= SamplePeriod;
	y *= SamplePeriod;
	z *= SamplePeriod;

	temp0 = 1 - 0.125f * (x * x + y * y + z * z);

	x *= 0.5f;
	y *= 0.5f;
	z *= 0.5f;

	delteTheta[0][0] = temp0;
	delteTheta[0][1] = o(x);
	delteTheta[0][2] = o(y);
	delteTheta[0][3] = o(z);

	delteTheta[1][0] = x;
	delteTheta[1][1] = temp0;
	delteTheta[1][2] = z;
	delteTheta[1][3] = o(y);

	delteTheta[2][0] = y;
	delteTheta[2][1] = o(z);
	delteTheta[2][2] = temp0;
	delteTheta[2][3] = x;

	delteTheta[3][0] = z;
	delteTheta[3][1] = y;
	delteTheta[3][2] = o(x);
	delteTheta[3][3] = temp0;

	for (int i = 0; i < 4; i++) {
		cuQua[i] = delteTheta[i][0] * cuQua[0] + delteTheta[i][1] * cuQua[1]
				+ delteTheta[i][2] * cuQua[2] + delteTheta[i][3] * cuQua[3];
	}

	float sumsq;
	arm_sqrt_f32(
			cuQua[0] * cuQua[0] + cuQua[1] * cuQua[1] + cuQua[2] * cuQua[2]
					+ cuQua[3] * cuQua[3], &sumsq);
	for (int i = 0; i < 4; i++) {
		cuQua[i] = cuQua[i] / sumsq;
	}
	for(int  i = 0;i<4;i++){
		cuQua[i] = AcWeightFactor*exqu[i] + (1-AcWeightFactor)*cuQua[i];
	}
	float euler[3];
	quat2Angle(cuQua,euler);
}

void initQua(){
	getQuaterfromAccler(cuQua);
	getQuaterfromAccler(cuQua);
	getQuaterfromAccler(cuQua);
}



