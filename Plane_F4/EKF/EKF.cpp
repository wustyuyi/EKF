/*
 * EKF.cpp
 *
 *  Created on: 2015年12月4日
 *      Author: kohill
 */

#include "EKF.h"
#include <stdio.h>
typedef Matrix<float,9,9> Matrix9f;
typedef Matrix<float,6,9> Matrix6_9f;
typedef Matrix<float,9,6> Matrix9_6f;
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,9,1> Vector9f;
typedef Matrix<float,6,1> Vector6f;
/*
 * 变量定义规则：尾部带有下划线的为矢量，首字母大写的为矩阵，普通变量全小写且尾部不带下划线
 */
Vector3f b_omega_={0,0,0};//状态向量之角速度,b系
Vector3f b_beta_={0,0,1};//状态向量之角加速度,b系
Vector3f b_g_={0,0,0};//状态向量之角加速度,b系
Matrix9f A_link = Matrix9f::Zero(9,9);

const Matrix3f I = Matrix3f::Identity(3,3);
const Matrix3f ZERO = Matrix3f::Zero(3,3);
const Matrix6f R= Matrix6f::Identity(6,6)*9;
const Matrix9f Q= Matrix9f::Identity(9,9)*9;
Matrix6_9f H;
static Matrix9f P = Matrix9f::Zero(9,9);
static void update(float da[],float T){
	//由输入构造量测向量
	Vector6f z_;
	for(int i=0;i<6;i++){
		z_(i) = da[i];
	}
	//由旋转角b_beta_构造旋转矩阵
	Matrix3f A_3_3;
	A_3_3 <<
			1,            -b_omega_[2],  -b_omega_[1],
			-b_omega_[2],     1,          b_omega_[0],
			-b_omega_[1], -b_omega_[0],     1	;

	A_3_3 *= T;
	//计算先验估计
	Vector9f x_pri_;
	for(int i=0;i<3;i++){
		x_pri_(i) = b_omega_[i]+b_beta_[i]*T;
		x_pri_(i+3) = b_beta_[i];
		x_pri_[i+6] = b_g_[i] + (A_3_3 * b_g_)(i);
	}
	A_3_3 += Matrix3f::Identity(3,3);
	Matrix3f A_3_1;
	A_3_1<<0,-1,1,
			1,0,-1,
			-1,1,0;
	A_link<<  I,   I*T,   ZERO,
			ZERO,   I,    ZERO,
			A_3_1, ZERO,  A_3_3;
	P = A_link*P*A_link.transpose() + Q;
	H  = Matrix6_9f::Zero(6,9);
	for(int i = 0;i<3;i++){
		H(i,i) = 1;
	}
	for(int i=0,j=6;i<6 && j<9;i++,j++){
		H(i,j) = 1;
	}
	Matrix9_6f K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
	Vector9f x_;
	for(int i=0;i<3;i++){
		x_(i) =  b_omega_[i];
		x_(i+3) = b_beta_[i];
		x_(i+6) = b_g_[i];
	}
	x_ = x_pri_+K*(z_ - H*x_pri_);
	//更新状态量
	for(int i=0;i<3;i++){
		b_omega_[i] = x_[i];
		b_beta_[i] = x_[i+3];
		b_g_[i]= x_[i+6];
	}
	P = (Matrix9f::Identity(9,9)-K*H)*P;
}
#include <cmath>
#include <cstdlib>
using namespace std;
/*
 * @Function:由加速度传感器获得四元数
 */
static void getQuaterfromAccler(float Acceler[],float quater[4]){
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
static bool isInit=false;
static Vector4f q_;
void ekfupdate(float da[],float period){
	if(!isInit){
		float quater[4];
		getQuaterfromAccler(da+3,quater);
		q_<<quater[0],quater[1],quater[2],quater[3];
		isInit = true;
	}
	Matrix4f A;
	Vector3f theta;
	theta<<da[0]*period , da[1]*period ,da[2]*period;
	float m = theta.norm();
	A<<    cos(m/2),         -theta[0]/m*sin(m/2),      -theta[1]/m*sin(m/2),    theta[2]/m*sin(m/2),
 		theta[0]/m*sin(m/2),      cos(m/2),              theta[2]/m*sin(m/2),  -theta[2]/m*sin(m/2),
		theta[1]/m*sin(m/2),    -theta[2]/m*sin(m/2),         cos(m/2),          theta[0]/m*sin(m/2),
		theta[2]/m*sin(m/2),     theta[1]/m*sin(m/2),    -theta[0]/m*sin(m/2),       cos(m/2);
	q_ = A*q_;
	q_ = q_/q_.norm();
	printf("%f,%f,%f,%f    ",q_[0],q_[1],q_[2],q_[3]);
	Vector4f q =q_;
	float T[3];
	T[0] = 2*(q[1]*q[3]-q[0]*q[2]);
	T[1] = 2*(q[2]*q[3]+q[0]*q[1]);
	T[2] = q[3]*q[3]-q[2]*q[2]-q[1]*q[1]+q[0]*q[0];
	float the[3];
	the[0] = asin(T[1]);
	the[1] = atan(-T[0]/T[2]);
	the[2] = 0;
}



