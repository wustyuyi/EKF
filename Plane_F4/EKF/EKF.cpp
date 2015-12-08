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



