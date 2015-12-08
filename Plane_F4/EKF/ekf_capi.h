/*
 * ekf_capi.h
 *
 *  Created on: 2015Äê12ÔÂ5ÈÕ
 *      Author: kohill
 */

#ifndef EKF_CAPI_H_
#define EKF_CAPI_H_
#ifdef __cplusplus
extern "C"{
#endif
#include <stm32f4xx.h>
#include <arm_math.h>
#include <stdlib.h>
#include <stdio.h>
#include "mpu6050.h"
void ekfupdate(float da[],float T);



#define Sample_CLK 200

#define SamplePeriod (1.0f/Sample_CLK)
typedef float Quaternion[4];

void initQua(void);

void updateQuat(float x, float y, float z);
#ifdef __cplusplus

}
#endif

#endif /* EKF_CAPI_H_ */
