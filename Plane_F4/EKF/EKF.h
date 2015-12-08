/*
 * EKF.h
 *
 *  Created on: 2015Äê12ÔÂ4ÈÕ
 *      Author: kohill
 */

#ifndef EKF_H_
#define EKF_H_
#ifndef __cplusplus
#error "This library need C++ linker."
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wctor-dtor-privacy"
#pragma GCC system_header
#pragma GCC push_options
#pragma GCC optimize ("O2")
#endif


#include "./Eigen/Eigen"
#include <iostream>
using namespace std;

#ifdef __GNUC__
#pragma GCC diagnostic pop
#pragma GCC pop_options
#endif

using namespace Eigen;
#include <stdint.h>

/*
 *Parameters:da[7]
 *	da[0..2],ax, ay and az
 *	da[3..5],gx, gy and gz
 *	da[6],temperature*100
 */
extern "C" void ekfupdate(float da[],float T);


#endif /* EKF_H_ */
