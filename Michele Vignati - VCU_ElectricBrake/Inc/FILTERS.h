/*
 * FILTERS.h
 *
 *  Created on: Nov 26, 2018
 *      Author: mic
 */

#ifndef FILTERS_H_
#define FILTERS_H_
#include "main.h"

/*
 * apply First Order filter
 * x_k+1 = x_k + (u-x_k)*dt/tau
 * x_k 		previous value of x
 * u		new value of x
 * dt		integration time 1/freq
 * tau		characteristic time constant
 * x_k+1	filter output
 */

void FILTER1(double *u, double *x, const double tau, const double dt);
void FILTER1Velo(struct ControllerStruct *controllerStatus, const double tau, const double dt);
void FILTER1Acc(struct ControllerStruct *controllerStatus, const double tau, const double dt);
void FILTER1Torque( struct ControllerStruct *controllerStatus ,const double tau, const double dt);
void FILTER1Fx( struct ABSControlStruct *ABS_control ,const double tau, const double dt);

/*
 * apply Second Order filter
 * x = x0 + (u-x0)/tau/freq
 * x0 		previous value of x
 * u		new value of x
 * freq		integration frequency (dt)
 * tau		characteristic time constant
 * x		filter output
 */

void FILTER2(double *u, double *x, double *v, const double w0, const double h, const double dt);

/*
 * apply ramp limit to input
 * u 		new value of x
 * x0		previous value of x
 * slope	desired max slope
 * freq		integration frequency
 */

void RAMP(double *u, double *x, const double slope, const double dt); // u, x0, slope, freq



#endif /* FILTERS_H_ */
