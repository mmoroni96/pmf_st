/*
 * SpeedCtrl.c
 *
 *  Created on: Mar 6, 2019
 *      Author: mic
 */
#include "CtrlSpeed.h"

#define SPEED_WHEEL_R  0.25
#define SPEED_ENC_SENS  0.019634954	// 1/pulses*2*pi*R = 1/80*2*pi*0.25
#define SPEED_AERO     0 	// [kg/m] 0.5*1.225*0.36*1.2/3.6/3.6 			// Aerodynamic force coefficient .5*rho*S*Cx
#define SPEED_ROLL     0.15 			// [N] 820*9.806*0.0198 			// rolling resistance force
#define SPEED_MASS     15.0 		// vehicle mass
#define SPEED_INERTIA  15.0 		// vehicle inertia
#define WHEEL_RADIUS 0.1259204

void CtrlSpeed(struct SpeedStruct *speed, struct ControllerStruct *ControlRef, const double dt){

	double errP, errI;
	double forceFF, forceFB, force;
	double Torque;

	if(speed->enable){

		// feed-forward force
		if (speed->v_ref < 1) {
			forceFF = SPEED_AERO * (speed->v_ref) * (speed->v_ref) + SPEED_INERTIA * (speed->a_ref) + SPEED_ROLL * (speed->v_ref);
		} else {
			forceFF = SPEED_AERO * (speed->v_ref) * (speed->v_ref) + SPEED_INERTIA * (speed->a_ref) + SPEED_ROLL; // acc in m/s2, speed in km/h
		}
		if (speed->v_ref == 0 && speed->a_ref == 0){
			forceFF = 0;
		}

		// feed-back force
		errP = speed->v_ref - speed->v;
		errI = speed->errI + errP * dt;
		forceFB = errP * (speed->kp) + errI * (speed->ki);
		speed->errI = errI;
		speed->errP = errP;

		if ((speed->v_ref < 1) && (errP < 0) ){
			if (forceFB > 0){
				forceFB = 0;
			}
		}

		// total force
		force = forceFF*0 + forceFB;
		Torque = force * WHEEL_RADIUS;
		ControlRef->Torque = Torque;
		//speed->force_max = force;

		//force = forceFF;
		/*if (force > (speed->force_max)){
			force = (speed->force_max);
			errI  = (force - forceFF - errP * (speed->kp) + errD * (speed->kd) ) / (speed->ki);
			if (errI<0){
				errI = 0;
			}
		}*/
		/*if (force < (speed->force_min)){
			force = (speed->force_min);
			errI  = (force - forceFF - errP * (speed->kp) + errD * (speed->kd) ) / (speed->ki);
			if (errI>0){
				errI = 0;
			}
		}*/

	};
}
