#include <FILTERS.h>

void FILTER1(double *u, double *x, const double tau, const double dt){

	*x = *x + (*u - *x) / tau * dt;
};

void FILTER1Velo( struct ControllerStruct *controllerStatus ,const double tau, const double dt){
	double u;
	double x;
	//******************************************************
	u = (double) controllerStatus->Speed; // rinizializzazione con acc, per filtrare velo
	x = (double) controllerStatus->Speed_filt;

	controllerStatus -> Speed_filt0 = (double) x;

	x = x + (u - x) / tau * dt;

	controllerStatus->Speed_filt = x;
};

void FILTER1Acc( struct ControllerStruct *controllerStatus ,const double tau, const double dt){
	//*****************************************************
	double u = controllerStatus->Acc; // rinizializzazione con acc, per filtrare acc
	double x = controllerStatus->Acc_filt;

	x = x + (u - x) / tau * dt;

	controllerStatus->Acc_filt = x;
	//*******************************************************
};

void FILTER1Torque( struct ControllerStruct *controllerStatus ,const double tau, const double dt){
	//*****************************************************
	double u = controllerStatus->Torque; // rinizializzazione con acc, per filtrare acc
	double x = controllerStatus->Torque_filt;

	x = x + (u - x) / tau * dt;

	controllerStatus->Torque_filt = x;
	//*******************************************************
};

void FILTER1Fx( struct ABSControlStruct *ABS_control ,const double tau, const double dt){
	double u;
	double x;
	//******************************************************
	u = (double) ABS_control->Fx; // rinizializzazione con acc, per filtrare velo
	x = (double) ABS_control->FxFilt;

	ABS_control ->FxFilt0 = (double) x;

	x = x + (u - x) / tau * dt;

	ABS_control ->FxFilt = (double) x;
};




void FILTER2(double *u, double *x, double *v, const double w0, const double h, const double dt){
	*v = *v * (1-2 * h * w0 * dt) + w0 * w0 * (*u - *x) * dt;
	*x = *x + *v * dt;
};

void RAMP(double *u, double *x, const double slope, const double dt){
	double v = (*u - *x)/dt;
	double x0 = *u;
	if (v > slope){
		x0 = *x + slope*dt;
	}
	if (v < -slope){
		x0 = *x - slope*dt;
	}
	*x = x0;
};

