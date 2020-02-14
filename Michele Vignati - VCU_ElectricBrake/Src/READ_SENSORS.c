#include "READ_SENSORS.h"
int speed_count = 0;
int WR_Pulse0, WL_Pulse0;
unsigned int 	enc_cnt;
unsigned int    enc_cnt0;

/* Name : ADC2Val[0] 510, 634, 477, 571, 3, 201 */
#define BRAKE_P_SENS		1.0		  // bits2MPa
#define BRAKE_PRESSF_SENS 	0.0390625 // 40 MPa / 1024
#define SPEED_ENC_PULSE  	2500		// pulses/turn
#define SPEED_SENS 			0.0007669903939 // 2 * PI/ (4096 ppr x 4 fronti / 2 presc) (rad/s)
#define SPEED_FILTER_TAU    0.25

void READ_SENSORS(struct SysStatusStruct *SysStatus, const double dt){

	unsigned int 	adc2_value[4];
	unsigned int 	W_Pulse;
	double 			SpeedW;
	double 			Speed;

	/* steer encoder */
	enc_cnt = TIM3->CNT; //

	if((enc_cnt<1000) & (enc_cnt0>(0xffff-1000))){
		Speed = ((double)((double)enc_cnt+(double)(0xffff-enc_cnt0)))/dt * SPEED_SENS;

	} else if ((enc_cnt0<1000) & (enc_cnt>(0xffff-1000))) {
		Speed = ((double)((double)(0xffff-enc_cnt) +(double)enc_cnt0))/(-dt) * SPEED_SENS;

	} else {
		Speed = ((double)((double)enc_cnt-(double)enc_cnt0))/dt * SPEED_SENS;
	}

	//Speed = ((double)((double)enc_cnt-(double)enc_cnt0))/dt * SPEED_SENS;

	SysStatus->Speed = Speed;

	enc_cnt0 = enc_cnt;

	SysStatus->Cnt = enc_cnt;

	/* analog inputs */
	HAL_ADC_Start(&hadc2);
	for (int ii=0; ii<4; ii++){
		HAL_ADC_PollForConversion(&hadc2,5);
		adc2_value[ii] = HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
	SysStatus->Throttle = adc2_value[0]; // IN0 - PC0 - CN11, 38 - analog input for throttle


	/*steer->x 	= steer_angle; 	// read the encoder counter and apply conversion to degrees
	steer->v  	= (steer->x - steer_angle)*dt;
	//FILTER1(&steerVM,&(steer->v),0.05,0.005);*/

	/* wheel speed enc */
	/*speed_count++;
	if ( speed_count > 4 ){
		WR_Pulse 	= TIM1->CNT;
		WL_Pulse 	= TIM8->CNT;
		speedR 		= ((double) (WR_Pulse0 - WR_Pulse)) * 1.404; //20.0 / 80.0 * 0.248278 * 6.2831853 * 3.6 ;
		speedL 		= ((double) (WL_Pulse0 - WL_Pulse)) * 1.404; //20.0 / 80.0 * 0.248278 * 6.2831853 * 3.6 ;
		speedM 		= (speedR + speedL)/2.0;
		speed->v    = speedM;
		speed->d_v  = (speedR-speedL)/3.6;
		//FILTER1(&speedM,&(speed->v),SPEED_FILTER_TAU,0.05);
		speed_count = 0;
		WR_Pulse0 	= WR_Pulse;
		WL_Pulse0 	= WL_Pulse;
	};*/


	/*
	 * brake->pF  	= ((double)adc2_value[0]) * BRAKE_PRESSF_SENS;
	*/

}
