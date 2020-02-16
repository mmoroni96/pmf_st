#include "CanWriteBack.h"

HAL_StatusTypeDef CanWriteBack(CAN_TxHeaderTypeDef *CAN_TxHeader1, CAN_TxHeaderTypeDef *CAN_TxHeader2, unsigned char CAN_TxMex163[],
		unsigned char CAN_TxMex164[], uint32_t *pTxMailbox,struct ControllerStruct *ControlStatus,
		struct ControllerStruct *ControlRef, struct SysStatusStruct *SysStatus,
		struct SpeedStruct *CtrlSpeed, struct ABSControlStruct *ABS_controllo) {

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// prepare and send CAN message
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	short signed int Torque;
	Torque 		= (short signed int) round( (ControlStatus	-> Torque ) );

	short signed int SpeedW;
	SpeedW 		= (short signed int) round( (ControlStatus 	-> Speed)*1000.0 );

	short signed int Speed_filt;
	Speed_filt 	= (short signed int) round( (ControlStatus 	-> Speed_filt)*1000.0 );

	short signed int Acc_filt;
    Acc_filt 	= (short signed int) round( (ControlStatus 	-> Acc_filt)*500.0 );	//acc ruota kart


    short signed int SpeedD;
	SpeedD 		= (short signed int) round( (SysStatus		-> Speed)*400.0 ); //drum

	unsigned int Distance;
	Distance 	= (short signed int) round( SysStatus		-> Cnt);


    short signed int Fx;
    Fx 			= (short signed int) round( (ABS_controllo	-> Fx)*50.0 );

    short signed int Fmax;
    Fmax 		= (short signed int) round( (ABS_controllo	-> Fmax)* 50.0 );

    short signed int errFx;
    errFx 		= (short signed int) round( (ABS_controllo	-> errFx)*10000.0 );

    short signed int FxD_filt;
    FxD_filt 	= (short signed int) round( (ABS_controllo	-> FxD_filt)*50.0 );

    short signed int DFDT_filt;
    DFDT_filt 	= (short signed int) round( (ABS_controllo	-> DFDT_filt)*100.0 );

    short signed int phase;
    phase 		= (short signed int) round( (ABS_controllo	-> phase)*1.0 );


	short signed int SpeedWr;
	SpeedWr		= (short signed int) round( (CtrlSpeed		-> v_ref)*1000.0 );

	short signed int errP;
	errP		= (short signed int) round( (CtrlSpeed		-> errP)*1000.0 );

	short signed int errI;
	errI		= (short signed int) round( (CtrlSpeed		-> errI)*1000.0 );


	//================================================ messaggio 163
	CAN_TxMex163[0] = SpeedD 	>> 8  & 0xff;
	CAN_TxMex163[1] = SpeedD 	   & 0xff;


	CAN_TxMex163[2] = DFDT_filt	>> 8  & 0xff;
	CAN_TxMex163[3] = DFDT_filt    & 0xff;


	CAN_TxMex163[4] = Fx 		>> 8  & 0xff;
	CAN_TxMex163[5] = Fx 	  	   & 0xff;


	CAN_TxMex163[6] = FxD_filt 	>> 8 & 0xff;
	CAN_TxMex163[7] = FxD_filt	   & 0xff;


	//================================================ messaggio 164
	CAN_TxMex164[0] = errFx 	>> 8  & 0xff;
	CAN_TxMex164[1] = errFx 	   & 0xff;


	CAN_TxMex164[2] = Fmax 		>> 8  & 0xff;
	CAN_TxMex164[3] = Fmax 	  	   & 0xff;


	CAN_TxMex164[4] = Acc_filt 	>> 8  & 0xff;
	CAN_TxMex164[5] = Acc_filt     & 0xff;


	CAN_TxMex164[6] = phase 	>> 8 & 0xff;
	CAN_TxMex164[7] = phase        & 0xff;
	//================================================


	return (HAL_CAN_AddTxMessage(&hcan1, CAN_TxHeader1, CAN_TxMex163, pTxMailbox),
			HAL_CAN_AddTxMessage(&hcan1, CAN_TxHeader2, CAN_TxMex164, pTxMailbox));

}



	/*
	short unsigned int TorqueNegative;

	 if ((ControlRef->TorqueRef)>=0){
		TorquePositive = (short unsigned int) round( (ControlRef->TorqueRef) * 4096.0 / 100.0 );
		TorqueNegative = 0;
	} else {
		TorquePositive = 0;
		TorqueNegative = (short unsigned int) round( -(ControlRef->TorqueRef) * 4096.0 / 100.0 );
	} */


	/*short int steer_value_int = (short int) round( (*steer_angle) * 70.0 );	// * 70.0 );
	short unsigned int speed_value_int = (short unsigned int) round( (speed->v) * 1000.0 );		// * 1000.0  	km/h
	int dv_d = round( (speed->d_v) * 100.0 );
	if (dv_d>127){
		dv_d = 127;
	} else if (dv_d<-128){
		dv_d = -128;
	}
	unsigned char dv 		= (unsigned char) dv_d;		// * 100.0  delta v right-left in m/s
	unsigned char gas_int 	= (unsigned char) round( (*gas) / 11.764706);
	unsigned char press_int = (unsigned char) round( (*press) * 20.0 );
	unsigned char checksum = 0;*/


	//CAN_TxMex[2] = TorqueNegative >> 8 & 0xff;
	//CAN_TxMex[3] = TorqueNegative 	    & 0xff;

	/*CAN_TxMex[4] = gas_int; steer angle non serve
	CAN_TxMex[5] = press_int;*/

	/*CAN_TxMex[6] = 0;
	CAN_TxMex[6] = CAN_TxMex[6] | ControlRef->FW | ControlRef->RW <<1 |
			ControlRef->FootSwitch <<2 | ControlRef->SeatSwitch <<3;*/


	/*CAN_TxMex[7] = (*enable) << 4; il toggle è scritto nel main*/

	/*for (int i=0; i<7; i++){
		checksum += CAN_TxMex[i];
	}
	checksum += CAN_TxMex[7];
	checksum &= 0x0F;
	CAN_TxMex[7] = (*enable) << 4) | checksum;*/
