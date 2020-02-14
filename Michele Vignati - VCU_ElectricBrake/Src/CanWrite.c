#include "CanWrite.h"

Boolean Toggle = 1;

HAL_StatusTypeDef CanWrite(CAN_TxHeaderTypeDef *CAN_TxHeader1, unsigned char CAN_TxMex[], uint32_t *pTxMailbox, struct ControllerStruct *ControlRef){
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// prepare and send CAN message
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	short unsigned int TorquePositive;
	short unsigned int TorqueNegative;
	if ((ControlRef->Torque)>=0){
		TorquePositive = (short unsigned int) round( (ControlRef->Torque) * 4096.0 / 33.5180 );
		TorqueNegative = 0;
	} else {
		TorquePositive = 0;
		TorqueNegative = (short unsigned int) round( -(ControlRef->Torque) * 4096.0 / 33.5180 );
	}

	CAN_TxMex[0] = TorquePositive >> 8 & 0xff; /*L'ordine era giusto, devo solo dividere 1 16 bit*/
	CAN_TxMex[1] = TorquePositive 	   & 0xff;
	CAN_TxMex[2] = TorqueNegative >> 8 & 0xff;
	CAN_TxMex[3] = TorqueNegative 	   & 0xff;

	CAN_TxMex[6] = 0;
	CAN_TxMex[6] = CAN_TxMex[6] | ControlRef->FW | ControlRef->RW <<1 |
			ControlRef->FootSwitch <<2 | ControlRef->SeatSwitch <<3;

	Toggle ^= 1;
	CAN_TxMex[7] = Toggle;

	return (HAL_CAN_AddTxMessage(&hcan1, CAN_TxHeader1, CAN_TxMex, pTxMailbox));
}
