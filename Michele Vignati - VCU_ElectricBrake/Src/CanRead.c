/*
 * CanRead.c
 *
 *  Created on: Mar 18, 2019
 *      Author: mic
 */

#include "CanRead.h"


//#define SPEED_SENS 6.427381511E-3 	// mot% -> periph wheel speed  3993.0 / 4096.0 * 2 * pi / 60 * 0.1259 / 2 [m/s]
#define SPEED_SENS 6.3191415E-3 	// mot% -> periph wheel speed  3993.0 / 4096.0 * 2 * pi / 60 * 0.1238 / 2 [m/s] ruota tacchettata
#define TORQUE_SENS 8.183112E-3 	// 33.518 / 4096.0

void CanRead(struct CanStruct *MexStruct, CAN_RxHeaderTypeDef *header, unsigned char ReceivedMex[],struct ControllerStruct *ControlStatus){
	unsigned char Status;
	unsigned char Fault;
	short signed int receivedTorque;
	short signed int receivedSpeed;

	MexStruct->NewMexCan 	= 0;

	if (header->StdId == 0x142){
			HAL_GPIO_TogglePin(Led6_GPIO_Port,Led6_Pin);
			MexStruct->ComErrorCan 	= 0;

			receivedTorque = ReceivedMex[0] <<8 | ReceivedMex[1];
			receivedSpeed  = ReceivedMex[2] <<8 | ReceivedMex[3];

			ControlStatus -> Torque = receivedTorque * TORQUE_SENS;
			//FILTER1Torque( &ControlStatus, 0.05, 0.01);

			ControlStatus -> Speed  = receivedSpeed  * SPEED_SENS;
			//FILTER1Velo ( &ControlStatus, 0.05, 0.01);




			Status =  (ReceivedMex[4]>>4) & 0x0F;

			switch (Status) {
			case 2:
				ControlStatus->N  = 1;
				ControlStatus->FW = 0;
				ControlStatus->RW = 0;
				break;
			case 3:
				ControlStatus->N  = 0;
				ControlStatus->FW = 1;
				ControlStatus->RW = 0;
				break;
			case 4:
				ControlStatus->N  = 0;
				ControlStatus->FW = 0;
				ControlStatus->RW = 1;
				break;
			default:
				ControlStatus->N  = 0;
				ControlStatus->FW = 0;
				ControlStatus->RW = 0;
				break;
			}

			ControlStatus->Fault 	 = ReceivedMex[6];
			ControlStatus->ToggleBit = ReceivedMex[7] & 0x1;
			//MexStruct -> FaultCode =    ReceivedMex[5];

			/*} else {
			MexStruct->ComErrorCan = MexStruct->ComErrorCan + 1;
			if ((MexStruct->ComErrorCan)>10){
				MexStruct->ComErrorCan = 10;
			}*/
		}

}

