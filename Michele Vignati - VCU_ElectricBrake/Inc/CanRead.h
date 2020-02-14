/*
 * CanRead.h
 *
 *  Created on: Mar 18, 2019
 *      Author: mic
 */

#ifndef CANREAD_H_
#define CANREAD_H_

struct CanStruct{
	double ActualTorque;
	double ActualSpeed;
	unsigned char DriveStatus;
	unsigned char FaultCode;
	char ComErrorCan;
	char NewMexCan;
};

#include "main.h"
#include "can.h"

void CanRead(struct CanStruct *CanMexIn, CAN_RxHeaderTypeDef *header, unsigned char RxMex[], struct ControllerStruct *ControlStatus);

#endif /* CANREAD_H_ */
