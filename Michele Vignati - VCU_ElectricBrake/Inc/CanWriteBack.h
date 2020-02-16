#ifndef CanWriteBack_H_
#define CanWriteBack_H_

#include "can.h"
#include "CanRead.h"
#include "main.h"
#include "math.h"


//void CanWriteBack(CAN_TxHeaderTypeDef *CAN_TxHeader1, unsigned char CAN_TxMex[], uint32_t *pTxMailbox, CAN_MexIn);
HAL_StatusTypeDef CanWriteBack(CAN_TxHeaderTypeDef *CAN_TxHeader1, CAN_TxHeaderTypeDef *CAN_TxHeader2, unsigned char CAN_TxMex163[], unsigned char CAN_TxMex164[], uint32_t *pTxMailbox,
		struct ControllerStruct *ControlStatus, struct ControllerStruct *ControlRef,
		struct SysStatusStruct *SysStatus, struct SpeedStruct *ctrlSpeed,struct ABSControlStruct *ABS_controllo);
#endif
