#ifndef CanWrite_H_
#define CanWrite_H_

#include "can.h"
#include "main.h"
#include "math.h"


HAL_StatusTypeDef CanWrite(CAN_TxHeaderTypeDef *CAN_TxHeader1, unsigned char CAN_TxMex[], uint32_t *pTxMailbox, struct ControllerStruct *ControlRef);

#endif
