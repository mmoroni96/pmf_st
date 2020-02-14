#ifndef Controllo_H_
#define Controllo_H_

#include "can.h"
#include "main.h"
#include "math.h"

void Controllo(struct ControllerStruct *ControlStatus, struct ABSControlStruct *ABS_control, struct ControllerStruct *ControlRef);

#endif
