/*
 * READ_SENSORS.h
 *
 *  Created on: Dec 6, 2018
 *      Author: mic
 */

#ifndef READ_SENSORS_H_
#define READ_SENSORS_H_

#include "tim.h"
#include "main.h"
#include "FILTERS.h"
#include "adc.h"

void READ_SENSORS(struct SysStatusStruct *SysStruct, const double dt);

#endif /* READ_SENSORS_H_ */
