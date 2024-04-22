/*
 * app_counter.h
 *
 *  Created on: Feb 5, 2022
 *      Author: Alessandro
 */

#ifndef INC_APP_COUNTER_H_
#define INC_APP_COUNTER_H_

#include "stm32h7xx_hal.h"
#include "math.h"


void StartCounter(void);

void TIM2_IRQHandler(void);
void COMP_IRQHandler(void);
double MeasProcess(void);
double getMeas(void);
void InitPeriodMeasure(void);
void StopPeriodMeasure(void);
void getVelues(double * valBuff);
void N_Measurement(double* ,int);
double statMean(double* measBuf,int n);
void correctMeasAll(double* measBuf,int n);
double correctMeas(double meas);
#endif /* INC_APP_COUNTER_H_ */
