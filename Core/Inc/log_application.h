/*
 * log_application.h
 *
 *  Created on: Feb 1, 2022
 *      Author: Alessandro
 */

#ifndef SRC_LOG_APPLICATION_H_
#define SRC_LOG_APPLICATION_H_


#include "stm32h7xx_hal.h"


#define ADC_CLOCK_SOURCE_AHB



 //void init_acquisition(void);
 //void CreateNewFile( UART_HandleTypeDef uartOut );
 void waitNewData(void);
 void OpenLogFile(void);
 void CloseLogFile(void);
 void WriteDataToFile(void);
 void LoggerTask(void);
 void LoggerTaskComplete(void);
 void CreateInfoFile(uint32_t Nwaves, uint32_t Nsample, uint32_t Nmeas);

 //adc conf
 void start_acquisition(void);
 void stop_acquisition(void);
 void my_TIM1_Config(void);
 void my_toggle();
 void my_InitADC(void);
 void DMA1_Stream1_IRQHandler(void);
 void bsp_GetAdcValues(void);
 double getMaxVpp(void);

#endif /* SRC_LOG_APPLICATION_H_ */
