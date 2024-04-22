/*
 * app_counter.c
 *
 *  Created on: Feb 5, 2022
 *      Author: Alessandro
 *
 *      USAGE:
 *      StartperiodMeasureContinuous() and StopperiodMeasureContinuous() MUST decomment in comparator isr
 *
 *
 */
#include "app_counter.h"
#include "main.h"
#include "comp.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

#define NMEAS 10
#define Tck 5*pow(10,-9)      //measurement timer period
#define volt2cod  4095/3.30   //dac conv factor

#define VoffsetDN 1.500//1.380   // voltage offset (vonnected to opamp inverting in - )
#define VoffsetUP 1.800//1.920   // voltage offset (vonnected to opamp inverting in - )

// CALIB DATA //
//#define calib 2.9300*pow(10,-6)//4.7100*pow(10,-7)
//#define calib -2.9300E-6//4.7100*pow(10,-7)
// y = C1*x+C0
#define C1 1.0257
#define C0 -3.0609

uint32_t Ncount;
double Tmeas;
uint8_t measFlag;
int compCall;

uint8_t id;

double measurements[NMEAS] = {0.0};
double mean;
double calibOffset;
double CorrectMeas[NMEAS] = {0.0};


void EnableDacVref(double VrefDN,double VrefUP){
	uint32_t dacOut = ceil(VrefDN*volt2cod);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R ,  dacOut);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	 dacOut = ceil(VrefUP*volt2cod);
	HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R ,  dacOut);
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

}

void InitPeriodMeasure(void){
	//precompute offset
	//calibOffset = calib;
	/*start DAC for comparator inverting input Vref*/
	EnableDacVref(VoffsetDN,VoffsetUP);

	/** START COMPARATOR (only first time)**/
	__disable_irq();

	compCall =0;
	WRITE_REG(EXTI->PR1, COMP_EXTI_LINE_COMP2);
	__HAL_COMP_CLEAR_C2IFLAG();
	if(HAL_COMP_Start_IT(&hcomp2) != HAL_OK) Error_Handler();

	WRITE_REG(EXTI->PR1, COMP_EXTI_LINE_COMP1);
	__HAL_COMP_CLEAR_C1IFLAG();
	if(HAL_COMP_Start_IT(&hcomp1) != HAL_OK) Error_Handler();

	__enable_irq();
}




void StartCounter(void){

	InitPeriodMeasure();
	//    HAL_Delay(80);
	//continuousMeasurement();
	while(1){
	N_Measurement(measurements,10);
	correctMeasAll(measurements,10);
	mean = statMean(measurements,10);
	HAL_Delay(1000);
	}

}



void continuousMeasurement(void){
	uint8_t i = 0;
	while(1){
		measurements[i] = MeasProcess()*1E6;
		CorrectMeas[i] = measurements[i]+calibOffset;
		HAL_Delay(1);
		i++;
		i = i%10;
	}
}

void N_Measurement(double* measBuf,int n){
	uint8_t i = 0;
	MeasProcess();
	while(i<n){
		measBuf[i] = MeasProcess()*1E6;
		HAL_Delay(1);
		i++;
		//i = i%10;
	}
}

void correctMeasAll(double* measBuf,int n){
	uint8_t i = 0;
	double x =0.00;
	MeasProcess();
	for(int i =0;i<n;i++){
		x = measBuf[i];
		measBuf[i] = C1*x+C0;
	}
}

double correctMeas(double meas){
	return  C1*meas + C0;
}

double statMean(double* measBuf,int n){
	double f = 3;
	double meanVal=0.00;
	double sum=0.00;
	double stdDev=0.00;
	int j=0;

	for(int i =0;i<n;i++)
		sum += measBuf[i];
	meanVal = sum/(double)n;
	sum =0.00;
	for(int i =0;i<n;i++)
		sum += pow((measBuf[i]-meanVal),2);
	stdDev = sqrt(sum/(double)n);
	sum = 0.00;
	for(int i =0;i<n;i++){
		if(abs(measBuf[i]-meanVal)<(f*stdDev)){
			sum += measBuf[i];
			j++;
		}
	}
	sum =0.00;
	for(int i =0;i<j;i++)
			sum += measBuf[i];
    meanVal = sum/(double)j;
	return meanVal;
}

double MeasProcess(void){
	//start measurement
	Ncount = 0;
	Tmeas = 0.00;
	measFlag = 0;
	//htim2.Instance->SR = ~TIM_FLAG_TRIGGER;
	htim2.Instance->CNT =0U;
	if(HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
	while(!measFlag){
		//wait for meas
	}
	//stop meas
	HAL_TIM_Base_Stop(&htim2);
	//HAL_COMP_Stop_IT(&hcomp2);
	return Tmeas;
}

double getMeas(void){
	//start measurement
	if(measFlag){
		HAL_TIM_Base_Stop(&htim2);
		measFlag = 0;
		htim2.Instance->SR = ~TIM_FLAG_TRIGGER;
		htim2.Instance->CNT =0U;
		HAL_TIM_Base_Start(&htim2);
		return Tmeas;
	}
	return 0;
}



void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){


}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles COMP1 and COMP2 global interrupt.
  */
void COMP_IRQHandler(void)
{
  /* USER CODE BEGIN COMP_IRQn 0 */
	Ncount = htim2.Instance->CNT;
		compCall++;
		if((!measFlag)  && ((htim2.Instance->SR & TIM_FLAG_TRIGGER) == TIM_FLAG_TRIGGER)){
			//htim2.Instance->CR1 &= ~TIM_CR1_CEN;
			 htim2.Instance->SR = ~TIM_FLAG_TRIGGER;
			 Tmeas = Ncount*Tck;
			 //HAL_TIM_Base_Stop(&htim2);
//			 measurements[id] = Ncount*Tck;
//			 id++;
//			 id = id%10;
//             //exp
//			 measFlag = 0;
//			 htim2.Instance->CNT =0U;
//			 HAL_TIM_Base_Start(&htim2);
             measFlag = 1;
		}

  /* USER CODE END COMP_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp2);
  HAL_COMP_IRQHandler(&hcomp1);
  /* USER CODE BEGIN COMP_IRQn 1 */
  /* USER CODE END COMP_IRQn 1 */
}
