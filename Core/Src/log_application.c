/*
 * log_application.c
 *
 *  Created on: Feb 1, 2022
 *      Author: Alessandro
 */
#include "log_application.h"

//#include <stdio.h>
//#include <string.h>
#include "fatfs.h"
#include "sdmmc.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

// USART for pc com and debug
#define USE_PC_SERIAL 1
#define PIN_DEBUG_ON  1
#define uartOut  huart3
uint8_t uartBuff[32]={"\0"};  //USART buffer

// ADC READINGS BUFFFER
#define cod2volt 1.8311E-4
#define buffSize 32768 //65536 //32768   //16384  //8192 //4096
 uint16_t ADCxValues[buffSize];
 //ALIGN_32BYTES(__attribute__((section (".RAM_D3"))) uint16_t ADCxValues[buffSize]);
 double Voffset;

 // double buffer 1 or 2 indicate dma-busy half buffer
 int s_DmaFlag ; //=0
 int toggle_state;
 TIM_HandleTypeDef htim1;
 //TIM_HandleTypeDef htim2;
 ADC_HandleTypeDef hadc1;
 DMA_HandleTypeDef   DmaHandle;

 /// FATFS common objects
  uint8_t retSD;    // Return value for SD
  char SDPath[4];   // SD logical drive path
  FATFS SDFatFS;    // File system object for SD logical drive
  DIR DirInf;
  FILINFO FileInf;
  FIL file;
  FRESULT result;

  char filePath[32]; // buff for path of file to open


  uint16_t datiProva[5] = {3214,5050,3343,343,3434};
  uint8_t newData;

  /// PERIOD MEASUREMENT
  double timeMeas[10] = {0.0};


void start_acquisition(void){
newData = 0;

//HAL_TIM_Base_Start_IT(&htim2);
//if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)  Error_Handler();
//if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCxValues, buffSize) != HAL_OK)  Error_Handler();

/*  ##-6-Start ADC DMA transfer ##################################### */
htim1.Instance->SR = ~TIM_FLAG_TRIGGER; // wait next trig to be synchro
if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCxValues, buffSize) != HAL_OK)
  {
      Error_Handler();
  }

}

void stop_acquisition(void){
newData = 0;

//HAL_TIM_Base_Start_IT(&htim2);
//if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)  Error_Handler();
//if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCxValues, buffSize) != HAL_OK)  Error_Handler();

/*  ##-6-Start ADC DMA transfer ##################################### */
if(HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }

}


void HAL_TIM_TriggerCallback(TIM_HandleTypeDef * htim){
	if(PIN_DEBUG_ON) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
  //not called, go to stm32h7xx_it.c
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  newData = 1;
  if(PIN_DEBUG_ON) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
  //HAL_TIM_Base_Stop_IT(&htim2);
  if(HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)  Error_Handler();
  HAL_ADC_Stop_DMA(&hadc1);

  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCxValues, buffSize);
  //HAL_TIM_Base_Start_IT(&htim2);
}


//-------FATFS-----------------

void waitNewData(void){

	if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Waiting new data...\n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}

	while(!newData){

	}
	newData =0;

	if(USE_PC_SERIAL){
		sprintf(uartBuff, "get new data: len %d \n", sizeof(ADCxValues));
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}
}

void OpenLogFile(void){

	// Mount file system
	result = f_mount(&SDFatFS, SDPath, 1);            /* Mount a logical drive */
	if (result != FR_OK) Error_Handler();

	// open log file
	sprintf(filePath, "%smeas1.txt", SDPath);
	result = f_open(&file, filePath, FA_CREATE_ALWAYS | FA_WRITE); // prova FA_OPEN_APPEND puntatore alla fine
	if (result != FR_OK) Error_Handler();
}

void CloseLogFile(void){
	//  Shut down file
	result = f_close(&file);
	if (result != FR_OK) Error_Handler();

	//  Uninstall file system
	result = f_mount(NULL, SDPath, 0);
	if (result != FR_OK) Error_Handler();
}


void WriteDataToFile(void)
{
    uint32_t bw =0U;

    if(PIN_DEBUG_ON) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);

    result = f_write(&file, (uint8_t*)ADCxValues, sizeof(ADCxValues), &bw);
    if (result != FR_OK) Error_Handler();

    if(PIN_DEBUG_ON) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);

    if(USE_PC_SERIAL){
    	if (result == FR_OK) sprintf(uartBuff, "write success: bw %d\n", bw);
    	else sprintf(uartBuff, "file write error: cod %d ", result);
    	HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
    }

}

void CreateInfoFile(uint32_t Nwaves, uint32_t Nsample, uint32_t Nmeas){
	// open log file
	uint32_t dims[3] = {Nwaves,Nsample,Nmeas};
	FIL Infofile;
	uint8_t infoWriteBuf[100] = {"\0"};
	uint32_t bw =0U;
    //filePath = {"\0"};
	sprintf(filePath, "%sInfoFile.txt", SDPath);
	result = f_open(&Infofile, filePath, FA_CREATE_ALWAYS | FA_WRITE); // prova FA_OPEN_APPEND puntatore alla fine
	if (result != FR_OK) Error_Handler();

	result = f_write(&Infofile, (uint8_t*)dims, sizeof(dims), &bw);
	if (result != FR_OK) Error_Handler();

	//logging period measurement result
	result = f_write(&Infofile, (uint8_t*)timeMeas, sizeof(timeMeas), &bw);
	if (result != FR_OK) Error_Handler();

	sprintf(infoWriteBuf, "\n\nOnde: %d\nNum campioni: %d\nFreq: 2MHz\n", Nwaves, Nsample);
	result = f_write(&Infofile, (uint8_t*)infoWriteBuf, strlen(infoWriteBuf), &bw);
	if (result != FR_OK) Error_Handler();

	result = f_close(&Infofile);
	if (result != FR_OK) Error_Handler();

}


void LoggerTaskComplete(void){

	my_InitADC();
	//start_acquisition();
    uint8_t is_running = 1;
    uint8_t maxTransferts = 4;
    uint8_t transferts =0;

    if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Hello, mounting SD\n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
    }

    OpenLogFile();

	if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Waiting new data...\n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}

	newData=0;
	double measBuff =0.00;

	InitPeriodMeasure();
	start_acquisition();
  int k =0;

    while(transferts<maxTransferts){
    	measBuff = getMeas();
    	if(measBuff) timeMeas[k] = measBuff;

	   if(newData){
			stop_acquisition();// move to dma isr ?
			if(USE_PC_SERIAL){
				sprintf(uartBuff, "get new data: len %d \n", sizeof(ADCxValues));
				HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
			}
			WriteDataToFile();
			transferts++;
			newData =0;
			start_acquisition();
	   }
    }



			//start_acquisition();



	if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Bye, unmounting SD \n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}

	CreateInfoFile(transferts, buffSize,0);

	CloseLogFile();

}

double getMaxVpp(void){
	int Nstop = 2048;
	uint16_t max =0;
	uint16_t min =16384;

	for(int i =0;i< Nstop;i++){
		uint16_t val = ADCxValues[i];

		if(val > max) max = val;
		if(val < min) min = val;
	}

	 Voffset = (max-min)*cod2volt;
	 return Voffset;
}


void LoggerTask(void){

	my_InitADC();
	//start_acquisition();
    uint8_t is_running = 1;
    uint8_t maxTransferts = 4;
    uint8_t transferts =0;

    if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Hello, mounting SD\n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
    }

    OpenLogFile();

	if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Waiting new data...\n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}

	newData=0;


	start_acquisition();

	while(is_running){

		if(newData){
			stop_acquisition();// move to dma isr ?
			if(USE_PC_SERIAL){
				sprintf(uartBuff, "get new data: len %d \n", sizeof(ADCxValues));
				HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
			}
			WriteDataToFile();
			transferts++;
			newData =0;



			if(transferts < maxTransferts){
				start_acquisition();
				is_running = 1;
			}
			else is_running = 0;
		}

	}

	InitPeriodMeasure();
	N_Measurement(timeMeas,10);

	if(USE_PC_SERIAL){
		sprintf(uartBuff, "%s", "Bye, unmounting SD \n");
		HAL_UART_Transmit(&uartOut,uartBuff,sizeof(uartBuff),10);
	}

	CreateInfoFile(transferts, buffSize,10);

	CloseLogFile();

}



//////##############  adc cnf


/*
*********************************************************************************************************
* Function name: bsp_GetAdcValues
 * Function description: Get ADC data and print
 * Formal parameter: None
 * Return value: None
*********************************************************************************************************
*/
void bsp_GetAdcValues(void)
{
    uint32_t values;
    float  temp;

    //  The current DMA operation is the second half of the buffer, read the first 4 values ??of the first half of the buffer to average
    if(s_DmaFlag == 1)
    {
        //DISABLE_INT(); da pescare chi sa dove
        s_DmaFlag = 0;
        values = (ADCxValues[0] + ADCxValues[1] + ADCxValues[2] + ADCxValues[3])/4;

        //ENABLE_INT();
    }
    //  The current DMA operation is the last buffer, read the first 4 values ??of the second half buffer and average
    else if(s_DmaFlag == 2)
    {
        //DISABLE_INT();  da pescare chi sa dove
        s_DmaFlag = 0;
        values = (ADCxValues[64] + ADCxValues[65] + ADCxValues[66] + ADCxValues[67])/4;
        //ENABLE_INT();
    }

    //  Print the serial port value read out
    temp = values *3.3 / 65536;

    //printf("ADCxValues = %d, %5.3f\r\n", values, temp);
}

void my_toggle(){
  if(toggle_state){
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
   toggle_state=0;
  }
  else{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
    toggle_state=1;
  }


}

/*
*********************************************************************************************************
 * Function name: DMA1_Stream1_IRQHandler  WITHOUT MPU  (cache maintenance)
 * Function description: DMA1 Stream1 interrupt service routine
 * Formal parameter: None
 * Return value: None
*********************************************************************************************************
*/
void DMA1_Stream1_IRQHandler(void)
{
    /*  Transfer complete interrupt */
    if((DMA1->LISR & DMA_FLAG_TCIF1_5) != RESET)
    {
        my_toggle();
        s_DmaFlag = 2;
        newData = 1;
        //stop_acquisition();
        /*  Clear flag */
        DMA1->LIFCR = DMA_FLAG_TCIF1_5;
    }

    /*  Half transfer completed interrupt */
    if((DMA1->LISR & DMA_FLAG_HTIF1_5) != RESET)
    {
        s_DmaFlag = 1;
        /*  Clear flag */
        DMA1->LIFCR = DMA_FLAG_HTIF1_5;
    }

    /*  Transmission error interrupt */
    if((DMA1->LISR & DMA_FLAG_TEIF1_5) != RESET)
    {
        /*  Clear flag */
        DMA1->LIFCR = DMA_FLAG_TEIF1_5;
    }

    /*  Direct mode error interrupt */
    if((DMA1->LISR & DMA_FLAG_DMEIF1_5) != RESET)
    {
        /*  Clear flag */
        DMA1->LIFCR = DMA_FLAG_DMEIF1_5;
    }
}


    /*
    ******************************************************************************************************
 * Function name: my_InitADC
 * Function description: Initialize ADC
 * Parameter: None
 * Return value: None
    ******************************************************************************************************
    */
void my_InitADC(void)
{
//    ADC_HandleTypeDef   AdcHandle = {0};
//    AdcHandle = hadc1;

    ADC_ChannelConfTypeDef   sConfig = {0};
    GPIO_InitTypeDef         GPIO_InitStruct;
    /*  ##-1-Configure ADC sampling clock ##################################### */
  #if defined (ADC_CLOCK_SOURCE_PLL)
    /*  Configure the PLL2 clock to 72MHz, which is convenient for frequency division to generate the highest ADC clock 36MHz */
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 8;
    PeriphClkInitStruct.PLL2.PLL2N = 288;
    PeriphClkInitStruct.PLL2.PLL2P = 4;
    PeriphClkInitStruct.PLL2.PLL2Q = 4;
    PeriphClkInitStruct.PLL2.PLL2R = 4;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
  #elif defined (ADC_CLOCK_SOURCE_AHB)

  /*  If AHB clock is used, no configuration is required, default selection*/

  #endif

  /*  ##-2-Configure the collection pins used for ADC sampling ################################### ## */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*  ##-3-Configure the clock used for ADC sampling ##################################### */
  __HAL_RCC_DMA1_CLK_ENABLE();
  DmaHandle.Instance                 = DMA1_Stream1;            /*  DMA1 Stream1 used */
  DmaHandle.Init.Request             = DMA_REQUEST_ADC1;        /*  The request type uses DMA_REQUEST_ADC1 */
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /*  The transfer direction is from the peripheral to the memory */
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;        /*  Peripheral address auto-increment prohibited */
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;         /*  Memory address increment enable */
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;/*  Peripheral data transmission bit width selection half word, 16bit */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;/*  Memory data transmission bit width select half word, 16bit */
  DmaHandle.Init.Mode                = DMA_NORMAL;  //DMA_CIRCULAR;            /*  one shot */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;        /*  Low priority */
  DmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;    /*  Disable FIFO*/
  DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL; /*  Disable FIFO This bit has no effect and is used to set the threshold */
  DmaHandle.Init.MemBurst      = DMA_MBURST_SINGLE;       /*  Disable FIFO this bit has no effect, used for memory burst */
  DmaHandle.Init.PeriphBurst   = DMA_PBURST_SINGLE;      /*  Disable FIFO this bit has no effect, it is used for peripheral burst */
  /*  Initialize DMA */
  if(HAL_DMA_Init(&DmaHandle) != HAL_OK)
  {
      Error_Handler();
  }

  /*  Enable interrupt of DMA1 Stream1 */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  /*  Associate ADC handle and DMA handle */
  __HAL_LINKDMA(&hadc1, DMA_Handle, DmaHandle);

  /*  ##-4-Configure ADC ######################################## ################ */
  __HAL_RCC_ADC12_CLK_ENABLE();
  hadc1.Instance = ADC1;

  #if defined (ADC_CLOCK_SOURCE_PLL)
    AdcHandle.Init.ClockPrescaler  = ADC_CLOCK_ASYNC_DIV2;     /*  Using PLL asynchronous clock, divided by 2, that is 72MHz/2
                                                                    = 36MHz */
  #elif defined (ADC_CLOCK_SOURCE_AHB)
    hadc1.Init.ClockPrescaler  = ADC_CLOCK_SYNC_PCLK_DIV4;  /*  Using AHB synchronous clock, divided by 4, that is, 200MHz/4
                                                                      = 50MHz */
  #endif
    hadc1.Init.Resolution            = ADC_RESOLUTION_14B;   /*  16-bit resolution */
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;     /*  Scanning is prohibited because only one channel is opened */
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;  /*  EOC conversion end flag */
    hadc1.Init.LowPowerAutoWait      = DISABLE;              /*  Disable low-power automatic delay feature */
    hadc1.Init.ContinuousConvMode    = DISABLE;            /*  Prohibit automatic conversion, the timer used to trigger the conversion */
    hadc1.Init.NbrOfConversion       = 1;         /*  1 conversion channel used */
    hadc1.Init.DiscontinuousConvMode = DISABLE;   /*  Discontinuous mode is prohibited */
    hadc1.Init.NbrOfDiscConversion   = 1;         /*  After discontinuous mode is disabled, this parameter is ignored, this bit is used to configure Number of channels in discontinuous subgroups */
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T1_CC1;            /*  CC1 trigger of timer 1 */
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /*  Rising edge trigger */
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT; //ADC_CONVERSIONDATA_DMA_CIRCULAR; /*  DMA cycle mode to receive ADCConverted data */
  //AdcHandle.Init.BoostMode    = ENABLE;                 /*  If the ADC clock exceeds 20MHz, enable boost */
    hadc1.Init.Overrun    = ADC_OVR_DATA_OVERWRITTEN; /*  If the ADC conversion overflows, overwrite the ADC data register */
    hadc1.Init.OversamplingMode         = DISABLE;    /*  Prohibit oversampling */

  /*  Initialize ADC */
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }

  /*  Calibrate ADC, using offset calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  /*  Configure ADC channel  */
  sConfig.Channel      = ADC_CHANNEL_10;              /*  Configure the ADC channel used */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /*  The first in the sampling sequence */
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;     /*  The sampling period  */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /*  Single-ended input */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /*  No offset */
  sConfig.Offset = 0;                                 /*  In the case of no offset, this parameter is ignored */

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }

  /*  ##-5-Configure ADC timer trigger ##################################### */
  my_TIM1_Config();

  /*  ##-6-Start ADC DMA transfer ##################################### */
//  if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)ADCxValues, buffSize) != HAL_OK)
//  {
//      Error_Handler();
//  }
//  if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

}



/**
    ******************************************************************************************************
 * Function name: TIM1_Config
 * Function description: Configure TIM1, used to trigger ADC, currently configured trigger frequency of 100KHz
 * Parameter: None
 * Return value: None
 -----------------------------------------------------------------------
 The void SystemClock_Config(void) function in the bsp.c file configures the clock as follows:

           System Clock source       = PLL (HSE)
           SYSCLK(Hz)                = 200000000 (CPU Clock) %my is <= 280M ??
            HCLK(Hz)                  = 200000000 (AXI and AHBs Clock)
          AHB Prescaler             = 2
           D1 APB3 Prescaler         = 2 (APB3 Clock  100MHz)
          D2 APB1 Prescaler         = 2 (APB1 Clock  100MHz)
            D2 APB2 Prescaler         = 2 (APB2 Clock  100MHz)
           D3 APB4 Prescaler         = 2 (APB4 Clock  100MHz)

 Because APB1 prescaler != 1, so TIMxCLK on APB1 = APB1 x 2 = 200MHz;
Because APB2 prescaler != 1, so TIMxCLK on APB2 = APB2 x 2 = 200MHz;
 The TIMxCLK on APB4 has no frequency division, so it is 100MHz;

 APB1 timers include TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14, LPTIM1
 APB2 timers include TIM1, TIM8, TIM15, TIM16, TIM17
 APB4 timer has LPTIM2, LPTIM3, LPTIM4, LPTIM5

    TIM12CLK = 200MHz/(Period + 1) / (Prescaler + 1) = 200MHz / 2000 / 1 = 100KHz 1999 arr ,0 psc
    Duty cycle = Pulse / (Period + 1) = 1000 / (1999+1) = 50%         ->1000 pulse
    -----------------------------------------------------------------------
    TIM12CLK = 200MHz/(Period + 1) / (Prescaler + 1) = 200MHz / 200 / 1 = 1MHz -> 199 arr , 0 psc
    Duty cycle = Pulse / (Period + 1) = 100 / (199+1) = 50%    ->100 pulse
      -----------------------------------------------------------------------
    TIM12CLK = 200MHz/(Period + 1) / (Prescaler + 1) = 200MHz / 100 / 1 = 2MHz -> 99 arr , 0 psc
    Duty cycle = Pulse / (Period + 1) = 50 / (99+1) = 50%    -> 50 pulse
 *****************************************************************************************************
**/
void my_TIM1_Config(void)
{

    TIM_OC_InitTypeDef sConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    //TIM_MasterConfigTypeDef sMasterConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*  Enable clock */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();


	/**TIM2 GPIO Configuration
	PA0     ------> TIM2_ETR
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

   HAL_TIM_Base_DeInit(&htim1);

    htim1.Instance = TIM1;
   htim1.Init.Period            = 99;
   htim1.Init.Prescaler         = 0;
   htim1.Init.ClockDivision     = 0;
   htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.RepetitionCounter = 0;
   HAL_TIM_Base_Init(&htim1);

   sConfig.OCMode     = TIM_OCMODE_PWM1;
   sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

   /*  50% duty cycle */
   sConfig.Pulse = 50;
   if(HAL_TIM_OC_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1) != HAL_OK)
   {
       Error_Handler();
   }

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ETRF;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
	sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}

   /*  Start OC1 */
//    if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
//  {
//       Error_Handler();
//  }
}
