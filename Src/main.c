/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "et_stm32f_arm_kit_lcd.h"
#include <string.h>
	int noteon = 144;
	int noteoff = 128;
	int note = 0;
	int volume=0;
	//int velocity = 0;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int count=0;
int bufferKey[5]={0,0,0,0,0};
int keyIndex=0; //------------------------------------------------check current key index

int checkBuffer(int index,int key){
		for(int i=0;i<index;i++){
			if(bufferKey[i]==key)return 0;
		}
		return 1;
}
int bufferIsEmpty(){
		for(int i=0;i<5;i++){
			if(bufferKey[i]!=0)return 0;
		}
		return 1;
}
int clearBuffer(){
		for(int i=0;i<5;i++){
			bufferKey[i]=0;
		}
}
void playSound(int note,int vol){	
	char aa[]="";
	char bb[]="";
	char cc[]="";
	
	aa[0] = noteon;
	bb[0] = note;
	cc[0] = vol;
			
	HAL_UART_Transmit(&huart2, (uint8_t*)aa,strlen(aa),100);
	HAL_UART_Transmit(&huart2, (uint8_t*)bb,strlen(bb),100);
	HAL_UART_Transmit(&huart2, (uint8_t*)cc,strlen(cc),100);			
		
}

void offSound(int note , int vol){	
	char bb[]="";
	char cc[]="";
	char dd[]="";
	
	dd[0] = noteoff;
	bb[0] = note;
	cc[0] = vol;
			
	HAL_UART_Transmit(&huart2, (uint8_t*)dd,strlen(dd),100);
	HAL_UART_Transmit(&huart2, (uint8_t*)bb,strlen(bb),100);
	HAL_UART_Transmit(&huart2, (uint8_t*)cc,strlen(cc),100);			
	
}
void playAllSound(int volume){
	for(int i=0;i<5;i++){
		if(bufferKey[i]!=0) playSound(bufferKey[i],volume);
	}
}
void offAllSound(int volume){
	for(int i=0;i<5;i++){
		if(bufferKey[i]!=0) offSound(bufferKey[i],volume);
	}
}

int changeVol(int level){
			if(level<=1){				 
				volume = 5;
			}
			else if(level==2){				 
				volume =15;
			}
			else if(level==3){				 
				volume =25;
			}
			else if(level==4){				
				volume =35;				
			}
			else if(level==5){				
				volume =45;
			}
			else if(level==6){				
				volume =55;
			} 
			else if(level==7){				
				volume =70;
			} 
			else if(level==8){				 
				volume =85;
			}
			else if(level==9){				
				volume =100;
			} 
	

}
void drawButton(uint16_t color,int xpos,int ypos,int w,int h){
	int temp=xpos,temp2=ypos;	
	LCD_SetBackColor(color);
		do{
		do{
			LCD_DisplayChar(ypos,xpos,' ');
			xpos-=16;
		}while(xpos>=temp-(16*w));
		ypos+=21;
		xpos=temp;
	}while(ypos<temp2+(21*h));
		
}	
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
		uint16_t posX, posY;
    uint16_t color;
		char pos[50];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	volatile uint32_t adc_val;
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start(&hadc1);
		int volume=0;
		int level = 0;
		LCD_Setup();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i= 320;
	int line=0;
	int menu=0; //Menu 0 ==Main Menu
	int j=320;
	
	int printed=0;
	HAL_TIM_Base_Start_IT (&htim1);
	
	while (1)
  {
//---------Read Position X&Y------------------
		posX = TCS_Read_X();
		posY = TCS_Read_Y();
		
		if(menu==0){
			if(printed==0){
			LCD_Clear(White);
			LCD_SetBackColor(Blue);
			LCD_SetTextColor(White);
			LCD_DisplayStringLine(Line0, "     MINI PIANO     ");
			LCD_SetBackColor(White);
			LCD_SetTextColor(Black);
			LCD_DisplayStringLine(Line2, "    Select  Menu    ");
			LCD_DisplayStringLine(Line7, " Instrument Game   ");
			drawButton(Red,270,90,5,3);
			drawButton(Red,140,90,5,3);
			LCD_SetBackColor(White);
			
			printed=1;
		}
				if(posX!=0&&posY!=0){
				if(posX>160&&posY>120){
					menu=1; //instrument
					LCD_Clear(White);
					printed=0;
					posX=0;
					posY=0;
				}
				if(posX<=160&&posY>120){
					color = Blue;
				}
			}
		}
		if(menu==1){
			if(printed==0){
				LCD_SetBackColor(Blue);
				LCD_SetTextColor(Black);
				LCD_DisplayStringLine(Line0, " Select Instrument  ");
				LCD_SetBackColor(White);
				LCD_DisplayStringLine(Line1, "    Ins1    Ins2    ");
				LCD_DisplayStringLine(Line5, "    Ins3    Ins4    ");
				drawButton(Red,270,50,5,3);
				drawButton(Red,140,50,5,3);
				drawButton(Red,270,145,5,3);
				drawButton(Red,140,145,5,3);
				LCD_SetBackColor(Green);
				LCD_SetTextColor(Black);
				LCD_DisplayStringLine(Line9, "        Back        ");
				LCD_SetBackColor(White);
				printed=1;
			}
		if(posX!=0&&posY!=0){
			if(posY>240-24){
				menu=0;
				printed=0;
			}
			if(posX>160&&posY<140&&posY>=16){
				LCD_Clear(Blue);
			}
			if(posX<=160&&posY<140&&posY>=16){
				LCD_Clear(Red);
			}
			if(posX>160&&posY>=140&&posY<240-24){
				LCD_Clear(Yellow);
			}
			if(posX<=160&&posY>=140&&posY<240-24){
				LCD_Clear(Green);
			}
			
		}
	}
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		while(HAL_ADC_PollForConversion(&hadc1,100)!=HAL_OK){}
//---------------Adjust Volumn---------------------------------		
			adc_val=HAL_ADC_GetValue(&hadc1);			
			int level=(adc_val*10)/4096;
			volume = changeVol(level);
			
//------------Check Key press-----------------------------------
			
			if(bufferKey[0]==0)count=0;
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_SET&&checkBuffer(keyIndex,36)){
					bufferKey[keyIndex]=36;
					if(keyIndex<5-1) keyIndex++;
			}else
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==GPIO_PIN_SET&&checkBuffer(keyIndex,38)){
					bufferKey[keyIndex]=38;
					if(keyIndex<5-1) keyIndex++;
			}
			else
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_SET&&checkBuffer(keyIndex,40)){
					bufferKey[keyIndex]=40;
					if(keyIndex<5-1) keyIndex++;
			}
			else
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET&&checkBuffer(keyIndex,41)){
					bufferKey[keyIndex]=41;
					if(keyIndex<5-1) keyIndex++;
			}
			if(count>=50){
			playAllSound(volume);
			HAL_Delay(200);
			offAllSound(volume);
			count=0;
			clearBuffer();
			keyIndex=0;
			}
			
			

			/*
			char text[100]="";
			sprintf(text,"%d\n\r",count);
			while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
				HAL_UART_Transmit(&huart2,(uint8_t*)text,strlen(text),1000);
			*/
			
  /* USER CODE END 3 */

}
	}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA11 
                           PA12 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 
                           PB12 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
