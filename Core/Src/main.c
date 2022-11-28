/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_VARRE  5             // inc varredura a cada 5 ms (~200 Hz)
#define DIGITO_APAGADO 0x10    // kte valor p/ apagar um dígito no display
#define DT_LEI 10            // Leitura a cada 250 ms
#define NVZ_D1 12 // num vezes piscar LED D1
#define DT_D1 100 // delay do LED D1
#define NVZ_D2 4 // num vezes piscar LED D2
#define DT_D2 600 // delay do LED D2
#define NVZ_D3 8 // num vezes piscar LED D3
#define DT_D3 300 // delay do LED D3
#define NVZ_D4 2 // num vezes piscar LED D4
#define DT_D4 1000 // delay do LED D4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
// funcoes do arquivo stm32f1xx_it.c
  void set_modo_oper(int);             // seta modo_oper (no stm32f1xx_it.c)
  int get_modo_oper(void);             // obtém modo_oper (stm32f1xx_it.c)
  // funcoes do arquivo prat_05_funcoes.c
  void reset_pin_GPIOs (void);         // reset pinos da SPI
  void serializar(int ser_data);       // prot fn serializa dados p/ 74HC595
  int16_t conv_7_seg(int NumHex);      // prot fn conv valor --> 7-seg
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  int16_t val_adc = 0;                 // var global: valor lido no ADC
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// vars e flags de controle do programa no superloop...
		  int milADC = 0,                      // ini decimo de seg
			  cenADC = 0,                      // ini unidade de seg
			  dezADC = 0,                      // ini dezena de seg
			  uniADC = 0;                      // ini unidade de minuto
		  int hora=0;							//flag hora
		  int16_t val7seg = 0x00FF,            // inicia 7-seg com 0xF (tudo apagado)
			  serial_data = 0x01FF;            // dado a serializar (dig | val7seg)
		  uint32_t miliVolt = 0x0,             // val adc convertido p/ miliVolts
			  tIN_varre = 0,                   // registra tempo última varredura
		  	  tIN_lei = 0;                     // registra tempo da última leitura
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // desliga o LED
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // desliga o LED
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // desliga o LED
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // desliga o LED
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // desliga o LED
    // inicializa a SPI (pinos 6,9,10 da GPIOB)
     reset_pin_GPIOs ();
     // var de estado que controla a varredura (qual display é mostrado)
     static enum {DIG_UNI, DIG_DEC, DIG_CENS, DIG_MILS} sttVARRE=DIG_UNI;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // tarefa #1: se (modo_oper=1) faz uma conversão ADC
	 	  	if ((HAL_GetTick()-tIN_lei) > DT_LEI)
	 	  	{
	 	  	  // dispara por software uma conversão ADC
	 	  	tIN_lei = HAL_GetTick();
	 	  	  set_modo_oper(0);                // muda modo_oper p/ 0
	 	  	  //HAL_ADC_Start_IT(&hadc1);        // dispara ADC p/ conversão por IRQ
	 	  	val_adc++;

	 	  	if(milADC + 1 == 10){
	 	  		milADC = 0;
	 	  		cenADC++;
	 	  	}
	 	  	else {
	 	  		milADC++;
	 	  	}


	 	  	if(cenADC == 6){
	 	  		cenADC = 0;
	 	  		milADC = 0;
	 	  		dezADC++;
	 	  	}
	 	  	if(dezADC==10){
	 	  		dezADC=0;
	 	  		cenADC=0;
	 	  		milADC=0;
	 	  		uniADC++;
	 	  	}
	 		if(uniADC==6){
	 		 	  		dezADC=0;
	 		 	  		cenADC=0;
	 		 	  		milADC=0;
	 		 	  		uniADC=0;
	 		 	  		hora++;
	 		 }



	 	  		 	  	set_modo_oper(2);
	 	  		 	  	}

	 	  //tarefa #2: depois do IRQ ADC, converte para mVs (decimal, p/ 7-seg)
	 	      if (get_modo_oper()==2)            // entra qdo valor val_adc atualizado
	 	      {
	 	        // converter o valor lido em decimais p/ display
	 	    	 /*milADC = val_adc/1000000;
	 	    	 cenADC = (val_adc - 1000000*milADC)/100000;
	 	    	 dezADC = (val_adc - 100000*cenADC)/10000;
				 uniADC = (val_adc - 10000*dezADC)/1000;

	 	        miliVolt = val_adc;
	 	        dezADC = miliVolt/10000;
	 	        uniADC = miliVolt-(dezADC*10000);
	 	        	 */
	 	       // milADC =val_adc;
	 	       // uniADC = miliVolt/100;
	 	      //  dezADC = (miliVolt-(uniADC*1000))/100;
	 	       // cenADC = (miliVolt-(uniADC*1000)-(dezADC*100))/10;
	 	     //  milADC = miliVolt-(uniADC*1000)-(dezADC*100)-(cenADC*10);
	 	        set_modo_oper(0);                // zera var modo_oper
	 	      }

	 	  // tarefa #3: qdo milis() > DELAY_VARRE ms, desde a última mudança
	 	    if ((HAL_GetTick()-tIN_varre) > DT_VARRE)    // se ++0,1s atualiza o display
	 	    {
	 	  	//   tIN_varre = HAL_GetTick();         // salva tIN p/ prox tempo varredura
	 	  	switch(sttVARRE)                   // teste e escolha de qual DIG vai varrer
	 	  	{
	 	  	  case DIG_MILS:
	 	  	  {
	 	  		sttVARRE = DIG_CENS;           // ajusta p/ prox digito
	 	  		serial_data = 0x0008;          // display #1
	 	  		val7seg = conv_7_seg(milADC);
	 	  		break;
	 	  	  }
	 	  	  case DIG_CENS:
	 	  	  {
	 	  		sttVARRE = DIG_DEC;            // ajusta p/ prox digito
	 	  		serial_data = 0x00004;         // display #2
	 	  		if(cenADC>0 || dezADC>0 || uniADC>0)
	 	  		{
	 	  		  val7seg = conv_7_seg(cenADC);
	 	  		} else {
	 	  		  val7seg = conv_7_seg(DIGITO_APAGADO);
	 	  		}
	 	  		break;
	 	  	  }
	 	  	  case DIG_DEC:
	 	  	  {
	 	  		sttVARRE = DIG_UNI;            // ajusta p/ prox digito
	 	  		serial_data = 0x0002;          // display #3
	 	  		if(dezADC>0 || uniADC>0)
	 	  		{
	 	  		  val7seg = conv_7_seg(dezADC);
	 	  		} else {
	 	  		  val7seg = conv_7_seg(DIGITO_APAGADO);
	 	  		}
	 	  		break;
	 	  	  }
	 	  	  case DIG_UNI:
	 	  	  {
	 	  		sttVARRE = DIG_MILS;           // ajusta p/ prox digito
	 	  		serial_data = 0x0001;          // display #3
	 	  		if(uniADC>0)
	 	  		{
	 	  		  val7seg = conv_7_seg(uniADC);
	 	  		  val7seg &=0x7FFF;            // liga o ponto decimal
	 	  		} else {
	 	  		  val7seg = conv_7_seg(DIGITO_APAGADO);
	 	  		}
	 	  		break;
	 	  	  }
	 	  	}  // fim case
	 	    tIN_varre = HAL_GetTick();           // tmp atual em que fez essa varredura
	 	    serial_data |= val7seg;              // OR com val7seg = dado a serializar
	 	    serializar(serial_data);             // serializa dado p/74HC595 (shift reg)
	 	    }  // -- fim da tarefa #3 - varredura do display

	 	  }    // -- fim do loop infinito

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 PB5 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
