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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



#define adxl355Scale 256000.0f

typedef enum {
   SPI_WRITE_ONE_REG = 1,         /* Write 1 ACC register */
   SPI_WRITE_TWO_REG,             /* Write 2 ACC register */
   UART_WRITE_NO_INT,             /* Write data when interrupts are disabled */
   UART_WRITE_IN_INT,             /* Write data while in an interrupt routine */
   UART_WRITE
} enWriteData;

typedef enum {
   SPI_READ_ONE_REG = 1,            /* Read one ACC register */
   SPI_READ_TWO_REG,                /* Read two ACC registers */
   SPI_READ_THREE_REG,              /* Read X,Y,Z ACC registers */

} enRegsNum;

#define ADXL355_WRITE 0x0
#define ADXL355_READ 0x1
#define POWER_CTL 0x2D

#define XDATA3 0x08
#define YDATA3 0x0B
#define ZDATA3 0x0E

#define RANGE 0x2C

#define SCALE 256000.0f

uint8_t data_tx[2];

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

char segCode[11][8] = {
//  a  b  c  d  e  f  g  .
  { 1, 1, 1, 1, 1, 1, 0, 0},  // 0
  { 0, 1, 1, 0, 0, 0, 0, 0},  // 1
  { 1, 1, 0, 1, 1, 0, 1, 0},  // 2
  { 1, 1, 1, 1, 0, 0, 1, 0},  // 3
  { 0, 1, 1, 0, 0, 1, 1, 0},  // 4
  { 1, 0, 1, 1, 0, 1, 1, 0},  // 5
  { 1, 0, 1, 1, 1, 1, 1, 0},  // 6
  { 1, 1, 1, 0, 0, 0, 0, 0},  // 7
  { 1, 1, 1, 1, 1, 1, 1, 0},  // 8
  { 1, 1, 1, 1, 0, 1, 1, 0},  // 9
  { 0, 0, 0, 0, 0, 0, 0, 1}   // .
};

void set7SegOutput(int value){
	//if(0 <= value && value <= 9){
		int outputs = 0;
		for(int i = 0; i < 7; i++){
			outputs |= segCode[value%10][i] << i;
		}
		int actualOuputs = ((outputs & (1 << 6)) << 4) | ((outputs & ( 1 << 5)) << 4 )|
						   ((outputs & (1 << 4)) << 4 )| ((outputs & (1 << 3)) << 1 )| (outputs & 0x7);
		GPIOA->ODR &= ~(1 << 0 | 1 << 1 | 1 << 2 | 1 << 4 | 1 << 8 | 1 << 9 | 1 << 10);
		GPIOA->ODR |= actualOuputs;
		// GPIOA->ODR |= 0x7 | 1 << 4 | 1 << 8 | 1 << 9 | 1 << 10;
	//}
}

void selectAccelerometer(){
	GPIOA->ODR &= ~(1 << 11);
}
void deselectAccelerometer(){
	GPIOA->ODR |= 1 << 11;
}

void setAccelMeasurementRange(){

}
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode)
{
   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address <<1)|ADXL355_WRITE);

   if(enMode == SPI_WRITE_ONE_REG) {
      selectAccelerometer();        /* Select accelerometer */

      // SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);      /* Flush Tx and Rx FIFOs */
      data_tx[0] = ui8writeAddress;
      data_tx[1] = ui8Data;
      HAL_SPI_Transmit(&hspi1, &ui8writeAddress, 1, 1000);     /* Send register address */

      HAL_SPI_Transmit(&hspi1, &ui8Data, 1, 1000);             /* Send value to be written */
      HAL_Delay(500);
      // while (!(SPI1->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
      // while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_TWOBYTES) != SPI0STA_RXFSTA_TWOBYTES);        /* Wait until 2  bytes are received */ --- 2 ou 3??
      HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 2, 1000);

      deselectAccelerometer();         /* Deselect accelerometer */
   }

}

uint32_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{

   uint32_t ui32Result = 0;

   uint8_t ui32valueL = 0;
   uint8_t ui32valueM __attribute__((unused)) = 0;
   uint8_t ui32valueH = 0;

   uint8_t testingSomething[3];
   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address <<1)|ADXL355_READ);

   selectAccelerometer();      /* Select accelerometer */

   // SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);       /* Flush Tx and Rx FIFOs */

   HAL_SPI_Transmit(&hspi1, &ui8writeAddress, 1, 1000);       /* Send register address */

   // uint8_t dummyInfo = 0xAA;
   // HAL_SPI_Transmit(&hspi1, &dummyInfo, 1, 100);               /* Send a dummy byte in order to receive the register value */

   if (enRegs == SPI_READ_ONE_REG) {

	  // HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 2, 100);    /* Wait until 2 bytes are received */

      ui32valueL = HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 1, 100);;            /* One dummy read */

      ui32Result = HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 1, 100);;            /* Set read result*/

   }
   if (enRegs == SPI_READ_TWO_REG) {          /* Only used for Temp & X,Y,Z offset and threshold registers*/

	  // HAL_SPI_Transmit(&hspi1, &dummyInfo, 1, 100);                /* Send a dummy byte in order to receive the register value */

	  // HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 3, 100);    /* Wait until 3 bytes are received */

      HAL_SPI_Receive(&hspi1, &ui32valueL, 1, 100);             /* One dummy read */


      HAL_SPI_Receive(&hspi1, &ui32valueH, 1, 100);         /* Read the register value */
      HAL_SPI_Receive(&hspi1, &ui32valueL, 1, 100);

      ui32Result = ((ui32valueH << 8) | ui32valueL); /* Set read result*/

   }
   if (enRegs == SPI_READ_THREE_REG) {          /* Only used for X,Y,Z axis data registers*/

	  // HAL_SPI_Transmit(&hspi1, &dummyInfo, 1, 100);               /* Send a dummy byte in order to receive the register value */
	  // HAL_SPI_Transmit(&hspi1, &dummyInfo, 1, 100);               /* Send a dummy byte in order to receive the register value */

	  // HAL_SPI_Receive(&hspi1, (uint8_t*)NULL, 4, 100);		      /* Wait until 4 bytes are received */

       // HAL_SPI_Receive(&hspi1, &ui32valueL, 1, 1000);             /* One dummy read */
	   // while (!(SPI1->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty


       HAL_SPI_Receive(&hspi1, &ui32valueH, 1, 1000);             /* Read the register value */
       HAL_SPI_Receive(&hspi1, &ui32valueM, 1, 1000);
       HAL_SPI_Receive(&hspi1, &ui32valueL, 1, 1000);
      // HAL_SPI_Receive(&hspi1, testingSomething, 3, 100);


      ui32Result = ((ui32valueH << 16) | (ui32valueM << 8) | ui32valueL); /* Set read result*/

   }
   HAL_Delay(500);
   deselectAccelerometer();          /* Deselect accelerometer */

   return ui32Result;
}

void ADXL355_Start_Sensor(void)
{
   // SPI_Write(RANGE, 0x81, 0x00, SPI_WRITE_ONE_REG);
	SPI_Write(RANGE, 0x01, 0x00, SPI_WRITE_ONE_REG);

   // uint8_t ui8temp;

   // ui8temp = (uint8_t)SPI_Read(POWER_CTL, SPI_READ_ONE_REG);       /* Read POWER_CTL register, before modifying it */

   // ui8temp = ui8temp & 0xFE;                                          /* Set measurement bit in POWER_CTL register */

   SPI_Write(POWER_CTL, 0x06, 0x00, SPI_WRITE_ONE_REG);                    /* Write the new value to POWER_CTL register */
}

int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = (ui32SensorData  >> 4);
   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){

         i32Conversion = (ui32SensorData | 0xFFF00000);

   }
   else{
         i32Conversion = ui32SensorData;
   }

   return i32Conversion;
}
/* USER CODE END 0 */



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  RCC->AHB2ENR |= 1 << 0; // Enable clock
  GPIOA->MODER |=  1 << 22;
  GPIOA->MODER &= ~(1 << (22 + 1));
  GPIOA->OTYPER &= ~(1 << 11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);
  ADXL355_Start_Sensor();

  // where we store the measurements
  uint32_t volatile ui32SensorX = 0;
  uint32_t volatile ui32SensorY = 0;
  uint32_t volatile ui32SensorZ = 0;

  while (1)
  {
    /* USER CODE END WHILE */
	//for(int i = 0; i < 500000; i++);
	// GPIOA->ODR ^= 1 << 4;
	// GPIOB->ODR ^= 1 << 4;
	// set7SegOutput(count);

	HAL_Delay(100);

    ui32SensorX = SPI_Read(XDATA3, SPI_READ_THREE_REG);
    ui32SensorY = SPI_Read(YDATA3, SPI_READ_THREE_REG);
    ui32SensorZ = SPI_Read(ZDATA3, SPI_READ_THREE_REG);

    uint32_t resultX = ADXL355_Acceleration_Data_Conversion(0);
    uint32_t resultY = ADXL355_Acceleration_Data_Conversion(0);
    uint32_t resultZ = ADXL355_Acceleration_Data_Conversion(0);
    printf("------------------\r\n");
    printf("X accel is: %i\r\n", (int)((float)resultX ));
    printf("Y accel is: %i\r\n", (int)((float)resultY ));
    printf("Z accel is: %i\r\n", (int)((float)resultZ ));
    printf("------------------");
    // set7SegOutput(count);
  }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART2 and Loop until the end
of transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}

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
