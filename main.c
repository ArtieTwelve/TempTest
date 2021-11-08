/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Simple HTS221 Temperature reading. Code is specific to the L475VG board, 
  * which uses the I2C2 bus for connecting all it's sensors.
  *
  * Setup: I used STM32 CubeMX to generate the IAR project files.
  * 	   Use these pins for USART1 and I2C2, the default USART pins CubeMX 
  *        sets up did not work for me.
  *        
  * 	   PB6 - USART1_RX (moved from PA10)
  * 	   PB7 - USART1_TX (moved from PA9)
  *
  *        The default I2C2 pins work
  * 	   PB10 - I2C2_SCL
  * 	   PB11 - I2C2_SDA
  *
  * Tested in IAR and STM32Ide
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // for sprintf, prinf
#include <string.h> // for memset
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Grabbed all the defines from the Board Support package in case I want to extend
// this and also get the humidity

#define HTS221_H0_RH_X2        (uint8_t)0x30
#define HTS221_H1_RH_X2        (uint8_t)0x31
#define HTS221_T0_DEGC_X8      (uint8_t)0x32
#define HTS221_T1_DEGC_X8      (uint8_t)0x33
#define HTS221_T0_T1_DEGC_H2   (uint8_t)0x35
#define HTS221_H0_T0_OUT_L     (uint8_t)0x36
#define HTS221_H0_T0_OUT_H     (uint8_t)0x37
#define HTS221_H1_T0_OUT_L     (uint8_t)0x3A
#define HTS221_H1_T0_OUT_H     (uint8_t)0x3B
#define HTS221_T0_OUT_L        (uint8_t)0x3C
#define HTS221_T0_OUT_H        (uint8_t)0x3D
#define HTS221_T1_OUT_L        (uint8_t)0x3E
#define HTS221_T1_OUT_H        (uint8_t)0x3F

#define HTS221_WHO_AM_I		   (uint8_t)0x0F
#define HTS221_ADDR (uint8_t)0xBE

#define HTS221_HR_OUT_H_REG        (uint8_t)0x29

/**
  * @brief  Temperature data (LSB).
  *         Read
  *         Default value: 0x00.
  *         TOUT7 - TOUT0: temperature data LSB.
  */
#define HTS221_TEMP_OUT_L_REG         (uint8_t)0x2A

/**
  * @brief  Temperature data (MSB).
  *         Read
  *         Default value: 0x00.
  *         TOUT15 - TOUT8: temperature data MSB.
  */
#define HTS221_TEMP_OUT_H_REG         (uint8_t)0x2B

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

HAL_StatusTypeDef ret;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
 //                                         uint32_t Timeout)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



// Adding in the override to __io_putchar so SWO can be used for debugging

int __io_putchar(int ch)
{
 ITM_SendChar(ch);
 return(ch);
}




// A good article on HAL_I2C methods can be found at:
// https://forum.digikey.com/t/using-the-stm32cube-hal-i2c-driver-in-master-mode/15122


// working multi-byte read using HAL_I2C_Mem_Read
void readSensorMulti(I2C_HandleTypeDef *hi2c,uint8_t DevAddress,uint16_t  MemAddress,uint8_t *pData,uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	// Check Status and print failure message to SWO trace
	status = HAL_I2C_Mem_Read(hi2c,DevAddress,MemAddress,I2C_MEMADD_SIZE_8BIT, pData,Size,50);
    
    if(status != HAL_OK) {
        // Do something...
        printf("Call to readSensorMulti failed. HAL_STATUS: %d\n",status);
    }
}

// working single byte read using HAL_I2C_Mem_Read
uint8_t readSensor(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress) {

	  HAL_StatusTypeDef status = HAL_OK;
	  uint8_t read_value = 0;
	  // Check Status and print failure message to SWO trace
	  status = HAL_I2C_Mem_Read(hi2c,DevAddress,MemAddress,I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value,1,50);
      
      if(status != HAL_OK) {
        // Do something...
        printf("Call to readSensor failed. HAL_STATUS: %d\n",status);
      }
	  return read_value;
}

// Most of this calculation code is taken directly from HTS221.c in the Board Support Package
float HTS221_T_ReadTemp(I2C_HandleTypeDef *hi2c,uint16_t DeviceAddr)
{
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  uint8_t buffer[4], tmp;
  float tmp_f;

  // In order to get the correct values from the HTS221 regisers, I had 
  // to OR the addresses with 0x80. I don't know why this has to be done
  // when doing a mulit byte read. Perhaps it's something specific to 
  // the HAL_I2C_Mem_Read function. I don't need to do it with a single
  // byte read
  
  readSensorMulti(hi2c,HTS221_ADDR,(HTS221_T0_DEGC_X8 | 0x80), buffer, 2);

  tmp = readSensor(hi2c, HTS221_ADDR,HTS221_T0_T1_DEGC_H2);

  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16 >> 3;
  T1_degC = T1_degC_x8_u16 >> 3;


  readSensorMulti(hi2c, HTS221_ADDR,(HTS221_T0_OUT_L | 0x80), buffer, 4);

  T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];

  // Get the T_OUT_Low & T_OUT_High with a multi byte call
  readSensorMulti(hi2c, HTS221_ADDR,(HTS221_TEMP_OUT_L_REG | 0x80), buffer, 2);

  // repack T_OUT_Low & T_OUT_High into a a single 16 bit value
  T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  // calculate the temp
  tmp_f = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;

  return tmp_f;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 
  uint8_t uartBuffer[50] = {0};
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	// I could have just called HAL_I2C_Mem_Read in the main while loop, but I wanted them wrapped
	// in methods so I can replace the HAL calls with register flipping code in the future.

	float temp = HTS221_T_ReadTemp(&hi2c2,HTS221_ADDR);

	int len = sprintf(uartBuffer,"Temperature: %2.2f C.\n",temp);
    
    // print to the COM port
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, len, 50);
    
    
    // prints to the Terminal IO window in IAR or SWO Debug console in STM32IDE
    printf("Temperature: %2.2f C.\n",temp);

	HAL_Delay(2000);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
