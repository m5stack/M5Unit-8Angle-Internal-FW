/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "ws2812.h"
#include "flash.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define abs(x) (((x) > 0) ? (x) : -(x))
#define OFFSET	0.707106

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ADC DMA data in buffer
__IO uint32_t uiAdcValueBuf[160];

// Ave data
//__IO uint64_t adcTotal[8]={0};
__IO uint16_t usAdcValue16[8];
__IO uint8_t usAdcValue8[8];
__IO uint16_t CmpltAdcValue16[8];
__IO uint8_t CmpltAdcValue8[8];
uint8_t dummyValue[8];
// Press state
uint8_t ucPressState = 0;
uint16_t max_Value = 4025;

void user_i2c_init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = i2c_address[0]<<1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) 
{
	if(len == 1 && (rx_data[0] >= FROM_16BIT_REG & rx_data[0] <= TO_16BIT_REG)) 
	{
		i2c1_set_send_data((uint8_t *)&CmpltAdcValue16[(rx_data[0]/2)], 2);
	} 
	else if(len == 1 && (rx_data[0] >= FROM_8BIT_REG & rx_data[0] <= TO_8BIT_REG)) 
	{
		i2c1_set_send_data((uint8_t *)&CmpltAdcValue8[(rx_data[0] - 0x10)], 1);
  } 
	else if(len == 1 && (rx_data[0] == SWITCH_REG)) 
	{
		i2c1_set_send_data((uint8_t *)&ucPressState, 1);
  } 
	else if (len > 1 && (rx_data[0] >= FROM_RGB_REG & rx_data[0] <= TO_RGB_REG)) 
	{
		if(len == 2)
		{
			neopixel_set_single_color(((rx_data[0] - FROM_RGB_REG)/4),(rx_data[1]),((rx_data[0] - FROM_RGB_REG)%4));
			neopixel_show();
		}
		else
		{
			neopixel_set_color(((rx_data[0] - FROM_RGB_REG)/4), ((rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8) | rx_data[4]));
			neopixel_show();
		}
  }
	else if (len > 1 && (rx_data[0] == 0xFF)) 
	{
    if (len == 2) {
      if (rx_data[1] > 0 & rx_data[1] < 128) {
        i2c_address[0] = rx_data[1];
        i2c_address_write_to_flash();
        user_i2c_init();
      }
    }
  }  
	else if (len == 1 && (rx_data[0] == 0xFF)) 
	{
    if (i2c_address != 0)
      i2c1_set_send_data(i2c_address, 1);
  }  
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }     
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[8]={0};
	HAL_ADC_Stop_DMA(hadc);
	for (uint8_t i = 0; i < 160; i++) {
    adcTotal[i%8] += uiAdcValueBuf[i];
  }
	for (uint8_t i = 0; i < 8; i++) {
    usAdcValue16[i] = (adcTotal[i%8] / 20);
		if(usAdcValue16[i] > max_Value)
			max_Value = usAdcValue16[i];
		usAdcValue16[i] = map(usAdcValue16[i],0,max_Value,0,4095);
		usAdcValue8[i] = map(usAdcValue16[i],0,4095,0,255);
  }
	for (uint8_t i = 0; i < 8; i++) {
    CmpltAdcValue16[i] = usAdcValue16[i];
		CmpltAdcValue8[i] = usAdcValue8[i];
  }
	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, 160);
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
  MX_DMA_Init();
  MX_ADC_Init();
//  MX_I2C1_Init();
	i2c_address_read_from_flash();
	user_i2c_init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, 160);
  HAL_I2C_EnableListen_IT(&hi2c1);
	sk6812_init(TOTAL_RGB);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(5);
		ucPressState = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
