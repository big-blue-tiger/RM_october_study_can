/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//
// Created by 28672 on 24-10-12.
//




#include "stm32f4xx_hal_can.h"

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

/* USER CODE BEGIN PV */
float linearMapping(int in_,int in_min,int in_max,float out_min,float out_max){
  float k = (out_max-out_min)/(float) (in_max - in_min);
  return out_min + (float) (in_ - in_min)*k;
}
struct M3508_Motor_v5 {
  float ratio_;           // 电机减�?�比
  float angle_fractional; // deg 输出端累计转动角度的小数部分
  float angle_integer;	// 角度的整数部�????
  float delta_angle_;     // deg 输出端新转动的角�????
  float ecd_angle_;       // deg 当前电机编码器角�????
  float last_ecd_angle_;  // deg 上次电机编码器角�????
  float delta_ecd_angle_; // deg 编码器端新转动的角度
  float rotate_speed_;    // dps 反馈转子转�??
  float current_;         // A   反馈转矩电流
  float temp_;            // °C  反馈电机温度
};
struct M3508_Motor_v5 motor = {3591.0/187,0,0,0,0,0,0,0,0,0};
void canRxMsgCallback_v5(const uint8_t rx_data[8]){
  motor.last_ecd_angle_ = motor.ecd_angle_;
  motor.ecd_angle_ = linearMapping(rx_data[0]<< 8 | rx_data[1],0,8191,0,360);
  motor.delta_ecd_angle_ = motor.ecd_angle_  - motor.last_ecd_angle_;
  motor.rotate_speed_ = rx_data[2]<< 8 | rx_data[3];
  motor.current_ = rx_data[4]<< 8 | rx_data[5];
  motor.temp_ = rx_data[6];
  motor.delta_angle_ = motor.delta_angle_ + motor.delta_ecd_angle_;

  motor.angle_fractional = motor.angle_fractional + motor.delta_ecd_angle_;
  motor.angle_integer = motor.angle_integer +  (int) motor.angle_fractional;
  motor.angle_fractional = motor.angle_fractional - (int) motor.angle_fractional;
}

CAN_TxHeaderTypeDef TxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0; // 选择过滤器编�?????????
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 使用掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过�r���?????
  sFilterConfig.FilterIdHigh = 0x0000; // 设置滤波器ID高位（使�?????????0x0000为例�?????????
  sFilterConfig.FilterIdLow = 0x0000; // 设置滤波器ID低位
  sFilterConfig.FilterMaskIdHigh = 0x0000; // 设置掩码ID高位（使�?????????0x0000，这意味�?????????匹配�?????????有）
  sFilterConfig.FilterMaskIdLow = 0x0000; // 设置掩码ID低位
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 将过滤器分配�????????? FIFO0
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE; // 启用过滤�?????????
  HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig);
  HAL_CAN_Start(&hcan1);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t TxMessage[8] = {0,0x90,0,0,0,0,0,0};
  while (1)
  {
		for(int i = 0;i<8;i++)
			TxMessage[i] += i;
    TxHeader.ExtId = 0;
    TxHeader.StdId = 0x200; // 标准 ID
    TxHeader.IDE = CAN_ID_STD; // 标准标识�????
    TxHeader.RTR = CAN_RTR_DATA; // 数据�????
    TxHeader.DLC = sizeof(TxMessage); // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxMessage, &TxMailbox)!=HAL_OK)
    {
      HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
    }
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);

    HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
