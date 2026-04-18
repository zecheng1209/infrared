/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "infrared.h"
#include "can.h"
#include <string.h>
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
// 外部声明CAN接收变量
extern volatile uint8_t can_rx_buffer[8];
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t can_rx_id;

// 红外发送状态（用于非阻塞状态机）
#define IR_TX_IDLE       0
#define IR_TX_WAIT_ACK   1
volatile uint8_t ir_tx_state = IR_TX_IDLE;
volatile uint8_t tx_count = 0;  // 发送计数器

// CAN→红外发送缓冲区
uint8_t can_to_ir_buffer[8];
uint8_t can_to_ir_ready = 0;  // 数据就绪标志
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  IR_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IR_CheckRxTimeout();

    // ========== CAN → 红外（非阻塞状态机 + 自发自收过滤） ==========
    // 第1步：检查CAN数据并缓存（不阻塞，立即返回）
    if (ir_tx_state == IR_TX_IDLE && can_rx_flag && !IR_IsTXBusy())
    {
      can_rx_flag = 0;

      // 检查CAN ID是否匹配本模块
      if (can_rx_id == IR_MODULE_ID)
      {
        memcpy(can_to_ir_buffer, (uint8_t*)can_rx_buffer, 8);
        can_to_ir_ready = 1;
      }
      // ID不匹配则忽略该CAN帧
    }

    // 第2步：发送缓存的CAN数据（非阻塞）
    // 注意：IR_SendData()会自动设置ir_rx_ignore_self标志
    //       发送完成后80ms内会自动忽略接收到的数据（防自收）
    if (can_to_ir_ready && !IR_IsTXBusy())
    {
      if (IR_SendData(can_to_ir_buffer, 8))
      {
        tx_count++;  // 发送成功计数
        can_to_ir_ready = 0;  // 清除就绪标志

        // 调试信息：可以观察发送和忽略状态
        // printf("TX: count=%d, ignore=%d\r\n", tx_count, ir_rx_ignore_self);
      }
    }

    // ========== 红外 → CAN（非阻塞处理 + 自动防自收） ==========
    // ir_rx_complete_flag只在冷却期结束后才会被设置（在infrared.c中已处理）
    // 所以这里收到的数据一定是其他模块发的，不是自己发的！
    if (ir_rx_complete_flag)
    {
      ir_rx_complete_flag = 0;

      // 验证CRC（第9字节是CRC）
      uint8_t calculated_crc = IR_CRC8(received_data, 8);
      if (calculated_crc == received_data[8])
      {
        // CRC校验通过，发送ACK（使用非阻塞方式，也会触发80ms冷却期）
        IR_SendDataAck_NonBlocking(1);

        // 发送到CAN总线
        // CAN ID = 源模块ID，数据帧 = 纯数据（不含CRC）
        CAN_SendData(received_data, 8, IR_MODULE_ID);
      }
      else
      {
        // CRC错误，发送NACK
        IR_SendDataAck_NonBlocking(2);
      }
    }
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
#ifdef USE_FULL_ASSERT
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
