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
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Node_1의 CAN 수신 Identifier
 *
 * CAN Identifier는 수신 노드 ID로 사용한다.
 * 따라서 Main Node가 Node_1으로 보내는 메시지는
 * Standard ID 0x002를 사용한다.
 */
#define CAN_NODE1_ID              0x002U


/*
 * 차량 전진·후진·정지 상태 제어명령
 *
 * Main Node에서 수신하며,
 * Node_1에서는 초음파 Event 송신 방향을
 * 결정하는 조건값으로만 사용한다.
 */
#define CAN_CMD_DRIVE_STATE       60U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 전방 센서(HC01) 변수 (Echo: PA6 - TIM3_CH1)[cite: 13]
volatile uint32_t IC_Value1_Front = 0;
volatile uint8_t Is_First_Captured1 = 0;
volatile uint16_t Distance_Front = 0;

// 후방 센서(HC02) 변수 (Echo: PA7 - TIM3_CH2)[cite: 13]
volatile uint32_t IC_Value1_Back = 0;
volatile uint8_t Is_First_Captured2 = 0;
volatile uint16_t Distance_Back = 0;

// freertos.c에 선언된 3채널 ADC 버퍼 참조[cite: 4, 10]
extern uint16_t adc_dma_buff[3];

/*
 * Main Node에서 수신한 현재 차량 주행 상태
 *
 * 초기값은 정지 상태로 설정한다.
 *
 * Main Node의 60번 명령을 아직 받지 않았거나
 * CAN 통신이 연결되지 않은 상태에서
 * 초음파 Event가 임의로 송신되는 것을 방지한다.
 *
 * 이 변수는:
 * - main.c의 CAN 수신 콜백에서 갱신
 * - freertos.c의 초음파 Task에서 참조
 *
 * 용도로 사용한다.
 */
volatile DriveState_t g_drive_state = DRIVE_STATE_STOP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // 1. CAN 필터 설정 및 수신(RX0) 인터럽트 활성화[cite: 5, 6]
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // RX0 지정[cite: 5, 6]
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_Start(&hcan); // CAN 동작 개시[cite: 5, 6]
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // RX0 인터럽트 켜기[cite: 5, 6]

  // 2. TIM3 Input Capture 인터럽트 시작 (PA6: CH1-전방 / PA7: CH2-후방)[cite: 13]
  extern TIM_HandleTypeDef htim3;
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  HAL_ADCEx_Calibration_Start(&hadc1);   // ADC 보정

  // 3. ADC1 3채널 DMA 수집 시작 (PA0, PA1, PA4)[cite: 4, 10]
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buff, 3);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 * CAN 수신 콜백 함수
 *
 * Main Node에서 Node_1으로 전송한
 * 주행 상태 제어명령 60번을 수신한다.
 *
 * 이 콜백에서는:
 * - 수신 메시지 확인
 * - 주행 상태 저장
 *
 * 까지만 수행한다.
 *
 * 초음파 판단, CAN 송신 큐 등록 및 UART 출력은
 * 이 인터럽트 콜백에서 수행하지 않는다.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8] = {0, };


  /*
   * CAN1에서 발생한 수신 인터럽트인지 확인
   */
  if (hcan_ptr->Instance != CAN1)
  {
    return;
  }


  /*
   * FIFO0에서 수신 메시지 읽기
   *
   * 메시지를 정상적으로 읽지 못한 경우에는
   * 기존 주행 상태를 그대로 유지한다.
   */
  if (HAL_CAN_GetRxMessage(hcan_ptr,
                           CAN_RX_FIFO0,
                           &rxHeader,
                           rxData) != HAL_OK)
  {
    return;
  }


  /*
   * Standard ID Data Frame인지 확인
   *
   * 현재 통신 구조에서는:
   * - Standard 11-bit Identifier
   * - Data Frame
   *
   * 만 사용한다.
   */
  if ((rxHeader.IDE != CAN_ID_STD) ||
      (rxHeader.RTR != CAN_RTR_DATA))
  {
    return;
  }


  /*
   * 수신 노드 ID 확인
   *
   * Identifier는 수신 노드 ID이므로
   * Node_1 대상인 0x002 메시지만 처리한다.
   */
  if (rxHeader.StdId != CAN_NODE1_ID)
  {
    return;
  }


  /*
   * Command와 Value를 읽기 위해
   * 최소 2바이트 이상 수신되었는지 확인한다.
   *
   * Data[0] = Command
   * Data[1] = Value
   */
  if (rxHeader.DLC < 2U)
  {
    return;
  }


  /*
   * 제어명령 60번인지 확인
   *
   * 다른 Command는 이번 수신 로직에서 처리하지 않는다.
   */
  if (rxData[0] != CAN_CMD_DRIVE_STATE)
  {
    return;
  }


  /*
  * Data[1]의 Value를 현재 주행 상태로 저장
  *
  * 제어명령 60번의 ICD 정의:
  * 0 = 전진
  * 1 = 후진
  * 2 = 정지
  *
  * 이 상태값은 Node_1이 다시 송신하지 않는다.
  * 전방 또는 후방 초음파 Event의 송신 여부를
  * 결정하는 조건값으로만 사용한다.
  */
  switch (rxData[1])
  {
    /*
     * 전진 상태
     */
    case DRIVE_STATE_FORWARD:
        g_drive_state = DRIVE_STATE_FORWARD;
        break;


    /*
     * 후진 상태
     */
    case DRIVE_STATE_BACKWARD:
        g_drive_state = DRIVE_STATE_BACKWARD;
        break;


    /*
     * 정지 상태
     */
    case DRIVE_STATE_STOP:
        g_drive_state = DRIVE_STATE_STOP;
        break;


    default:

      /*
       * 정의되지 않은 Value가 들어온 경우
       * 잘못된 상태로 변경하지 않고
       * 기존 주행 상태를 그대로 유지한다.
       */
      break;
  }
}

// ---------------- [Input Capture 인터럽트 콜백] ----------------[cite: 13]
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // --- [전방 센서: TIM3_CH1 (PA6)] ---[cite: 13]
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Is_First_Captured1 == 0) // Rising Edge (Echo 시작)[cite: 13]
      {
        __HAL_TIM_SET_COUNTER(htim, 0); // 카운터 0 초기화[cite: 13]
        Is_First_Captured1 = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else if (Is_First_Captured1 == 1) // Falling Edge (Echo 끝)[cite: 13]
      {
        IC_Value1_Front = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        Distance_Front = IC_Value1_Front / 58;

        Is_First_Captured1 = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      }
    }

    // --- [후방 센서: TIM3_CH2 (PA7)] ---[cite: 13]
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      if (Is_First_Captured2 == 0) // Rising Edge[cite: 13]
      {
        __HAL_TIM_SET_COUNTER(htim, 0); // 카운터 0 초기화[cite: 13]
        Is_First_Captured2 = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else if (Is_First_Captured2 == 1) // Falling Edge[cite: 13]
      {
        IC_Value1_Back = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        Distance_Back = IC_Value1_Back / 58;

        Is_First_Captured2 = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      }
    }
  }
}

// Microsecond Delay 함수 (72MHz 소프트웨어 딜레이)
void delay_us(uint16_t us)
{
  uint32_t count = us * 8;
  while(count--) {
    __NOP();
  }
}

// printf UART2 재정의[cite: 13]
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
