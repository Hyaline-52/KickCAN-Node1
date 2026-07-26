/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications (STM32F103 CMSIS_V1 호환)
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"   // hadc1 핸들 사용
#include "can.h"   // hcan 핸들 사용
#include "usart.h" // huart2 핸들 사용
#include <stdio.h> // printf 출력을 위한 헤더
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 단일 센서 데이터 관리 구조체
typedef struct {
    uint16_t raw;       // DMA로 넘어온 순수 ADC 값 (0 ~ 4095)
    uint16_t filtered;  // LPF 적용된 안정적인 ADC 값
    int16_t  delta;     // 변화량 (이전 필터값 - 현재 필터값)
} LightSensor_t;

// 전체 오토라이트 & TP 시스템 구조체 (3개 채널)
typedef struct {
    LightSensor_t forward;  // LS1 (PA0)
    LightSensor_t overhead; // LS2 (PA1)
    LightSensor_t tp;       // TP  (PA4)
} AutoLight_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// main.c에 선언된 거리 변수 및 us delay 함수 참조
extern volatile uint16_t Distance_Front;
extern volatile uint16_t Distance_Back;
extern void delay_us(uint16_t us);

// PA0(LS1), PA1(LS2), PA4(TP) 센서 값이 저장될 DMA 버퍼 (3채널 대응)
uint16_t adc_dma_buff[3] = {0, };

AutoLight_t g_auto_light;

/* USER CODE END Variables */
osThreadId TaskTPHandle;
osThreadId TaskHC1Handle;
osThreadId TaskHC2Handle;
osThreadId TaskLDHandle;
osThreadId TaskCANHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskTP(void const * argument);
void StartTaskHC1(void const * argument);
void StartTaskHC2(void const * argument);
void StartTaskLD(void const * argument);
void StartTaskCAN(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskTP */
  osThreadDef(TaskTP, StartTaskTP, osPriorityNormal, 0, 128);
  TaskTPHandle = osThreadCreate(osThread(TaskTP), NULL);

  /* definition and creation of TaskHC1 */
  osThreadDef(TaskHC1, StartTaskHC1, osPriorityNormal, 0, 128);
  TaskHC1Handle = osThreadCreate(osThread(TaskHC1), NULL);

  /* definition and creation of TaskHC2 */
  osThreadDef(TaskHC2, StartTaskHC2, osPriorityNormal, 0, 128);
  TaskHC2Handle = osThreadCreate(osThread(TaskHC2), NULL);

  /* definition and creation of TaskLD */
  osThreadDef(TaskLD, StartTaskLD, osPriorityNormal, 0, 128);
  TaskLDHandle = osThreadCreate(osThread(TaskLD), NULL);

  /* definition and creation of TaskCAN */
  osThreadDef(TaskCAN, StartTaskCAN, osPriorityAboveNormal, 0, 128);
  TaskCANHandle = osThreadCreate(osThread(TaskCAN), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTaskTP */
/**
* @brief Function implementing the TaskTP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTP */
void StartTaskTP(void const * argument)
{
  /* USER CODE BEGIN StartTaskTP */
  for(;;)
  {
    // PA4 (TMP36 온도센서) 수집 처리
    uint16_t raw_tp = adc_dma_buff[2];
    g_auto_light.tp.raw = raw_tp;
    g_auto_light.tp.filtered = (uint16_t)(0.2f * raw_tp + 0.8f * g_auto_light.tp.filtered);

    osDelay(100);
  }
  /* USER CODE END StartTaskTP */
}

/* USER CODE BEGIN Header_StartTaskHC1 */
/**
* @brief Function implementing the TaskHC1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHC1 */
void StartTaskHC1(void const * argument)
{
  /* USER CODE BEGIN StartTaskHC1 */
  for(;;)
  {
    // PC0: HC1 (전방) Trig 10us 펄스 발사
    HAL_GPIO_WritePin(GPIOC, PC0_A5_HCTr1_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOC, PC0_A5_HCTr1_Pin, GPIO_PIN_RESET);

    // Echo 신호 수신 대기 및 측정 주기 관리
    osDelay(230);
  }
  /* USER CODE END StartTaskHC1 */
}

/* USER CODE BEGIN Header_StartTaskHC2 */
/**
* @brief Function implementing the TaskHC2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHC2 */
void StartTaskHC2(void const * argument)
{
  /* USER CODE BEGIN StartTaskHC2 */
  for(;;)
  {
    // PC1: HC2 (후방) Trig 10us 펄스 발사
    HAL_GPIO_WritePin(GPIOC, PC1_A4_HCTr2_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOC, PC1_A4_HCTr2_Pin, GPIO_PIN_RESET);

    // Echo 신호 수신 대기 및 측정 주기 관리
    osDelay(230);
  }
  /* USER CODE END StartTaskHC2 */
}

/* USER CODE BEGIN Header_StartTaskLD */
/**
* @brief Function implementing the TaskLD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLD */
void StartTaskLD(void const * argument)
{
  /* USER CODE BEGIN StartTaskLD */

  uint16_t prev_fw_filt = 0;
  uint16_t prev_oh_filt = 0;

  /* Infinite loop */
  for(;;)
  {
    // 1. DMA 버퍼에서 값 안전하게 읽어오기 (PA0, PA1)
    uint16_t raw_fw = adc_dma_buff[0]; // LS1 (PA0)
    uint16_t raw_oh = adc_dma_buff[1]; // LS2 (PA1)

    // 2. Low-Pass Filter (노이즈 제거, 가중치 0.2 : 0.8)
    g_auto_light.forward.filtered  = (uint16_t)(0.2f * raw_fw + 0.8f * g_auto_light.forward.filtered);
    g_auto_light.overhead.filtered = (uint16_t)(0.2f * raw_oh + 0.8f * g_auto_light.overhead.filtered);

    // 3. 변화량(Delta) 계산
    g_auto_light.forward.delta  = (int16_t)prev_fw_filt - g_auto_light.forward.filtered;
    g_auto_light.overhead.delta = (int16_t)prev_oh_filt - g_auto_light.overhead.filtered;

    // 다음 주기를 위해 이전 필터값 갱신
    prev_fw_filt = g_auto_light.forward.filtered;
    prev_oh_filt = g_auto_light.overhead.filtered;

    // Raw 값 구조체 업데이트
    g_auto_light.forward.raw  = raw_fw;
    g_auto_light.overhead.raw = raw_oh;

    osDelay(100);
  }
  /* USER CODE END StartTaskLD */
}

/* USER CODE BEGIN Header_StartTaskCAN */
/**
* @brief Function implementing the TaskCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCAN */
void StartTaskCAN(void const * argument)
{
  /* USER CODE BEGIN StartTaskCAN */
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8] = {0, };
  uint32_t txMailbox;

  txHeader.StdId = 0x101;            // 노드 ID 설정
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  for(;;)
  {
    // CAN 데이터 패킷 구성 (전방/후방 거리 데이터 반영)
    txData[0] = (uint8_t)(Distance_Front >> 8);
    txData[1] = (uint8_t)(Distance_Front & 0xFF);
    txData[2] = (uint8_t)(Distance_Back >> 8);
    txData[3] = (uint8_t)(Distance_Back & 0xFF);

    // CAN 메시지 전송
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);


    // ------------------------------------------------------------
    // TMP36 온도센서 변환
    //
    // TMP36 특성
    // 0℃   = 500mV
    // 25℃  = 750mV
    // 100℃ = 1500mV
    //
    // 출력 특성 : 10mV / ℃
    //
    // 계산식
    // Temperature = (Voltage[mV] - 500) / 10
    // ------------------------------------------------------------
    
    // TMP36의 필터링된 ADC 값 사용
    // adc_dma_buff[2]의 값을 StartTaskTP에서 LPF 처리한 결과
    // uint16_t tp_raw = g_auto_light.tp.filtered;
    uint16_t tp_raw = g_auto_light.tp.raw;

    // ADC 값을 mV 단위의 실수형 전압으로 변환
    // STM32F103 ADC 해상도 : 12bit, ADC 범위 : 0 ~ 4095
    // ADC 기준전압 : 3.3V = 3300mV
    float temp_mv = ((float)tp_raw * 3300.0f) / 4095.0f;

    // TMP36 온도 계산
    // TMP36 출력 특성:
    // 0℃에서 500mV
    // 온도가 1℃ 증가할 때마다 출력전압이 10mV 증가
    float temp_deg = (temp_mv - 500.0f) / 10.0f;

    // ---------------- [통합 디버그 출력] ----------------
    // 센서 전체 데이터를 한눈에 볼 수 있도록 CAN Task로 일괄 이전

    // printf("[NODE1] DIST F:%3dcm B:%3dcm | LS1:%4d LS2:%4d | TP_RAW:%d | TEMP:%d degC\r\n",
    //        Distance_Front,
    //        Distance_Back,
    //        g_auto_light.forward.filtered,
    //        g_auto_light.overhead.filtered,
    //        (int)tp_raw,
    //        (int)temp_deg);



    printf("ADC0=%4d ADC1=%4d ADC4=%4d | TP_RAW=%4d | TEMP=%.1f degC\r\n",
       adc_dma_buff[0],
       adc_dma_buff[1],
       adc_dma_buff[2],
       tp_raw,
       temp_deg);

    osDelay(200);
  }
  /* USER CODE END StartTaskCAN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

