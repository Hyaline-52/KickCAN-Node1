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

/*
 * Node_1에서 발생할 수 있는 CAN 제어 명령
 *
 * 각 센서 Task는 센서값을 판단한 뒤
 * 해당하는 명령 번호를 생성한다.
 *
 * 실제 CAN 전송은 이후 TaskCAN에서 처리할 예정이다.
 */
typedef enum
{
    CAN_CMD_NONE = 0,

    CAN_CMD_TEMP_WARNING       = 110,
    CAN_CMD_TEMP_NORMAL        = 111,

    CAN_CMD_LIGHT_FAST_ON      = 112,
    CAN_CMD_LIGHT_SMOOTH_ON    = 113,
    CAN_CMD_LIGHT_OFF          = 114,

    CAN_CMD_FRONT_WARNING_30CM = 115,
    CAN_CMD_FRONT_WARNING_20CM = 116,
    CAN_CMD_FRONT_WARNING_10CM = 117,

    CAN_CMD_REAR_WARNING_30CM  = 118,
    CAN_CMD_REAR_WARNING_20CM  = 119,
    CAN_CMD_REAR_WARNING_10CM  = 120,

    CAN_CMD_FRONT_CLEAR        = 121,
    CAN_CMD_REAR_CLEAR         = 122

} Node1CanCommand_t;


/*
 * 현재 조도 수준
 */
typedef enum
{
    LIGHT_LEVEL_DARK = 0,       // ADC 1000 이하
    LIGHT_LEVEL_TRANSITION,     // ADC 1001 ~ 2899
    LIGHT_LEVEL_BRIGHT          // ADC 2900 이상

} LightLevel_t;


/*
 * 현재 라이트 동작 상태
 */
typedef enum
{
    LIGHT_STATE_OFF = 0,        // 라이트 해제 상태
    LIGHT_STATE_FAST_ON,        // 터널 진입용 빠른 점등
    LIGHT_STATE_SMOOTH_ON       // 일몰/야간용 자연스러운 점등

} LightState_t;


/*
 * 초음파 거리 경고 상태
 */
typedef enum
{
    ULTRASONIC_STATE_CLEAR = 0,   // 장애물 없음: 30cm 초과
    ULTRASONIC_STATE_30CM,        // 21 ~ 30cm
    ULTRASONIC_STATE_20CM,        // 11 ~ 20cm
    ULTRASONIC_STATE_10CM         // 1 ~ 10cm

} UltrasonicState_t;


// 단일 센서 데이터 관리 구조체
typedef struct
{
    uint16_t raw;       // DMA로 넘어온 순수 ADC 값 (0 ~ 4095)
    uint16_t filtered;  // LPF 적용된 안정적인 ADC 값
    int16_t  delta;     // 이전 필터값 - 현재 필터값

} LightSensor_t;


// 전체 오토라이트 & TP 시스템 구조체
typedef struct
{
    LightSensor_t forward;  // LS1 (PA0)
    LightSensor_t overhead; // LS2 (PA1)
    LightSensor_t tp;       // TP  (PA4)

} AutoLight_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*
 * 조도센서 ADC 기준값
 *
 * 실제 측정 결과:
 * 밝은 환경: 약 2900 이상
 * 어두운 환경: 약 1000 이하
 */
#define LIGHT_BRIGHT_THRESHOLD           2500U
#define LIGHT_DARK_THRESHOLD             1000U


/*
 * 조도 변화량 기준
 *
 * raw delta:
 * 이전 raw 값 - 현재 raw 값
 *
 * 양수: ADC값 감소, 즉 어두워짐
 * 음수: ADC값 증가, 즉 밝아짐
 */
#define LIGHT_RAW_RAPID_DROP             1200


/*
 * 필터값 변화량 기준
 *
 * 현재 LPF가 0.2 : 0.8이므로
 * 실제 raw 변화보다 filtered 변화가 작게 나타난다.
 */
#define LIGHT_FILTERED_DROP               250
#define LIGHT_FILTERED_RISE               250


/*
 * 일시적 그림자 확인 횟수
 *
 * TaskLD 주기 100ms 기준:
 * 3회 = 약 300ms
 */
#define SHADOW_CONFIRM_COUNT                3U


/*
 * 밝음 복귀 확인 횟수
 *
 * 라이트가 순간적인 빛에 바로 꺼지지 않도록
 * 5회 연속 밝음 확인 후 해제한다.
 *
 * TaskLD 주기 100ms 기준:
 * 5회 = 약 500ms
 */
#define LIGHT_OFF_CONFIRM_COUNT              5U


/*
 * 초음파 거리 경고 기준값
 */
#define ULTRASONIC_WARNING_10CM       10U
#define ULTRASONIC_WARNING_20CM       20U
#define ULTRASONIC_WARNING_30CM       30U

/*
 * HC-SR04 측정값 유효 범위
 *
 * Distance 값이 0이면 아직 Echo가 측정되지 않았거나
 * 측정에 실패한 것으로 판단한다.
 */
#define ULTRASONIC_MIN_VALID_CM         1U
#define ULTRASONIC_MAX_VALID_CM       400U



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


// PA0(LS1), PA1(LS2), PA4(TP) 센서 값이 저장될 DMA 버퍼
uint16_t adc_dma_buff[3] = {0, };


// 전체 센서값 관리 구조체
AutoLight_t g_auto_light;


/*
 * TaskLD에서 판단한 현재 라이트 상태
 */
volatile LightState_t g_light_state = LIGHT_STATE_OFF;


/*
 * TaskLD에서 생성한 CAN 명령
 *
 * CAN_CMD_NONE:
 * 현재 새롭게 전송할 명령이 없음
 *
 * 이후 TaskCAN에서 이 값을 읽어 CAN 메시지로 전송할 예정이다.
 */
volatile Node1CanCommand_t g_light_can_command = CAN_CMD_NONE;

/*
 * 전방·후방 초음파 현재 상태
 */
volatile UltrasonicState_t g_front_ultrasonic_state =
    ULTRASONIC_STATE_CLEAR;

volatile UltrasonicState_t g_rear_ultrasonic_state =
    ULTRASONIC_STATE_CLEAR;


/*
 * 초음파 Task에서 생성한 CAN 명령
 *
 * TaskHC1:
 * 전방 명령 115, 116, 117, 121
 *
 * TaskHC2:
 * 후방 명령 118, 119, 120, 122
 */
volatile Node1CanCommand_t g_front_can_command = CAN_CMD_NONE;
volatile Node1CanCommand_t g_rear_can_command  = CAN_CMD_NONE;

/* USER CODE END Variables */
osThreadId TaskTPHandle;
osThreadId TaskHC1Handle;
osThreadId TaskHC2Handle;
osThreadId TaskLDHandle;
osThreadId TaskCANHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static LightLevel_t GetLightLevel(uint16_t adc_value);

static UltrasonicState_t GetUltrasonicState(uint16_t distance_cm);

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
  osThreadDef(TaskCAN, StartTaskCAN, osPriorityAboveNormal, 0, 512);
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

  
  /*
   * 전방 초음파의 현재 판단 상태와 이전 상태
   *
   * 상태가 실제로 바뀌었을 때만
   * 새로운 CAN 명령을 생성하기 위해 사용한다.
   */
  UltrasonicState_t current_front_state =
      ULTRASONIC_STATE_CLEAR;

  UltrasonicState_t previous_front_state =
      ULTRASONIC_STATE_CLEAR;


  /*
   * 최초 유효 거리 측정 여부
   *
   * 시스템 시작 직후 Distance_Front가 0인 상태에서
   * 장애물 경고가 발생하지 않도록 사용한다.
   */
  uint8_t first_valid_sample = 1;


  for (;;)
  {
    /*
     * 1. PC0: 전방 초음파 Trig 신호 발생
     */
    HAL_GPIO_WritePin(GPIOC,
                      PC0_A5_HCTr1_Pin,
                      GPIO_PIN_RESET);

    delay_us(2);

    HAL_GPIO_WritePin(GPIOC,
                      PC0_A5_HCTr1_Pin,
                      GPIO_PIN_SET);

    delay_us(10);

    HAL_GPIO_WritePin(GPIOC,
                      PC0_A5_HCTr1_Pin,
                      GPIO_PIN_RESET);


    /*
     * 2. Echo 인터럽트에서 계산된 전방 거리값 읽기
     */
    uint16_t front_distance = Distance_Front;


    /*
     * 3. 유효하지 않은 측정값 처리
     *
     * 0cm:
     * 아직 측정되지 않았거나 Echo 측정 실패
     *
     * 400cm 초과:
     * HC-SR04의 일반적인 유효 측정 범위를 벗어난 값
     *
     * 유효하지 않은 경우 기존 상태를 유지한다.
     */
    if ((front_distance < ULTRASONIC_MIN_VALID_CM) ||
        (front_distance > ULTRASONIC_MAX_VALID_CM))
    {
      osDelay(120);
      continue;
    }


    /*
     * 4. 최초 유효 거리값으로 초기 상태 설정
     */
    if (first_valid_sample == 1)
    {
      current_front_state =
          GetUltrasonicState(front_distance);

      previous_front_state =
          current_front_state;

      g_front_ultrasonic_state =
          current_front_state;


      /*
       * 시작 시 이미 장애물이 가까이 있는 경우에도
       * 현재 거리 단계에 해당하는 명령을 생성한다.
       */
      switch (current_front_state)
      {
        case ULTRASONIC_STATE_10CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_10CM;
          break;

        case ULTRASONIC_STATE_20CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_20CM;
          break;

        case ULTRASONIC_STATE_30CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_30CM;
          break;

        case ULTRASONIC_STATE_CLEAR:
        default:
          g_front_can_command =
              CAN_CMD_FRONT_CLEAR;
          break;
      }

      first_valid_sample = 0;

      osDelay(120);
      continue;
    }


    /*
     * 5. 현재 거리값을 경고 상태로 변환
     */
    current_front_state =
        GetUltrasonicState(front_distance);


    /*
     * 6. 거리 상태가 바뀌었을 때만
     *    전방 CAN 명령 생성
     */
    if (current_front_state != previous_front_state)
    {
      switch (current_front_state)
      {
        case ULTRASONIC_STATE_10CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_10CM;
          break;

        case ULTRASONIC_STATE_20CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_20CM;
          break;

        case ULTRASONIC_STATE_30CM:
          g_front_can_command =
              CAN_CMD_FRONT_WARNING_30CM;
          break;

        case ULTRASONIC_STATE_CLEAR:
        default:
          g_front_can_command =
              CAN_CMD_FRONT_CLEAR;
          break;
      }


      /*
       * 외부 확인용 전방 상태 저장
       */
      g_front_ultrasonic_state =
          current_front_state;


      /*
       * 다음 상태 비교를 위해 현재 상태 저장
       */
      previous_front_state =
          current_front_state;
    }


    /*
     * HC1 측정 주기
     *
     * HC2와 초음파 신호가 겹치는 것을 줄이기 위해
     * 기존 230ms보다 짧은 120ms 주기를 사용하고,
     * HC2는 시작 시 60ms 지연시킨다.
     */
    osDelay(120);
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

  
  UltrasonicState_t current_rear_state =
      ULTRASONIC_STATE_CLEAR;

  UltrasonicState_t previous_rear_state =
      ULTRASONIC_STATE_CLEAR;

  uint8_t first_valid_sample = 1;


  /*
   * HC1과 HC2가 동시에 Trig를 발생시키면
   * 서로의 초음파 Echo를 잘못 수신할 수 있다.
   *
   * 따라서 HC2는 HC1보다 60ms 늦게 시작한다.
   */
  osDelay(60);


  for (;;)
  {
    /*
     * 1. PC1: 후방 초음파 Trig 신호 발생
     */
    HAL_GPIO_WritePin(GPIOC,
                      PC1_A4_HCTr2_Pin,
                      GPIO_PIN_RESET);

    delay_us(2);

    HAL_GPIO_WritePin(GPIOC,
                      PC1_A4_HCTr2_Pin,
                      GPIO_PIN_SET);

    delay_us(10);

    HAL_GPIO_WritePin(GPIOC,
                      PC1_A4_HCTr2_Pin,
                      GPIO_PIN_RESET);


    /*
     * 2. Echo 인터럽트에서 계산된 후방 거리값 읽기
     */
    uint16_t rear_distance = Distance_Back;


    /*
     * 3. 유효하지 않은 측정값은 기존 상태 유지
     */
    if ((rear_distance < ULTRASONIC_MIN_VALID_CM) ||
        (rear_distance > ULTRASONIC_MAX_VALID_CM))
    {
      osDelay(120);
      continue;
    }


    /*
     * 4. 최초 유효 거리값으로 초기 상태 설정
     */
    if (first_valid_sample == 1)
    {
      current_rear_state =
          GetUltrasonicState(rear_distance);

      previous_rear_state =
          current_rear_state;

      g_rear_ultrasonic_state =
          current_rear_state;


      switch (current_rear_state)
      {
        case ULTRASONIC_STATE_10CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_10CM;
          break;

        case ULTRASONIC_STATE_20CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_20CM;
          break;

        case ULTRASONIC_STATE_30CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_30CM;
          break;

        case ULTRASONIC_STATE_CLEAR:
        default:
          g_rear_can_command =
              CAN_CMD_REAR_CLEAR;
          break;
      }

      first_valid_sample = 0;

      osDelay(120);
      continue;
    }


    /*
     * 5. 현재 거리값을 후방 경고 상태로 변환
     */
    current_rear_state =
        GetUltrasonicState(rear_distance);


    /*
     * 6. 후방 거리 상태가 바뀌었을 때만
     *    CAN 명령 생성
     */
    if (current_rear_state != previous_rear_state)
    {
      switch (current_rear_state)
      {
        case ULTRASONIC_STATE_10CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_10CM;
          break;

        case ULTRASONIC_STATE_20CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_20CM;
          break;

        case ULTRASONIC_STATE_30CM:
          g_rear_can_command =
              CAN_CMD_REAR_WARNING_30CM;
          break;

        case ULTRASONIC_STATE_CLEAR:
        default:
          g_rear_can_command =
              CAN_CMD_REAR_CLEAR;
          break;
      }


      /*
       * 외부 확인용 후방 상태 저장
       */
      g_rear_ultrasonic_state =
          current_rear_state;


      /*
       * 다음 상태 비교를 위해 현재 상태 저장
       */
      previous_rear_state =
          current_rear_state;
    }


    osDelay(120);
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

  
  /*
   * 이전 센서값
   */
  uint16_t prev_fw_raw = 0;
  uint16_t prev_oh_raw = 0;

  uint16_t prev_fw_filt = 0;
  uint16_t prev_oh_filt = 0;


  /*
   * 최초 실행 여부
   *
   * 시작 시 filtered 값이 0인 상태에서 delta를 계산하면
   * 실제 변화가 없더라도 큰 변화량이 발생할 수 있으므로 사용한다.
   */
  uint8_t first_sample = 1;


  /*
   * 일시적 그림자 및 밝음 복귀 확인 카운터
   */
  uint8_t shadow_count = 0;
  uint8_t light_off_count = 0;


  /*
   * 현재 판단 상태와 이전 상태
   */
  LightState_t current_light_state = LIGHT_STATE_OFF;
  LightState_t previous_light_state = LIGHT_STATE_OFF;


  /* Infinite loop */
  for (;;)
  {
    /*
     * 1. DMA 버퍼에서 전방 및 상공 조도센서값 읽기
     */
    uint16_t raw_fw = adc_dma_buff[0]; // LS1: 전방 조도센서
    uint16_t raw_oh = adc_dma_buff[1]; // LS2: 상공 조도센서


    /*
     * 2. 최초 센서값 초기화
     *
     * 첫 번째 값은 필터 계산과 변화량 판단에 바로 사용하지 않고
     * 초기 기준값으로 저장한다.
     */
    if (first_sample == 1)
    {
      g_auto_light.forward.raw = raw_fw;
      g_auto_light.overhead.raw = raw_oh;

      g_auto_light.forward.filtered = raw_fw;
      g_auto_light.overhead.filtered = raw_oh;

      g_auto_light.forward.delta = 0;
      g_auto_light.overhead.delta = 0;

      prev_fw_raw = raw_fw;
      prev_oh_raw = raw_oh;

      prev_fw_filt = raw_fw;
      prev_oh_filt = raw_oh;

      first_sample = 0;

      osDelay(100);
      continue;
    }


    /*
     * 3. raw 변화량 계산
     *
     * 양수: ADC값 감소 → 어두워짐
     * 음수: ADC값 증가 → 밝아짐
     */
    int16_t fw_raw_delta =
        (int16_t)prev_fw_raw - (int16_t)raw_fw;

    int16_t oh_raw_delta =
        (int16_t)prev_oh_raw - (int16_t)raw_oh;


    /*
     * 4. 현재 raw 값 저장
     */
    g_auto_light.forward.raw = raw_fw;
    g_auto_light.overhead.raw = raw_oh;


    /*
     * 5. Low-Pass Filter 적용
     *
     * 현재값 20%, 이전 필터값 80%
     */
    g_auto_light.forward.filtered =
        (uint16_t)((0.2f * (float)raw_fw) +
                   (0.8f * (float)g_auto_light.forward.filtered));

    g_auto_light.overhead.filtered =
        (uint16_t)((0.2f * (float)raw_oh) +
                   (0.8f * (float)g_auto_light.overhead.filtered));


    /*
     * 6. 필터값 변화량 계산
     *
     * 양수: ADC값 감소 → 어두워짐
     * 음수: ADC값 증가 → 밝아짐
     */
    g_auto_light.forward.delta =
        (int16_t)prev_fw_filt -
        (int16_t)g_auto_light.forward.filtered;

    g_auto_light.overhead.delta =
        (int16_t)prev_oh_filt -
        (int16_t)g_auto_light.overhead.filtered;


    /*
     * 7. 다음 주기 계산을 위해 현재값 저장
     */
    prev_fw_raw = raw_fw;
    prev_oh_raw = raw_oh;

    prev_fw_filt = g_auto_light.forward.filtered;
    prev_oh_filt = g_auto_light.overhead.filtered;


    /*
     * 8. 현재 필터값을 밝음/전환/어두움 단계로 변환
     */
    LightLevel_t forward_level =
        GetLightLevel(g_auto_light.forward.filtered);

    LightLevel_t overhead_level =
        GetLightLevel(g_auto_light.overhead.filtered);


    /*
     * 9-1. 터널 진입 판단
     *
     * 조건:
     * - 전방 raw 값이 1000 이하
     * - 상공 raw 값이 2900 이상
     * - 전방 raw 값이 한 주기 동안 1200 이상 급격히 감소
     *
     * 결과:
     * - 빠른 라이트 점등
     * - CAN 명령 112
     */
    if ((raw_fw <= LIGHT_DARK_THRESHOLD) &&
        (raw_oh >= LIGHT_BRIGHT_THRESHOLD) &&
        (fw_raw_delta >= LIGHT_RAW_RAPID_DROP))
    {
      current_light_state = LIGHT_STATE_FAST_ON;

      shadow_count = 0;
      light_off_count = 0;
    }


    /*
     * 9-2. 일몰 또는 야간 진입 판단
     *
     * 조건 1:
     * - 전방과 상공 센서 모두 1000 이하
     *
     * 또는 조건 2:
     * - 전방과 상공 필터값이 함께 감소
     *
     * 결과:
     * - 자연스러운 라이트 점등
     * - CAN 명령 113
     */
    else if (((forward_level == LIGHT_LEVEL_DARK) &&
              (overhead_level == LIGHT_LEVEL_DARK)) ||
             ((g_auto_light.forward.delta >= LIGHT_FILTERED_DROP) &&
              (g_auto_light.overhead.delta >= LIGHT_FILTERED_DROP)))
    {
      current_light_state = LIGHT_STATE_SMOOTH_ON;

      shadow_count = 0;
      light_off_count = 0;
    }


    /*
     * 9-3. 일시적 그림자 후보
     *
     * 조건:
     * - 전방은 어두움
     * - 상공은 밝음
     * - 터널 조건처럼 급격한 감소는 아님
     *
     * 짧게 유지되면 기존 상태를 그대로 유지한다.
     */
    else if ((raw_fw <= LIGHT_DARK_THRESHOLD) &&
             (raw_oh >= LIGHT_BRIGHT_THRESHOLD))
    {
      if (shadow_count < SHADOW_CONFIRM_COUNT)
      {
        shadow_count++;
      }

      light_off_count = 0;


      /*
       * 약 300ms 이상 전방 어두움이 지속되면
       * 지속적인 음영 또는 터널 구간으로 판단한다.
       */
      if (shadow_count >= SHADOW_CONFIRM_COUNT)
      {
        current_light_state = LIGHT_STATE_FAST_ON;
        shadow_count = 0;
      }
    }


    /*
     * 9-4. 밝은 환경 복귀 판단
     *
     * 전방과 상공 센서 모두 2900 이상인 상태가
     * 약 500ms 지속되면 라이트를 해제한다.
     */
    else if ((forward_level == LIGHT_LEVEL_BRIGHT) &&
             (overhead_level == LIGHT_LEVEL_BRIGHT))
    {
      shadow_count = 0;

      if (light_off_count < LIGHT_OFF_CONFIRM_COUNT)
      {
        light_off_count++;
      }

      if (light_off_count >= LIGHT_OFF_CONFIRM_COUNT)
      {
        current_light_state = LIGHT_STATE_OFF;
        light_off_count = 0;
      }
    }


    /*
     * 9-5. 전환 구간
     *
     * ADC 1001~2899 범위에서는
     * 현재 라이트 상태를 그대로 유지한다.
     */
    else
    {
      shadow_count = 0;
      light_off_count = 0;
    }


    /*
     * 10. 라이트 상태가 바뀌었을 때만
     *     CAN 명령값을 생성한다.
     *
     * 아직 여기서는 CAN으로 직접 전송하지 않는다.
     */
    if (current_light_state != previous_light_state)
    {
      switch (current_light_state)
      {
        case LIGHT_STATE_FAST_ON:
          g_light_can_command = CAN_CMD_LIGHT_FAST_ON;
          break;

        case LIGHT_STATE_SMOOTH_ON:
          g_light_can_command = CAN_CMD_LIGHT_SMOOTH_ON;
          break;

        case LIGHT_STATE_OFF:
          g_light_can_command = CAN_CMD_LIGHT_OFF;
          break;

        default:
          g_light_can_command = CAN_CMD_NONE;
          break;
      }


      /*
       * 외부에서 현재 라이트 상태를 확인할 수 있도록 저장
       */
      g_light_state = current_light_state;


      /*
       * 이번 상태를 다음 비교 기준으로 저장
       */
      previous_light_state = current_light_state;

    }



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


  /*
   * 실제 CAN으로 전송할 데이터 배열
   *
   * 현재는 CAN 모듈이 연결되지 않았으므로
   * UART를 통해 데이터 구성을 확인한다.
   */
  uint8_t txData[8] = {0, };


  /*
   * 실제 CAN 연결 후 사용할 변수
   *
   * 현재는 사용하지 않으므로 주석 처리한다.
   */
  // CAN_TxHeaderTypeDef txHeader;
  // uint32_t txMailbox;


  /*
   * CAN 연결 후 사용할 송신 헤더 설정
   */
  /*
  txHeader.StdId = 0x101;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;
  */


  /* Infinite loop */
  for (;;)
  {
    /*
     * TaskLD에서 생성한 CAN 명령을 읽는다.
     *
     * TaskCAN에서는 조도 조건을 다시 판단하지 않는다.
     */
    Node1CanCommand_t light_command = g_light_can_command;
    Node1CanCommand_t front_command = g_front_can_command;
    Node1CanCommand_t rear_command  = g_rear_can_command;


    /*
     * 실제 CAN 데이터 패킷 구성
     *
     * 현재는 첫 번째 바이트에
     * 조도 제어 명령을 저장한다.
     */
    txData[0] = (uint8_t)light_command;
    txData[1] = (uint8_t)front_command;
    txData[2] = (uint8_t)rear_command;
    txData[3] = 0;
    txData[4] = 0;
    txData[5] = 0;
    txData[6] = 0;
    txData[7] = 0;


    /*
     * UART 확인 출력
     *
     * TaskLD에서 계산된 센서값과
     * 실제 CAN으로 보낼 명령 및 데이터를 확인한다.
     */
     /*
    printf("[CAN CHECK] "
          "LIGHT_CMD=%3u FRONT_CMD=%3u REAR_CMD=%3u | "
          "TX=[%3u %3u %3u %3u %3u %3u %3u %3u] | "
          "LIGHT_STATE=%u FRONT_STATE=%u REAR_STATE=%u | "
          "DIST_F=%3ucm DIST_R=%3ucm | "
          "FW_RAW=%4u OH_RAW=%4u | "
          "FW_FILT=%4u OH_FILT=%4u | "
          "FW_DELTA=%5d OH_DELTA=%5d\r\n",

          (unsigned int)light_command,
          (unsigned int)front_command,
          (unsigned int)rear_command,

          (unsigned int)txData[0],
          (unsigned int)txData[1],
          (unsigned int)txData[2],
          (unsigned int)txData[3],
          (unsigned int)txData[4],
          (unsigned int)txData[5],
          (unsigned int)txData[6],
          (unsigned int)txData[7],

          (unsigned int)g_light_state,
          (unsigned int)g_front_ultrasonic_state,
          (unsigned int)g_rear_ultrasonic_state,

          (unsigned int)Distance_Front,
          (unsigned int)Distance_Back,

          (unsigned int)g_auto_light.forward.raw,
          (unsigned int)g_auto_light.overhead.raw,

          (unsigned int)g_auto_light.forward.filtered,
          (unsigned int)g_auto_light.overhead.filtered,

          (int)g_auto_light.forward.delta,
          (int)g_auto_light.overhead.delta);
    */


    /*
    * 초음파 Task UART 확인용 임시 출력
    *
    * 전방·후방 거리값
    * 거리 판단 상태
    * 생성된 CAN 명령만 확인한다.
    */
    printf("[ULTRASONIC CHECK] "
          "FRONT=%3ucm STATE=%u CMD=%3u | "
          "REAR=%3ucm STATE=%u CMD=%3u\r\n",

          (unsigned int)Distance_Front,
          (unsigned int)g_front_ultrasonic_state,
          (unsigned int)g_front_can_command,

          (unsigned int)Distance_Back,
          (unsigned int)g_rear_ultrasonic_state,
          (unsigned int)g_rear_can_command);

    /*
     * 실제 CAN 모듈 연결 후 사용할 송신 코드
     */
    /*
    if (light_command != CAN_CMD_NONE)
    {
      HAL_CAN_AddTxMessage(&hcan,
                           &txHeader,
                           txData,
                           &txMailbox);
    }
    */


    osDelay(500);
  }

  /* USER CODE END StartTaskCAN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * ADC값을 조도 단계로 변환하는 함수
 */
static LightLevel_t GetLightLevel(uint16_t adc_value)
{
    if (adc_value >= LIGHT_BRIGHT_THRESHOLD)
    {
        return LIGHT_LEVEL_BRIGHT;
    }
    else if (adc_value <= LIGHT_DARK_THRESHOLD)
    {
        return LIGHT_LEVEL_DARK;
    }
    else
    {
        return LIGHT_LEVEL_TRANSITION;
    }
}

/*
 * 초음파 거리값을 경고 단계로 변환하는 함수
 */
static UltrasonicState_t GetUltrasonicState(uint16_t distance_cm)
{
    /*
     * 가까운 구간부터 검사해야 한다.
     *
     * 예:
     * 8cm는 10cm, 20cm, 30cm 조건을 모두 만족하므로
     * 가장 먼저 10cm 구간으로 분류한다.
     */
    if (distance_cm <= ULTRASONIC_WARNING_10CM)
    {
        return ULTRASONIC_STATE_10CM;
    }
    else if (distance_cm <= ULTRASONIC_WARNING_20CM)
    {
        return ULTRASONIC_STATE_20CM;
    }
    else if (distance_cm <= ULTRASONIC_WARNING_30CM)
    {
        return ULTRASONIC_STATE_30CM;
    }
    else
    {
        return ULTRASONIC_STATE_CLEAR;
    }
}


/* USER CODE END Application */

