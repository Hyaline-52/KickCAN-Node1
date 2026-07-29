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
#include "queue.h"   // FreeRTOS 구조체 큐 사용
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
 * CAN 송신 큐에 저장할 메시지
 *
 * 각 센서 Task는 센서 상태를 판단한 뒤
 * 실제 CAN 송신에 필요한 정보만 이 구조체에 담아
 * QueueCAN으로 전달한다.
 *
 * 실제 CAN Header 구성, Payload 패킹 및 송신은
 * TaskCAN에서만 수행한다.
 */
typedef struct
{
    /*
     * CAN Header의 Standard Identifier로 사용할 수신 노드 ID
     *
     * 현재 Node_1에서 발생한 Event는
     * Node_2로 전송하므로 기본 목적지는 0x003이다.
     */
    uint16_t destination_id;

    /*
     * CAN Payload의 Data[0]에 저장할 제어 명령
     *
     * 예:
     * 110 = 온도 경고
     * 115 = 전방 30cm 경고
     */
    uint8_t command;

    /*
     * CAN Payload의 Data[1]에 저장할 실제 상태값
     */
    uint8_t value;

} CanTxMessage_t;


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

/*
 * TMP36 온도센서 데이터 관리 구조체
 */
typedef struct
{
    uint16_t raw;             // ADC 원본값
    uint16_t filtered_raw;    // 필터 적용 ADC값
    float voltage;            // 변환 전압
    float temperature;        // 계산된 온도(℃)

} TemperatureSensor_t;


// 전체 오토라이트 & TP 시스템 구조체
typedef struct
{
    LightSensor_t forward;  // LS1 (PA0)
    LightSensor_t overhead; // LS2 (PA1)
    TemperatureSensor_t tp;      // TMP36 (PA4)

} AutoLight_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * TMP36 온도 경고 기준값
 *
 * 테스트 단계:
 * 37도 이상 → 온도 경고 명령 110
 *
 * 최종 동작:
 * 50도 이상 → 온도 경고 명령 110
 *
 * 경고 해제:
 * 30도 이하 → 정상 복귀 명령 111
 */
#define TEMP_WARNING_THRESHOLD_TEST       37.0f
#define TEMP_WARNING_THRESHOLD_FINAL      50.0f
#define TEMP_NORMAL_THRESHOLD             30.0f


/*
 * 현재는 손으로 센서를 가열하여 동작을 확인해야 하므로
 * 테스트 기준값 37도를 사용한다.
 *
 * 최종 테스트가 끝난 뒤 아래 값을
 * TEMP_WARNING_THRESHOLD_FINAL로 변경한다.
 */
#define TEMP_WARNING_THRESHOLD \
        TEMP_WARNING_THRESHOLD_TEST
        // TEMP_WARNING_THRESHOLD_FINAL


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

/*
 * Node_2의 CAN 수신 Identifier
 *
 * CAN Identifier는 수신 노드 ID로 사용한다.
 * Node_1에서 발생한 센서 Event는
 * Node_2(ID 3)로 전송한다.
 */
#define CAN_NODE2_ID                       0x003U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/*
 * main.c에 선언된 거리 변수 및 us delay 함수 참조
 */
extern volatile uint16_t Distance_Front;
extern volatile uint16_t Distance_Back;
extern void delay_us(uint16_t us);


/*
 * Main Node에서 수신한 현재 차량 주행 상태
 *
 * main.c의 CAN 수신 콜백에서 값이 변경되며,
 * TaskHC1과 TaskHC2에서는 이 값을 읽기만 한다.
 *
 * Node_1이 이 주행 상태를 다시 송신하는 것은 아니다.
 */
extern volatile DriveState_t g_drive_state;


// PA0(LS1), PA1(LS2), PA4(TP) 센서 값이 저장될 DMA 버퍼
uint16_t adc_dma_buff[3] = {0, };


// 전체 센서값 관리 구조체
AutoLight_t g_auto_light;

/*
 * 현재 온도 경고 상태
 */
typedef enum
{
    TEMP_STATE_NORMAL = 0,    // 정상 상태
    TEMP_STATE_WARNING        // 고온 경고 상태

} TemperatureState_t;

/*
 * TaskTP에서 판단한 현재 온도 상태
 */
volatile TemperatureState_t g_temperature_state = TEMP_STATE_NORMAL;


/*
 * TaskTP에서 생성한 온도 CAN 명령
 *
 * 110: 온도 경고
 * 111: 온도 정상 복귀
 */
volatile Node1CanCommand_t g_temperature_can_command = CAN_CMD_NONE;


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

/*
 * CAN 송신 메시지를 저장할 FreeRTOS 큐 핸들
 *
 * 큐 하나의 항목에는
 * CanTxMessage_t 구조체 전체가 저장된다.
 */
QueueHandle_t QueueCANHandle;

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

  /* Create the queue(s) */
  /* definition and creation of QueueCAN */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* Create the queue(s) */

  /*
  * CAN 송신 큐 생성
  *
  * 큐 길이:
  * 최대 16개의 CAN 송신 요청을 대기시킬 수 있다.
  *
  * 큐 항목 크기:
  * CanTxMessage_t 구조체 하나의 크기
  *
  * 센서 Task는 이 큐에
  * 수신 노드 ID, Command, Value를 저장한다.
  *
  * TaskCAN은 큐에서 메시지를 하나씩 꺼내
  * 실제 CAN Header와 Payload를 구성한다.
  */
  QueueCANHandle =
      xQueueCreate(16, sizeof(CanTxMessage_t));


  /*
  * 큐 생성 실패 확인
  *
  * 메모리 부족 등의 이유로 큐가 생성되지 않으면
  * 더 이상 정상적인 CAN 송신이 불가능하므로
  * Error_Handler로 진입한다.
  */
  if (QueueCANHandle == NULL)
  {
      Error_Handler();
  }
  

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


  /*
   * 현재 온도 상태와 이전 온도 상태
   *
   * 상태가 실제로 바뀌었을 때만
   * 새로운 CAN 명령을 생성하기 위해 사용한다.
   */
  TemperatureState_t current_temperature_state =
      TEMP_STATE_NORMAL;

  TemperatureState_t previous_temperature_state =
      TEMP_STATE_NORMAL;


  /*
   * 최초 ADC 측정 여부
   *
   * filtered_raw가 0인 상태에서 바로 필터를 적용하면
   * 실제 ADC값보다 낮은 값에서 천천히 상승하므로,
   * 최초 측정값을 필터 초기값으로 사용한다.
   */
  uint8_t first_sample = 1;


  for (;;)
  {
    /*
     * 1. DMA 버퍼에서 TMP36 ADC값 읽기
     *
     * adc_dma_buff[2]:
     * ADC1_IN4, PA4에 연결된 TMP36
     */
    uint16_t raw_tp = adc_dma_buff[2];


    /*
     * 2. 최초 측정값 초기화
     */
    if (first_sample == 1)
    {
      g_auto_light.tp.raw = raw_tp;
      g_auto_light.tp.filtered_raw = raw_tp;

      first_sample = 0;
    }
    else
    {
      /*
       * 3. 현재 ADC 원본값 저장
       */
      g_auto_light.tp.raw = raw_tp;


      /*
       * 4. Low-Pass Filter 적용
       *
       * 현재 ADC값 20%
       * 이전 필터값 80%
       */
      g_auto_light.tp.filtered_raw =
          (uint16_t)((0.2f * (float)raw_tp) +
                     (0.8f *
                      (float)g_auto_light.tp.filtered_raw));
    }


    /*
     * 5. ADC값을 전압으로 변환
     *
     * STM32 ADC:
     * 12bit → 0~4095
     * 기준전압 → 3.3V
     */
    g_auto_light.tp.voltage =
        ((float)g_auto_light.tp.filtered_raw * 3.3f) /
        4095.0f;


    /*
     * 6. TMP36 출력전압을 온도로 변환
     *
     * TMP36:
     * 0℃에서 약 0.5V
     * 1℃당 약 10mV 증가
     *
     * Temperature = (Voltage - 0.5) × 100
     */
    g_auto_light.tp.temperature =
        (g_auto_light.tp.voltage - 0.5f) * 100.0f;


    /*
     * 7. 온도 상태 판단
     *
     * 정상 상태에서 설정 기준 이상이 되면 경고 상태로 전환한다.
     *
     * 현재 테스트 기준:
     * 37℃ 이상 → 경고
     *
     * 최종 기준:
     * 50℃ 이상 → 경고
     */
    if ((current_temperature_state == TEMP_STATE_NORMAL) &&
        (g_auto_light.tp.temperature >=
         TEMP_WARNING_THRESHOLD))
    {
      current_temperature_state =
          TEMP_STATE_WARNING;
    }


    /*
     * 경고 상태에서는 온도가 30℃ 이하로 내려가야
     * 정상 상태로 복귀한다.
     *
     * 경고 진입 온도와 해제 온도를 다르게 두어
     * 경계 온도에서 명령이 반복되는 것을 방지한다.
     */
    else if ((current_temperature_state == TEMP_STATE_WARNING) &&
             (g_auto_light.tp.temperature <=
              TEMP_NORMAL_THRESHOLD))
    {
      current_temperature_state =
          TEMP_STATE_NORMAL;
    }


    /*
     * 8. 온도 상태가 바뀌었을 때만
     *    CAN 명령을 생성한다.
     *
     * 여기서는 실제 CAN 송신을 하지 않는다.
     */
    if (current_temperature_state !=
        previous_temperature_state)
    {
      switch (current_temperature_state)
      {
        case TEMP_STATE_WARNING:

          /*
           * 온도 경고 명령 110
           */
          g_temperature_can_command =
              CAN_CMD_TEMP_WARNING;
          break;


        case TEMP_STATE_NORMAL:
        default:

          /*
           * 온도 정상 복귀 명령 111
           */
          g_temperature_can_command =
              CAN_CMD_TEMP_NORMAL;
          break;
      }


      /*
       * 외부 확인용 현재 온도 상태 저장
       */
      g_temperature_state =
          current_temperature_state;


      /*
       * 다음 상태 비교를 위해 현재 상태 저장
       */
      previous_temperature_state =
          current_temperature_state;
    }


    /*
     * 온도 측정 주기: 100ms
     */
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
   * 현재 측정된 전방 초음파 상태
   */
  UltrasonicState_t current_front_state = ULTRASONIC_STATE_CLEAR;


  /*
   * 이전 측정 주기의 전방 초음파 상태
   *
   * 전진 상태를 유지하는 동안
   * 거리 단계가 실제로 변경되었는지 확인하기 위해 사용한다.
   */
  UltrasonicState_t previous_front_state = ULTRASONIC_STATE_CLEAR;


  /*
   * TaskHC1이 마지막으로 확인한 주행 상태
   *
   * Main Node에서 받은 주행 상태가
   * 정지 또는 후진에서 전진으로 변경되었는지
   * 확인하기 위해 사용한다.
   *
   * 시스템 초기 상태는 정지로 설정한다.
   */
  DriveState_t previous_drive_state = DRIVE_STATE_STOP;


  /*
   * 최초 유효 거리 측정 여부
   *
   * 시스템 시작 직후 Distance_Front가 0인 상태를
   * 실제 거리 상태로 사용하지 않기 위해 사용한다.
   */
  uint8_t first_valid_sample = 1U;


  for (;;)
  {
    /*
     * 1. 전방 초음파 Trig 신호 발생
     *
     * PC0:
     * 전방 HC-SR04의 Trig 출력
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
     * 2. Echo 인터럽트에서 계산된
     *    전방 거리값을 읽는다.
     */
    uint16_t front_distance = Distance_Front;


    /*
     * 3. Main Node에서 수신한 현재 주행 상태를 읽는다.
     *
     * 이 값은 Node_1이 다시 송신하지 않는다.
     * 전방 초음파 Event의 송신 여부를
     * 결정하는 조건값으로만 사용한다.
     */
    DriveState_t current_drive_state = g_drive_state;


    /*
     * 4. 전방 거리값의 유효 범위를 확인한다.
     *
     * 0cm:
     * 아직 측정되지 않았거나 Echo 측정 실패
     *
     * 400cm 초과:
     * HC-SR04의 일반적인 측정 범위를 벗어난 값
     *
     * 유효하지 않은 경우:
     * - 기존 거리 상태 유지
     * - 초음파 Event 생성 안 함
     * - 이전 주행 상태도 갱신하지 않음
     *
     * 이전 주행 상태를 갱신하지 않는 이유:
     * 전진 상태로 변경된 순간 거리 측정이 실패하더라도,
     * 다음 유효 거리 측정 시 현재 전방 Event를
     * 한 번 송신하기 위해서다.
     */
    if ((front_distance < ULTRASONIC_MIN_VALID_CM) ||
        (front_distance > ULTRASONIC_MAX_VALID_CM))
    {
      osDelay(120);
      continue;
    }


    /*
     * 5. 현재 거리값을 전방 경고 단계로 변환한다.
     *
     * CLEAR : 30cm 초과
     * 30CM  : 21~30cm
     * 20CM  : 11~20cm
     * 10CM  : 1~10cm
     */
    current_front_state = GetUltrasonicState(front_distance);


    /*
     * 이번 측정에서 전방 거리 단계가
     * 실제로 변경되었는지 저장한다.
     */
    uint8_t front_state_changed = 0U;


    /*
     * 6. 최초 유효 거리값 처리
     *
     * 최초 유효 측정값을 현재 상태와
     * 이전 비교 상태로 함께 저장한다.
     */
    if (first_valid_sample == 1U)
    {
      previous_front_state = current_front_state;

      first_valid_sample = 0U;
    }


    /*
     * 최초 측정 이후에는 현재 상태와 이전 상태를 비교한다.
     */
    else if (current_front_state != previous_front_state)
    {
      front_state_changed = 1U;

      /*
       * 다음 거리 상태 비교를 위해
       * 현재 상태를 이전 상태로 저장한다.
       */
      previous_front_state = current_front_state;
    }


    /*
     * 외부 확인용 현재 전방 초음파 상태 저장
     *
     * 주행 상태와 관계없이 항상 최신 상태를 저장한다.
     */
    g_front_ultrasonic_state = current_front_state;


    /*
     * 7. 전진 상태로 새롭게 진입했는지 확인한다.
     *
     * 정지 → 전진
     * 후진 → 전진
     *
     * 두 경우 모두 현재 전방 거리 상태에 해당하는
     * Event를 한 번 송신해야 한다.
     */
    uint8_t entered_forward = 0U;

    if ((current_drive_state == DRIVE_STATE_FORWARD) &&
        (previous_drive_state != DRIVE_STATE_FORWARD))
    {
      entered_forward = 1U;
    }


    /*
     * 8. 전방 초음파 Event 송신 조건
     *
     * 현재 주행 상태가 전진이면서 다음 중 하나를 만족:
     *
     * 1) 정지 또는 후진에서 전진으로 변경됨
     * 2) 전진을 유지하는 동안 전방 거리 단계가 변경됨
     *
     * 후진 또는 정지 상태에서는
     * 전방 초음파 Event를 큐에 등록하지 않는다.
     */
    if ((current_drive_state == DRIVE_STATE_FORWARD) &&
        ((entered_forward == 1U) ||
         (front_state_changed == 1U)))
    {
      /*
       * TaskCAN으로 전달할 CAN 송신 요청
       *
       * 센서 Task에서는:
       * - 수신 노드 ID
       * - Command
       * - Value
       *
       * 만 설정한다.
       *
       * 실제 CAN Header 구성과 Payload 패킹은
       * TaskCAN에서 수행한다.
       */
      CanTxMessage_t front_message;


      /*
       * 전방 초음파 Event의 목적지는
       * Node_2(ID 3)이다.
       */
      front_message.destination_id = CAN_NODE2_ID;


      /*
       * 현재 전방 거리 상태에 따라
       * Command와 Value를 결정한다.
       */
      switch (current_front_state)
      {
        case ULTRASONIC_STATE_10CM:

          /*
           * 전방 10cm 경고
           *
           * Command = 117
           * Value   = 3
           */
          front_message.command = (uint8_t)CAN_CMD_FRONT_WARNING_10CM;

          front_message.value = 3U;
          break;


        case ULTRASONIC_STATE_20CM:

          /*
           * 전방 20cm 경고
           *
           * Command = 116
           * Value   = 2
           */
          front_message.command = (uint8_t)CAN_CMD_FRONT_WARNING_20CM;

          front_message.value = 2U;
          break;


        case ULTRASONIC_STATE_30CM:

          /*
           * 전방 30cm 경고
           *
           * Command = 115
           * Value   = 1
           */
          front_message.command = (uint8_t)CAN_CMD_FRONT_WARNING_30CM;

          front_message.value = 1U;
          break;


        case ULTRASONIC_STATE_CLEAR:
        default:

          /*
           * 전방 경고 해제
           *
           * Command = 121
           * Value   = 0
           */
          front_message.command = (uint8_t)CAN_CMD_FRONT_CLEAR;

          front_message.value = 0U;
          break;
      }


      /*
       * CAN 송신 요청을 QueueCAN에 등록한다.
       *
       * 실제 CAN 송신은 여기에서 수행하지 않는다.
       * TaskCAN이 큐에서 메시지를 꺼내 송신한다.
       *
       * 큐가 잠시 가득 차 있는 경우에도
       * Event가 사라지지 않도록 공간이 생길 때까지 대기한다.
       *
       * 초음파 Event는 거리 상태가 변경될 때만 발생하므로
       * 일반적인 동작에서는 Task가 장시간 대기하지 않는다.
       */
      xQueueSend(QueueCANHandle,
                 &front_message,
                 portMAX_DELAY);
    }


    /*
     * 9. 다음 주기의 주행 상태 비교를 위해
     *    현재 주행 상태를 저장한다.
     *
     * 전진에서 정지 또는 후진으로 변경되는 경우에는
     * 별도의 전방 경고 해제 Event를 송신하지 않는다.
     *
     * Node_2는 Main Node에서 주행 상태를 별도로 받아
     * 현재 사용하지 않는 방향의 Event를 처리한다.
     */
    previous_drive_state =
        current_drive_state;


    /*
     * 전방 초음파 측정 주기
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


  /*
   * 현재 측정된 후방 초음파 상태
   */
  UltrasonicState_t current_rear_state = ULTRASONIC_STATE_CLEAR;


  /*
   * 이전 측정 주기의 후방 초음파 상태
   *
   * 후진 상태를 유지하는 동안
   * 거리 단계가 실제로 변경되었는지 확인하기 위해 사용한다.
   */
  UltrasonicState_t previous_rear_state = ULTRASONIC_STATE_CLEAR;


  /*
   * TaskHC2가 마지막으로 확인한 주행 상태
   *
   * Main Node에서 받은 주행 상태가
   * 정지 또는 전진에서 후진으로 변경되었는지
   * 확인하기 위해 사용한다.
   *
   * 시스템 초기 상태는 정지로 설정한다.
   */
  DriveState_t previous_drive_state = DRIVE_STATE_STOP;


  /*
   * 최초 유효 거리 측정 여부
   *
   * 시스템 시작 직후 Distance_Back이 0인 상태를
   * 실제 거리 상태로 사용하지 않기 위해 사용한다.
   */
  uint8_t first_valid_sample = 1U;


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
     * 1. 후방 초음파 Trig 신호 발생
     *
     * PC1:
     * 후방 HC-SR04의 Trig 출력
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
     * 2. Echo 인터럽트에서 계산된
     *    후방 거리값을 읽는다.
     */
    uint16_t rear_distance = Distance_Back;


    /*
     * 3. Main Node에서 수신한 현재 주행 상태를 읽는다.
     *
     * 이 값은 Node_1이 다시 송신하지 않는다.
     * 후방 초음파 Event의 송신 여부를
     * 결정하는 조건값으로만 사용한다.
     */
    DriveState_t current_drive_state = g_drive_state;


    /*
     * 4. 후방 거리값의 유효 범위를 확인한다.
     *
     * 0cm:
     * 아직 측정되지 않았거나 Echo 측정 실패
     *
     * 400cm 초과:
     * HC-SR04의 일반적인 측정 범위를 벗어난 값
     *
     * 유효하지 않은 경우:
     * - 기존 거리 상태 유지
     * - 후방 Event 생성 안 함
     * - 이전 주행 상태도 갱신하지 않음
     *
     * 이전 주행 상태를 갱신하지 않는 이유:
     * 후진 상태로 변경된 순간 거리 측정이 실패하더라도,
     * 다음 유효 거리 측정 시 현재 후방 Event를
     * 한 번 송신하기 위해서다.
     */
    if ((rear_distance < ULTRASONIC_MIN_VALID_CM) ||
        (rear_distance > ULTRASONIC_MAX_VALID_CM))
    {
      osDelay(120);
      continue;
    }


    /*
     * 5. 현재 거리값을 후방 경고 단계로 변환한다.
     *
     * CLEAR : 30cm 초과
     * 30CM  : 21~30cm
     * 20CM  : 11~20cm
     * 10CM  : 1~10cm
     */
    current_rear_state = GetUltrasonicState(rear_distance);


    /*
     * 이번 측정에서 후방 거리 단계가
     * 실제로 변경되었는지 저장한다.
     */
    uint8_t rear_state_changed = 0U;


    /*
     * 6. 최초 유효 거리값 처리
     *
     * 최초 유효 측정값을 현재 상태와
     * 이전 비교 상태로 함께 저장한다.
     */
    if (first_valid_sample == 1U)
    {
      previous_rear_state = current_rear_state;

      first_valid_sample = 0U;
    }


    /*
     * 최초 측정 이후에는 현재 상태와 이전 상태를 비교한다.
     */
    else if (current_rear_state != previous_rear_state)
    {
      rear_state_changed = 1U;


      /*
       * 다음 거리 상태 비교를 위해
       * 현재 상태를 이전 상태로 저장한다.
       */
      previous_rear_state = current_rear_state;
    }


    /*
     * 외부 확인용 현재 후방 초음파 상태 저장
     *
     * 주행 상태와 관계없이 항상 최신 상태를 저장한다.
     */
    g_rear_ultrasonic_state = current_rear_state;


    /*
     * 7. 후진 상태로 새롭게 진입했는지 확인한다.
     *
     * 정지 → 후진
     * 전진 → 후진
     *
     * 두 경우 모두 현재 후방 거리 상태에 해당하는
     * Event를 한 번 송신해야 한다.
     */
    uint8_t entered_backward = 0U;

    if ((current_drive_state == DRIVE_STATE_BACKWARD) &&
        (previous_drive_state != DRIVE_STATE_BACKWARD))
    {
      entered_backward = 1U;
    }


    /*
     * 8. 후방 초음파 Event 송신 조건
     *
     * 현재 주행 상태가 후진이면서 다음 중 하나를 만족:
     *
     * 1) 정지 또는 전진에서 후진으로 변경됨
     * 2) 후진을 유지하는 동안 후방 거리 단계가 변경됨
     *
     * 전진 또는 정지 상태에서는
     * 후방 초음파 Event를 큐에 등록하지 않는다.
     */
    if ((current_drive_state == DRIVE_STATE_BACKWARD) &&
        ((entered_backward == 1U) ||
         (rear_state_changed == 1U)))
    {
      /*
       * TaskCAN으로 전달할 CAN 송신 요청
       *
       * 센서 Task에서는:
       * - 수신 노드 ID
       * - Command
       * - Value
       *
       * 만 설정한다.
       *
       * 실제 CAN Header 구성과 Payload 패킹은
       * TaskCAN에서 수행한다.
       */
      CanTxMessage_t rear_message;


      /*
       * 후방 초음파 Event의 목적지는
       * Node_2(ID 3)이다.
       */
      rear_message.destination_id = CAN_NODE2_ID;


      /*
       * 현재 후방 거리 상태에 따라
       * Command와 Value를 결정한다.
       */
      switch (current_rear_state)
      {
        case ULTRASONIC_STATE_10CM:

          /*
           * 후방 10cm 경고
           *
           * Command = 120
           * Value   = 3
           */
          rear_message.command = (uint8_t)CAN_CMD_REAR_WARNING_10CM;

          rear_message.value = 3U;
          break;


        case ULTRASONIC_STATE_20CM:

          /*
           * 후방 20cm 경고
           *
           * Command = 119
           * Value   = 2
           */
          rear_message.command = (uint8_t)CAN_CMD_REAR_WARNING_20CM;

          rear_message.value = 2U;
          break;


        case ULTRASONIC_STATE_30CM:

          /*
           * 후방 30cm 경고
           *
           * Command = 118
           * Value   = 1
           */
          rear_message.command = (uint8_t)CAN_CMD_REAR_WARNING_30CM;

          rear_message.value = 1U;
          break;


        case ULTRASONIC_STATE_CLEAR:
        default:

          /*
           * 후방 경고 해제
           *
           * Command = 122
           * Value   = 0
           */
          rear_message.command = (uint8_t)CAN_CMD_REAR_CLEAR;

          rear_message.value = 0U;
          break;
      }


      /*
       * CAN 송신 요청을 QueueCAN에 등록한다.
       *
       * 실제 CAN 송신은 이 Task에서 수행하지 않는다.
       * TaskCAN이 큐에서 메시지를 꺼내 송신한다.
       *
       * 큐가 가득 찬 경우에는
       * 공간이 생길 때까지 대기한다.
       */
      xQueueSend(QueueCANHandle,
                 &rear_message,
                 portMAX_DELAY);
    }


    /*
     * 9. 다음 주기의 주행 상태 비교를 위해
     *    현재 주행 상태를 저장한다.
     *
     * 후진에서 정지 또는 전진으로 변경되는 경우에는
     * 별도의 후방 경고 해제 Event를 송신하지 않는다.
     *
     * Node_2는 Main Node에서 현재 주행 상태를
     * 별도로 수신해서 처리한다.
     */
    previous_drive_state = current_drive_state;


    /*
     * 후방 초음파 측정 주기
     */
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

    Node1CanCommand_t temperature_command = g_temperature_can_command;

    Node1CanCommand_t light_command = g_light_can_command;

    Node1CanCommand_t front_command = g_front_can_command;

    Node1CanCommand_t rear_command = g_rear_can_command;


    /*
     * 실제 CAN 데이터 패킷 구성
     *
     * 현재는 첫 번째 바이트에
     * 조도 제어 명령을 저장한다.
     */
    txData[0] = (uint8_t)temperature_command;
    txData[1] = (uint8_t)light_command;
    txData[2] = (uint8_t)front_command;
    txData[3] = (uint8_t)rear_command;
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
    // printf("[ULTRASONIC CHECK] "
    //       "FRONT=%3ucm STATE=%u CMD=%3u | "
    //       "REAR=%3ucm STATE=%u CMD=%3u\r\n",

    //       (unsigned int)Distance_Front,
    //       (unsigned int)g_front_ultrasonic_state,
    //       (unsigned int)g_front_can_command,

    //       (unsigned int)Distance_Back,
    //       (unsigned int)g_rear_ultrasonic_state,
    //       (unsigned int)g_rear_can_command);

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

    /*
    * TMP36 온도센서 UART 확인
    *
    * 모든 온도 계산은 TaskTP에서 수행하고,
    * TaskCAN에서는 결과만 출력한다.
    */
    printf("[TEMP CHECK] "
          "RAW=%4u FILT=%4u | "
          "VOLT=%.3fV TEMP=%.1f degC | "
          "STATE=%u CMD=%3u | "
          "WARN=%.1f degC NORMAL=%.1f degC\r\n",

          (unsigned int)g_auto_light.tp.raw,
          (unsigned int)g_auto_light.tp.filtered_raw,

          (double)g_auto_light.tp.voltage,
          (double)g_auto_light.tp.temperature,

          (unsigned int)g_temperature_state,
          (unsigned int)temperature_command,

          (double)TEMP_WARNING_THRESHOLD,
          (double)TEMP_NORMAL_THRESHOLD);


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

