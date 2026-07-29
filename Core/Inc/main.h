/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/*
 * Main Node에서 수신하는 차량 주행 상태
 *
 * 제어명령 60번의 Value에 따라
 * Node_1 내부의 현재 주행 상태를 저장한다.
 *
 * 이 상태는 Node_1이 다시 송신하는 데이터가 아니며,
 * 초음파 Event 송신 방향을 결정하는 조건으로만 사용한다.
 */
typedef enum
{
    /*
     * 전진 상태
     *
     * 전방 초음파 Event와
     * 전방 경고 해제 Event만 송신한다.
     */
    DRIVE_STATE_FORWARD = 0,

    /*
     * 후진 상태
     *
     * 후방 초음파 Event와
     * 후방 경고 해제 Event만 송신한다.
     */
    DRIVE_STATE_BACKWARD = 1,

    /*
     * 정지 상태
     *
     * 전방·후방 초음파는 계속 측정하지만
     * 초음파 Event를 송신하지 않는다.
     */
    DRIVE_STATE_STOP = 2

} DriveState_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PC0_A5_HCTr1_Pin GPIO_PIN_0
#define PC0_A5_HCTr1_GPIO_Port GPIOC
#define PC1_A4_HCTr2_Pin GPIO_PIN_1
#define PC1_A4_HCTr2_GPIO_Port GPIOC
#define PA0_A0_LS1_Pin GPIO_PIN_0
#define PA0_A0_LS1_GPIO_Port GPIOA
#define PA1_A1_LS2_Pin GPIO_PIN_1
#define PA1_A1_LS2_GPIO_Port GPIOA
#define PA2_D1_TX_Pin GPIO_PIN_2
#define PA2_D1_TX_GPIO_Port GPIOA
#define PA3_D0_RX_Pin GPIO_PIN_3
#define PA3_D0_RX_GPIO_Port GPIOA
#define PA4_A2_TP_Pin GPIO_PIN_4
#define PA4_A2_TP_GPIO_Port GPIOA
#define PA5_D13_GLED_Pin GPIO_PIN_5
#define PA5_D13_GLED_GPIO_Port GPIOA
#define PA6_D12_HCEc1_Pin GPIO_PIN_6
#define PA6_D12_HCEc1_GPIO_Port GPIOA
#define PA7_D11_HCEc2_Pin GPIO_PIN_7
#define PA7_D11_HCEc2_GPIO_Port GPIOA
#define PA11_CanRx_Pin GPIO_PIN_11
#define PA11_CanRx_GPIO_Port GPIOA
#define PA12_CanTx_Pin GPIO_PIN_12
#define PA12_CanTx_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
