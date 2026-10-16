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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSD_CS_Pin GPIO_PIN_14
#define OSD_CS_GPIO_Port GPIOC
#define BEEPER_Pin GPIO_PIN_15
#define BEEPER_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define VBAT_SENSE_Pin GPIO_PIN_0
#define VBAT_SENSE_GPIO_Port GPIOC
#define CURR_SENSE_Pin GPIO_PIN_1
#define CURR_SENSE_GPIO_Port GPIOC
#define SPI2_MISO_FLASH_Pin GPIO_PIN_2
#define SPI2_MISO_FLASH_GPIO_Port GPIOC
#define SPI2_MOSI_FLASH_Pin GPIO_PIN_3
#define SPI2_MOSI_FLASH_GPIO_Port GPIOC
#define UART4_TX_DJI_Pin GPIO_PIN_0
#define UART4_TX_DJI_GPIO_Port GPIOA
#define UART4_RX_DJI_Pin GPIO_PIN_1
#define UART4_RX_DJI_GPIO_Port GPIOA
#define USART2_TX_RC_Pin GPIO_PIN_2
#define USART2_TX_RC_GPIO_Port GPIOA
#define USART2_RX_RC_Pin GPIO_PIN_3
#define USART2_RX_RC_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_4
#define GYRO_CS_GPIO_Port GPIOA
#define SPI1_SCK_GYRO_Pin GPIO_PIN_5
#define SPI1_SCK_GYRO_GPIO_Port GPIOA
#define SPI1_MISO_GYRO_Pin GPIO_PIN_6
#define SPI1_MISO_GYRO_GPIO_Port GPIOA
#define SPI1_MOSI_GYRO_Pin GPIO_PIN_7
#define SPI1_MOSI_GYRO_GPIO_Port GPIOA
#define GYRO_EXTI_Pin GPIO_PIN_4
#define GYRO_EXTI_GPIO_Port GPIOC
#define GYRO_EXTI_EXTI_IRQn EXTI4_IRQn
#define RSSI_SENSE_Pin GPIO_PIN_5
#define RSSI_SENSE_GPIO_Port GPIOC
#define CAM_CONTROL_Pin GPIO_PIN_0
#define CAM_CONTROL_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_1
#define SERVO1_GPIO_Port GPIOB
#define MOTOR8_Pin GPIO_PIN_10
#define MOTOR8_GPIO_Port GPIOB
#define MOTOR7_Pin GPIO_PIN_11
#define MOTOR7_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define SPI2_SCK_FLASH_Pin GPIO_PIN_13
#define SPI2_SCK_FLASH_GPIO_Port GPIOB
#define PINIO1_CAM_SWITCH_Pin GPIO_PIN_14
#define PINIO1_CAM_SWITCH_GPIO_Port GPIOB
#define MOTOR4_Pin GPIO_PIN_15
#define MOTOR4_GPIO_Port GPIOB
#define USART6_TX_USER_Pin GPIO_PIN_6
#define USART6_TX_USER_GPIO_Port GPIOC
#define USART6_RX_USER_Pin GPIO_PIN_7
#define USART6_RX_USER_GPIO_Port GPIOC
#define MOTOR3_Pin GPIO_PIN_8
#define MOTOR3_GPIO_Port GPIOC
#define MOTOR2_Pin GPIO_PIN_9
#define MOTOR2_GPIO_Port GPIOC
#define MOTOR1_Pin GPIO_PIN_8
#define MOTOR1_GPIO_Port GPIOA
#define USART1_TX_ESC_TLM_Pin GPIO_PIN_9
#define USART1_TX_ESC_TLM_GPIO_Port GPIOA
#define USART1_RX_ESC_TLM_Pin GPIO_PIN_10
#define USART1_RX_ESC_TLM_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOA
#define MOTOR6_Pin GPIO_PIN_15
#define MOTOR6_GPIO_Port GPIOA
#define USART3_TX_VTX_Pin GPIO_PIN_10
#define USART3_TX_VTX_GPIO_Port GPIOC
#define USART3_RX_VTX_Pin GPIO_PIN_11
#define USART3_RX_VTX_GPIO_Port GPIOC
#define UART5_TX_GPS_Pin GPIO_PIN_12
#define UART5_TX_GPS_GPIO_Port GPIOC
#define UART5_RX_GPS_Pin GPIO_PIN_2
#define UART5_RX_GPS_GPIO_Port GPIOD
#define SPI3_SCK_OSD_Pin GPIO_PIN_3
#define SPI3_SCK_OSD_GPIO_Port GPIOB
#define SPI3_MISO_OSD_Pin GPIO_PIN_4
#define SPI3_MISO_OSD_GPIO_Port GPIOB
#define SPI3_MOSI_OSD_Pin GPIO_PIN_5
#define SPI3_MOSI_OSD_GPIO_Port GPIOB
#define MOTOR5_Pin GPIO_PIN_6
#define MOTOR5_GPIO_Port GPIOB
#define LED_STRIP_Pin GPIO_PIN_7
#define LED_STRIP_GPIO_Port GPIOB
#define I2C1_SCL_BARO_Pin GPIO_PIN_8
#define I2C1_SCL_BARO_GPIO_Port GPIOB
#define I2C1_SDA_BARO_Pin GPIO_PIN_9
#define I2C1_SDA_BARO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
