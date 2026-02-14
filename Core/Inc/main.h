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

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin LL_GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOC
#define V_BLANK_Pin LL_GPIO_PIN_0
#define V_BLANK_GPIO_Port GPIOA
#define RGB_LSB_R_Pin LL_GPIO_PIN_2
#define RGB_LSB_R_GPIO_Port GPIOA
#define RGB_MSB_R_Pin LL_GPIO_PIN_3
#define RGB_MSB_R_GPIO_Port GPIOA
#define RGB_LSB_G_Pin LL_GPIO_PIN_4
#define RGB_LSB_G_GPIO_Port GPIOA
#define RGB_MSB_G_Pin LL_GPIO_PIN_5
#define RGB_MSB_G_GPIO_Port GPIOA
#define RGB_LSB_B_Pin LL_GPIO_PIN_6
#define RGB_LSB_B_GPIO_Port GPIOA
#define RGB_MSB_B_Pin LL_GPIO_PIN_7
#define RGB_MSB_B_GPIO_Port GPIOA
#define H_BLANK_Pin LL_GPIO_PIN_1
#define H_BLANK_GPIO_Port GPIOB
#define VSYNC_Pin LL_GPIO_PIN_10
#define VSYNC_GPIO_Port GPIOB
#define Sound_Pin LL_GPIO_PIN_12
#define Sound_GPIO_Port GPIOB
#define GPIO_PA10_Pin LL_GPIO_PIN_10
#define GPIO_PA10_GPIO_Port GPIOA
#define SPI1_CS_Pin LL_GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define HSYNC_Pin LL_GPIO_PIN_6
#define HSYNC_GPIO_Port GPIOB
#define VSYNCB8_Pin LL_GPIO_PIN_8
#define VSYNCB8_GPIO_Port GPIOB
#define TIM11_CH1_AUDIO_BCK_Pin LL_GPIO_PIN_9
#define TIM11_CH1_AUDIO_BCK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern void getTimes(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
