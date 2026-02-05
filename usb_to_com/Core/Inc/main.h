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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart2;
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
#define O13_Duphong4_Pin GPIO_PIN_2
#define O13_Duphong4_GPIO_Port GPIOE
#define O14_Duphong5_Pin GPIO_PIN_3
#define O14_Duphong5_GPIO_Port GPIOE
#define O15_Duphong6_Pin GPIO_PIN_4
#define O15_Duphong6_GPIO_Port GPIOE
#define O16_Duphong7_Pin GPIO_PIN_5
#define O16_Duphong7_GPIO_Port GPIOE
#define O17_Duphong8_Pin GPIO_PIN_6
#define O17_Duphong8_GPIO_Port GPIOE
#define O18_Duphong9_Pin GPIO_PIN_13
#define O18_Duphong9_GPIO_Port GPIOC
#define I4_E_stop_Pin GPIO_PIN_3
#define I4_E_stop_GPIO_Port GPIOC
#define I5_STOP_Pin GPIO_PIN_4
#define I5_STOP_GPIO_Port GPIOC
#define I6_START_Pin GPIO_PIN_5
#define I6_START_GPIO_Port GPIOC
#define I7_RESTART_Pin GPIO_PIN_0
#define I7_RESTART_GPIO_Port GPIOB
#define I8_VACUM1_Pin GPIO_PIN_1
#define I8_VACUM1_GPIO_Port GPIOB
#define I9_VACUM2_Pin GPIO_PIN_7
#define I9_VACUM2_GPIO_Port GPIOE
#define I_10Left_DOOR_Pin GPIO_PIN_8
#define I_10Left_DOOR_GPIO_Port GPIOE
#define I_11_RIGHT_DOOR_Pin GPIO_PIN_9
#define I_11_RIGHT_DOOR_GPIO_Port GPIOE
#define I12_AP_SUAT_Pin GPIO_PIN_10
#define I12_AP_SUAT_GPIO_Port GPIOE
#define I13_DU_PHONG_Pin GPIO_PIN_11
#define I13_DU_PHONG_GPIO_Port GPIOE
#define I14_DU_PHONG2_Pin GPIO_PIN_12
#define I14_DU_PHONG2_GPIO_Port GPIOE
#define I15_DU_PHONG3_Pin GPIO_PIN_13
#define I15_DU_PHONG3_GPIO_Port GPIOE
#define I16_DU_PHONG4_Pin GPIO_PIN_14
#define I16_DU_PHONG4_GPIO_Port GPIOE
#define I17_DU_PHONG5_Pin GPIO_PIN_15
#define I17_DU_PHONG5_GPIO_Port GPIOE
#define I18_DU_PHONG6_Pin GPIO_PIN_10
#define I18_DU_PHONG6_GPIO_Port GPIOB
#define DIR_TIM3_Pin GPIO_PIN_7
#define DIR_TIM3_GPIO_Port GPIOC
#define DIR_TIM8_Pin GPIO_PIN_9
#define DIR_TIM8_GPIO_Port GPIOC
#define DIR_TIM1_Pin GPIO_PIN_9
#define DIR_TIM1_GPIO_Port GPIOA
#define DIR_NO_Pin GPIO_PIN_15
#define DIR_NO_GPIO_Port GPIOA
#define O1_xilanh1_Pin GPIO_PIN_3
#define O1_xilanh1_GPIO_Port GPIOD
#define O2_xilanh2_Pin GPIO_PIN_4
#define O2_xilanh2_GPIO_Port GPIOD
#define O3_vacum_hut1_Pin GPIO_PIN_5
#define O3_vacum_hut1_GPIO_Port GPIOD
#define O4_vacum_hut2_Pin GPIO_PIN_6
#define O4_vacum_hut2_GPIO_Port GPIOD
#define O5_vacum1_nha_Pin GPIO_PIN_7
#define O5_vacum1_nha_GPIO_Port GPIOD
#define O6_vacum2_nha_Pin GPIO_PIN_3
#define O6_vacum2_nha_GPIO_Port GPIOB
#define O7_Den_Xanh_Pin GPIO_PIN_4
#define O7_Den_Xanh_GPIO_Port GPIOB
#define O8_Den_Do_Pin GPIO_PIN_5
#define O8_Den_Do_GPIO_Port GPIOB
#define O9_Coi_Pin GPIO_PIN_8
#define O9_Coi_GPIO_Port GPIOB
#define O10_Duphong1_Pin GPIO_PIN_9
#define O10_Duphong1_GPIO_Port GPIOB
#define O11_Duphong2_Pin GPIO_PIN_0
#define O11_Duphong2_GPIO_Port GPIOE
#define O12_Duphong3_Pin GPIO_PIN_1
#define O12_Duphong3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
