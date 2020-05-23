#pragma once

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void TIM2_IRQHandler(void);

void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void USART2_IRQHandler(void);

void Error_Handler(void);
void Print_Error(char* sErrorMessage, int msgLen);


extern uint8_t sMessage[255];

#ifdef __cplusplus
}
#endif

