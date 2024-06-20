/*
 * com_debug.h
 *
 *  Created on: Jun 19, 2024
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;

void myDebug(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint32_t us);

#endif /* INC_COM_DEBUG_H_ */
