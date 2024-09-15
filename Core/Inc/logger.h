#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"

typedef struct logData_t {
	char 		dateTime[30];
	char  		typeCode;
	uint16_t 	value;
} logData_t;

extern USART_HandleTypeDef husart2;

void logger_log(logData_t data);

void logger_writeToSD(logData_t data);

void logger_cleanUp();

#endif /* LOGGER_H_ */
