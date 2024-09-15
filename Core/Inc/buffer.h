/*
 * buffer.h
 *
 *  Created on: Jul 29, 2024
 *      Author: sausage
 */

#ifndef INC_BUFFER_H_
#define INC_BUFFER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DELAY   1000

#define BUFFER_SIZE 256
#define BUFFER_CHUNK_SIZE_IN_BYTES 16 //bytes each
extern char circularBuffer[BUFFER_SIZE];

extern uint8_t bufferReadIndex;
extern uint8_t bufferWriteIndex;
extern uint8_t bufferItemsLength; //this changes depending how many items are in the buffer;

void gpsBuffer_PrintCircularBuffer();

void gpsBuffer_ReadChunkFromCircularBuffer();

void gpsBuffer_WriteStreamDataChunkToCircularBuffer(uint8_t *chunk);

void gpsBuffer_ExtractTimeFromBuffer(uint8_t *chunk); //12+eof char output buffer for the moment
void gpsBuffer_ExtractDateTimeFromBuffer(uint8_t *chunk); //12+eof char output buffer for the moment

char* gpsBuffer_GetCircularBuffer();

#endif /* INC_BUFFER_H_ */
