/*
 * buffer.c
 *
 *  Created on: Jul 29, 2024
 *      Author: sausage
 */
#include "buffer.h"

char circularBuffer[BUFFER_SIZE];
uint8_t bufferReadIndex = 0;
uint8_t bufferWriteIndex = 0;
uint8_t bufferItemsLength = 0; //this changes depending how many items are in the buffer;


void gpsBuffer_WriteStreamDataChunkToCircularBuffer(uint8_t *chunk){

    if (bufferItemsLength == BUFFER_SIZE){
        //printf("Buffer is full. Try again shortly.\r\n");
        return;
    } else {
        //printf("writeStreamDataChunkToCircularBuffer\r\n");
    }

    int c;
    for (c=0; c<BUFFER_CHUNK_SIZE_IN_BYTES; c++){

        //circularBuffer[bufferWriteIndex] = fakeStreamData[fakeBufferIndex];
        circularBuffer[bufferWriteIndex] = chunk[c];
        bufferItemsLength++;
        bufferWriteIndex++;

        if (bufferWriteIndex == BUFFER_SIZE){
            bufferWriteIndex = 0;
        }

//        fakeBufferIndex++;
//        if (fakeBufferIndex >= 140) //fake continued loading of data.
//            fakeBufferIndex = 0;

        //printf("bufferItemsLength %d bufferWriteIndex %d  \r\n", bufferItemsLength, bufferWriteIndex);
    }


    //printCircularBuffer();

}

//ensure passed in outputChunk as output is a 128 char buffer
void gpsBuffer_ReadChunkFromCircularBuffer(uint8_t *outputChunk){

    if (bufferItemsLength == 0){
        //printf("Buffer is empty. Try again shorty.\r\n");
        return;
    } else {
        //printf("readChunkFromCircularBuffer\r\n");
    }

    int c;
    for (c=0; c<128; c++){
        //printf("%c", fakeStreamData[fakeBufferIndex]);

        if (bufferItemsLength > 0){
            char bufferValue = circularBuffer[bufferReadIndex];
            outputChunk[c] = bufferValue;
            //printf("%c", bufferValue);
            circularBuffer[bufferReadIndex] = '-';

            bufferItemsLength--;
            bufferReadIndex++;
            if (bufferReadIndex == BUFFER_SIZE){
                bufferReadIndex = 0;
            }
        } else {
            //bufferReadIndex = 0; //is this right?
        }

    }

    //printf("\r\n");
    //printCircularBuffer();
}

/* This is a NMEA line */
void gpsBuffer_ExtractDateTimeFromBuffer(uint8_t *chunk){ //13 char output buffer 12+EOF (HHMMSSDDMMYY) for the moment
	char sTime[6];
	char sDate[6];

	char *tokenLineState;

	char* lineToken = strtok_r(circularBuffer, "\r\n", &tokenLineState);

	while (lineToken != NULL){

		int tokenPos = 0;
		int timeOK = 0;
		int dateOK = 0;

		char* token = strsep(&lineToken, ",");

		while (token != NULL){

			if (strcmp(token, "$GPRMC") == 0){
				token = strsep(&lineToken, ",");
				tokenPos++;
				while (token != NULL){
					if (tokenPos == 1 && strlen(token) == 9 ){
						strcpy(sTime, token);
						timeOK = 1;
					}
					if (tokenPos == 9 && strlen(token) == 6){
						strcpy(sDate, token);
						dateOK = 1;
					}
					token = strsep(&lineToken, ",");
					tokenPos++;
				}

				if (timeOK == 1 && dateOK == 1){
					sprintf(chunk, "%.6s%.6s", sTime, sDate);
				}

				token = NULL; //or break while?
				lineToken = NULL;
				return;
			} else {
				token = strsep(&lineToken, ",");
			}



		}

		lineToken = strtok_r(NULL, "\r\n", &tokenLineState);

	}

}

char* gpsBuffer_GetCircularBuffer(){
	return circularBuffer;
}

void gpsBuffer_PrintCircularBuffer(){
    int c;
    for (c=0; c<BUFFER_SIZE; c++){
        printf("%c", circularBuffer[c]);
    }
}
