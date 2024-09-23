#include "logger.h"
#include "fatfs.h"

#define LOG_MAX 4

logData_t sensorLog[LOG_MAX];
uint8_t logIndex = 0;

void logger_log(logData_t data){
	strcpy(sensorLog[logIndex].dateTime, data.dateTime);
	sensorLog[logIndex].typeCode = data.typeCode;
	sensorLog[logIndex].value = data.value;

	logger_writeToSD(data);

	logIndex++;
	if (logIndex >= LOG_MAX)
		logIndex = 0;
}

FATFS fatFS;
FIL loggerFile;
FRESULT fileResult;
FATFS *fsPtr;
char txBuffer[250];
char sdBuffer[250];
DWORD freeClusters;
uint32_t totalSize, freeSpace;

void logger_writeToSD(logData_t data){

	//------------------[ Mount The SD Card ]--------------------
	fileResult = f_mount(&fatFS, "", 1);
	if (fileResult != FR_OK)
	{
	  sprintf(txBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", fileResult);
	  HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	  return;
	}
	sprintf(txBuffer, "SD Card Mounted Successfully! \r\n\n");
	size_t length = strlen(txBuffer);
	HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, length, 10U);

	//------------------[ Get & Print The SD Card Size & Free Space ]--------------------
	f_getfree("", &freeClusters, &fsPtr);
	totalSize = (uint32_t)((fsPtr->n_fatent - 2) * fsPtr->csize * 0.5);
	freeSpace = (uint32_t)(freeClusters * fsPtr->csize * 0.5);
	sprintf(txBuffer, "Total SD Card Size: %lu Bytes\r\n", totalSize);
	HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	sprintf(txBuffer, "Free SD Card Space: %lu Bytes\r\n\n", freeSpace);
	HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);

	// look at adding FF_FS_LOCK for safety on this write, and even
	// locking on this function so that only one thing can
	// attempt to write. All others should be silently turned
	// away.

	//------------------[ Open A Text File For Write & Write Data ]--------------------
	//Open the file
	fileResult = f_open(&loggerFile, "log.txt", FA_WRITE | FA_OPEN_ALWAYS);
	if(fileResult != FR_OK)
	{
	  sprintf(txBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", fileResult);
	  HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	  return;
	}

	fileResult = f_lseek(&loggerFile, f_size(&loggerFile)); // Move The File Pointer To The EOF (End-Of-File)
	if(fileResult != FR_OK)
	{
	  sprintf(txBuffer, "Error! Couldn't move to the end of the file: (%i)\r\n", fileResult);
	  HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	  return;
	}

	sprintf(txBuffer, "Text File Created or Appended! Writing Data To The Text File..\r\n\n");
	HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);

	// (1) Write Data To The Text File [ Using f_puts() Function ]
	sprintf(sdBuffer, "%s\t%c\t%d\n", data.dateTime, data.typeCode, data.value);
	int charsWritten = f_puts(sdBuffer, &loggerFile);
	if(charsWritten == 0)
	{
	  sprintf(txBuffer, "Error! puts string into the file: (%i)\r\n", fileResult);
	  HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	  return;
	}
	// (2) Write Data To The Text File [ Using f_write() Function ]
	//		strcpy(sdBuffer, "Hello! From STM32 To SD Card Over SPI, Using f_write()\r\n");
	//		fileResult = f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
	//		if(fileResult != FR_OK)
	//		{
	//		  sprintf(txBuffer, "Error! f_write string into the file: (%i)\r\n", fileResult);
	//		  HAL_USART_Transmit(&husart2, (uint8_t *)txBuffer, strlen(txBuffer), 10U);
	//		  return;
	//		}

	// Close The File
	f_close(&loggerFile);
}
