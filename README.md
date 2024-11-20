# Sensor System Project

This is a general Sensor System developed as a proof of concept. It is based on the STM32 and FreeRTOS. The project aims to implement battery management, sensor reading, logging, commands and LoRa communication.

There is a companion project, the Sensor System Gateway https://github.com/sausagejohnson/Sensor-System-Gateway which is a USB-based serial terminal that communicates with, and controls this Sensor System.

Code is in C.

***

## Building

The project is developed under STM32CubeIDE.

Libraries used FATFS, FreeRTOS, Tiny LoRa for STM32 HAL (by belyalov) and FATFS_SD SPI (by Khaled Magdy).


## Hardware, Modules and Wiring

Development: Nucleo STM32F303RE

Modules currently connected:

 - TF Card Memory Shield Module (SPI)
 - LoRa RFM95 915Mhz 			(SPI)
 - GY-NEO6MV2 NEO-6M GPS Module (RX)
 - Red Generic Turbidity Module (Analogue)

The wiring is currently documented in the code.


## More details
This project is developed on Mondays! If you want to get in touch, find me at https://waynejohnson.net

***