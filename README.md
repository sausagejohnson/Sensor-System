# Sensor System Project

This is general sensor system developed as a proof of concept. It is based on the STM32 and FreeRTOS. The project aims to implement battery management, sensor reading, logging, commands and LoRa communication.

Code is in C.

***

## Building

The project is developed under STM32CubeIDE.

Libraries used FATFS, FreeRTOS and FATFS_SD SPI (by Khaled Magdy).


## Hardware, Modules and Wiring

Development: Nucleo STM32F303RE

Modules currently connected:

 - TF Card Memory Shield Module (SPI)
 - GY-NEO6MV2 NEO-6M GPS Module (RX)
 - Red Generic Turbidity Module (Analogue)

The wiring is currently documented in the code.


## More details
This project is developed on Mondays! If you want to get in touch, find me at https://waynejohnson.net

***