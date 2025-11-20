# STM32F103C8T6-FreeRTOS-Project-Atollic-TrueSTUDIO-No-HAL-
This repository contains a minimal and clean FreeRTOS project for the STM32F103C8T6 (Blue Pill) board, built entirely in Atollic TrueSTUDIO without using STM32CubeMX or the STM32 HAL library. The project is designed for developers who prefer bare-metal control, lightweight code, and a fully manual project structure. Stack Size is smaller because of the small size of RAM in the MCU. tasks are in separate files to keep the project more organized. 


# What This Project Demonstrates
Creating a FreeRTOS project manually in TrueSTUDIO
Configuring FreeRTOSConfig.h for STM32F103C8T6
Using vTaskDelay() and tick interrupts from SysTick
Avoiding common linker/startup issues like:
undefined reference to vTaskDelay
multiple definition of g_pfnVectors
separate files for tasks


# How to Build
Clone the repo
Open the workspace in Atollic TrueSTUDIO
browse and add the folder locations in Project->Build Settings . in C/C++ Build->Settings  select Tool Settings. Go to C Compiler->Directories . it should look like below. 
<img width="824" height="613" alt="image" src="https://github.com/user-attachments/assets/b8afbdb6-f35b-4aee-a455-ba33c5a6e5a9" />
if hex file is required go to  C/C++ Build->Settings  select Tool Settings. Go to Others->Output Format . tick the Convert Build Output. it should look like below: 
<img width="819" height="649" alt="image" src="https://github.com/user-attachments/assets/1e290493-01bc-4bdb-a06c-e6612ca9d1bc" />
Clean and Build the project (Ctrl + B)
Flash to Blue Pill using ST-Link
