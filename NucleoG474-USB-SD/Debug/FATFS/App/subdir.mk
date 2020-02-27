################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/App/app_fatfs.c 

OBJS += \
./FATFS/App/app_fatfs.o 

C_DEPS += \
./FATFS/App/app_fatfs.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/App/app_fatfs.o: ../FATFS/App/app_fatfs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -c -I../FATFS/App -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -I../Core/Inc -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.2.0/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.2.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.2.0/Middlewares/Third_Party/FatFs/src -I../FATFS/Target -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.2.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.2.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/App/app_fatfs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

