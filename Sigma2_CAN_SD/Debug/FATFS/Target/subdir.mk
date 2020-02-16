################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/Target/my_sd.c \
../FATFS/Target/stm32g4xx_nucleo_bus.c \
../FATFS/Target/user_diskio.c 

OBJS += \
./FATFS/Target/my_sd.o \
./FATFS/Target/stm32g4xx_nucleo_bus.o \
./FATFS/Target/user_diskio.o 

C_DEPS += \
./FATFS/Target/my_sd.d \
./FATFS/Target/stm32g4xx_nucleo_bus.d \
./FATFS/Target/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/Target/my_sd.o: ../FATFS/Target/my_sd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -c -I../FATFS/App -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/my_sd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
FATFS/Target/stm32g4xx_nucleo_bus.o: ../FATFS/Target/stm32g4xx_nucleo_bus.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -c -I../FATFS/App -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/stm32g4xx_nucleo_bus.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
FATFS/Target/user_diskio.o: ../FATFS/Target/user_diskio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -c -I../FATFS/App -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/STM32G4xx_HAL_Driver/Inc -I../Core/Inc -I../FATFS/Target -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Drivers/CMSIS/Include -IC:/Users/galla/STM32Cube/Repository/STM32Cube_FW_G4_V1.1.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/user_diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

