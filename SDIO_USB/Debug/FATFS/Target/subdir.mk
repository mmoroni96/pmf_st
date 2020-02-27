################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/Target/bsp_driver_sd.c \
../FATFS/Target/sd_diskio.c 

OBJS += \
./FATFS/Target/bsp_driver_sd.o \
./FATFS/Target/sd_diskio.o 

C_DEPS += \
./FATFS/Target/bsp_driver_sd.d \
./FATFS/Target/sd_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/Target/bsp_driver_sd.o: ../FATFS/Target/bsp_driver_sd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/bsp_driver_sd.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
FATFS/Target/sd_diskio.o: ../FATFS/Target/sd_diskio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/sd_diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

