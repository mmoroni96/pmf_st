################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/diskio.c \
C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/ff.c \
C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/FatFs/diskio.o \
./Middlewares/FatFs/ff.o \
./Middlewares/FatFs/ff_gen_drv.o \
./Middlewares/FatFs/syscall.o 

C_DEPS += \
./Middlewares/FatFs/diskio.d \
./Middlewares/FatFs/ff.d \
./Middlewares/FatFs/ff_gen_drv.d \
./Middlewares/FatFs/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/diskio.o: C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/diskio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff.o: C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/ff.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/ff.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff_gen_drv.o: C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/ff_gen_drv.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/syscall.o: C:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src/option/syscall.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/syscall.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

