################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Target/usbd_conf.c 

OBJS += \
./USB_DEVICE/Target/usbd_conf.o 

C_DEPS += \
./USB_DEVICE/Target/usbd_conf.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Target/usbd_conf.o: ../USB_DEVICE/Target/usbd_conf.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../USB_DEVICE/App -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../FATFS/Target -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32H7xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"USB_DEVICE/Target/usbd_conf.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

