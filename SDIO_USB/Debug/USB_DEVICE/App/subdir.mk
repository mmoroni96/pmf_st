################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/App/usb_device.c \
../USB_DEVICE/App/usbd_desc.c \
../USB_DEVICE/App/usbd_storage_if.c 

OBJS += \
./USB_DEVICE/App/usb_device.o \
./USB_DEVICE/App/usbd_desc.o \
./USB_DEVICE/App/usbd_storage_if.o 

C_DEPS += \
./USB_DEVICE/App/usb_device.d \
./USB_DEVICE/App/usbd_desc.d \
./USB_DEVICE/App/usbd_storage_if.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/App/usb_device.o: ../USB_DEVICE/App/usb_device.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"USB_DEVICE/App/usb_device.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
USB_DEVICE/App/usbd_desc.o: ../USB_DEVICE/App/usbd_desc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"USB_DEVICE/App/usbd_desc.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
USB_DEVICE/App/usbd_storage_if.o: ../USB_DEVICE/App/usbd_storage_if.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../FATFS/App -I../USB_DEVICE/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/Third_Party/FatFs/src -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../USB_DEVICE/App -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../FATFS/Target -IC:/Users/mmoro/STM32Cube/Repository/STM32Cube_FW_H7_V1.6.0/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"USB_DEVICE/App/usbd_storage_if.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

