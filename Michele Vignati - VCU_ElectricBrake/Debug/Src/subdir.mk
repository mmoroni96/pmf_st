################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/CanRead.c \
../Src/CanWrite.c \
../Src/CanWriteBack.c \
../Src/Controllo.c \
../Src/CtrlSpeed.c \
../Src/FILTERS.c \
../Src/READ_SENSORS.c \
../Src/adc.c \
../Src/can.c \
../Src/gpio.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/CanRead.o \
./Src/CanWrite.o \
./Src/CanWriteBack.o \
./Src/Controllo.o \
./Src/CtrlSpeed.o \
./Src/FILTERS.o \
./Src/READ_SENSORS.o \
./Src/adc.o \
./Src/can.o \
./Src/gpio.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/CanRead.d \
./Src/CanWrite.d \
./Src/CanWriteBack.d \
./Src/Controllo.d \
./Src/CtrlSpeed.d \
./Src/FILTERS.d \
./Src/READ_SENSORS.d \
./Src/adc.d \
./Src/can.d \
./Src/gpio.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F429xx -I"C:/Users/Gabriele Canonico/Politecnico di Milano/Michele Vignati - VCU_ElectricBrake/Inc" -I"C:/Users/Gabriele Canonico/Politecnico di Milano/Michele Vignati - VCU_ElectricBrake/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Gabriele Canonico/Politecnico di Milano/Michele Vignati - VCU_ElectricBrake/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gabriele Canonico/Politecnico di Milano/Michele Vignati - VCU_ElectricBrake/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Gabriele Canonico/Politecnico di Milano/Michele Vignati - VCU_ElectricBrake/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


