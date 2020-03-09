################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Analog_Conversions.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Analog_Conversions.list \


# All Target
all: Analog_Conversions.elf secondary-outputs

# Tool invocations
Analog_Conversions.elf: $(OBJS) $(USER_OBJS) C:\Users\Javier\STM32CubeIDE\workspace_1.1.0\Analog_Conversions\STM32G474RETX_FLASH.ld
	arm-none-eabi-gcc -o "Analog_Conversions.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Javier\STM32CubeIDE\workspace_1.1.0\Analog_Conversions\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Analog_Conversions.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Analog_Conversions.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Analog_Conversions.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
