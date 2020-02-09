################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
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
integrazioniponti.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
integrazioniponti.list \


# All Target
all: integrazioniponti.elf secondary-outputs

# Tool invocations
integrazioniponti.elf: $(OBJS) $(USER_OBJS) C:\Users\mmoro\OneDrive\Documents\pmf_st\integrazioniponti\STM32F042C6UX_FLASH.ld
	arm-none-eabi-gcc -o "integrazioniponti.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\mmoro\OneDrive\Documents\pmf_st\integrazioniponti\STM32F042C6UX_FLASH.ld" --specs=nosys.specs -Wl,-Map="integrazioniponti.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

integrazioniponti.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "integrazioniponti.list"
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
