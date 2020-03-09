################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/FatFs/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/subdir.mk
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
MainBoard-Test_Sensori.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MainBoard-Test_Sensori.list \

OBJCOPY_BIN += \
MainBoard-Test_Sensori.bin \


# All Target
all: MainBoard-Test_Sensori.elf secondary-outputs

# Tool invocations
MainBoard-Test_Sensori.elf: $(OBJS) $(USER_OBJS) C:\Users\mmoro\OneDrive\Documents\pmf_st\MainBoard-Test_Sensori\STM32G474RETX_FLASH.ld
	arm-none-eabi-gcc -o "MainBoard-Test_Sensori.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\mmoro\OneDrive\Documents\pmf_st\MainBoard-Test_Sensori\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MainBoard-Test_Sensori.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MainBoard-Test_Sensori.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MainBoard-Test_Sensori.list"
	@echo 'Finished building: $@'
	@echo ' '

MainBoard-Test_Sensori.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MainBoard-Test_Sensori.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
