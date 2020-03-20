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
SIMULATORE_CANDELA_STRUM.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SIMULATORE_CANDELA_STRUM.list \

OBJCOPY_BIN += \
SIMULATORE_CANDELA_STRUM.bin \


# All Target
all: SIMULATORE_CANDELA_STRUM.elf secondary-outputs

# Tool invocations
SIMULATORE_CANDELA_STRUM.elf: $(OBJS) $(USER_OBJS) C:\Users\mmoro\OneDrive\Desktop\SIMULATORE_CANDELA_STRUM\STM32G474RETX_FLASH.ld
	arm-none-eabi-gcc -o "SIMULATORE_CANDELA_STRUM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\mmoro\OneDrive\Desktop\SIMULATORE_CANDELA_STRUM\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SIMULATORE_CANDELA_STRUM.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SIMULATORE_CANDELA_STRUM.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SIMULATORE_CANDELA_STRUM.list"
	@echo 'Finished building: $@'
	@echo ' '

SIMULATORE_CANDELA_STRUM.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SIMULATORE_CANDELA_STRUM.bin"
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
