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
NucleoG474-USB-SD.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NucleoG474-USB-SD.list \

OBJCOPY_BIN += \
NucleoG474-USB-SD.bin \


# All Target
all: NucleoG474-USB-SD.elf secondary-outputs

# Tool invocations
NucleoG474-USB-SD.elf: $(OBJS) $(USER_OBJS) C:\Users\galla\OneDrive\ -\ Politecnico\ di\ Milano\PMF\ -\ Electric\Eagle\STM32\Sigma2_CAN_SD\STM32G474RETX_FLASH.ld
	arm-none-eabi-gcc -o "NucleoG474-USB-SD.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\galla\OneDrive - Politecnico di Milano\PMF - Electric\Eagle\STM32\Sigma2_CAN_SD\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="NucleoG474-USB-SD.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NucleoG474-USB-SD.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NucleoG474-USB-SD.list"
	@echo 'Finished building: $@'
	@echo ' '

NucleoG474-USB-SD.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NucleoG474-USB-SD.bin"
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
