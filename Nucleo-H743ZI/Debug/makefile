################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
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
CANDELA_STRUM_OK.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CANDELA_STRUM_OK.list \

OBJCOPY_BIN += \
CANDELA_STRUM_OK.bin \


# All Target
all: CANDELA_STRUM_OK.elf secondary-outputs

# Tool invocations
CANDELA_STRUM_OK.elf: $(OBJS) $(USER_OBJS) C:\Users\mmoro\OneDrive\Documents\pmf_st\Nucleo-H743ZI\STM32H743ZITX_FLASH.ld
	arm-none-eabi-gcc -o "CANDELA_STRUM_OK.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\mmoro\OneDrive\Documents\pmf_st\Nucleo-H743ZI\STM32H743ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CANDELA_STRUM_OK.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CANDELA_STRUM_OK.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CANDELA_STRUM_OK.list"
	@echo 'Finished building: $@'
	@echo ' '

CANDELA_STRUM_OK.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CANDELA_STRUM_OK.bin"
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
