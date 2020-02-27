################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/USB_Device_Library/subdir.mk
-include Middlewares/FatFs/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/subdir.mk
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
SDIO_USB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SDIO_USB.list \

OBJCOPY_BIN += \
SDIO_USB.bin \


# All Target
all: SDIO_USB.elf secondary-outputs

# Tool invocations
SDIO_USB.elf: $(OBJS) $(USER_OBJS) C:\Users\mmoro\OneDrive\Documents\pmf_st\SDIO_USB\STM32H743ZITX_FLASH.ld
	arm-none-eabi-gcc -o "SDIO_USB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\mmoro\OneDrive\Documents\pmf_st\SDIO_USB\STM32H743ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SDIO_USB.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SDIO_USB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SDIO_USB.list"
	@echo 'Finished building: $@'
	@echo ' '

SDIO_USB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SDIO_USB.bin"
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
