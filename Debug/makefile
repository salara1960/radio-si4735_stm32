################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/si4735/subdir.mk
-include Src/oled/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: stm32_ips.elf

# Tool invocations
stm32_ips.elf: $(OBJS) $(USER_OBJS) ../STM32F103C8Tx_FLASH.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -specs=nano.specs -T"../STM32F103C8Tx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "stm32_ips.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "stm32_ips.elf" "stm32_ips.hex"
	arm-none-eabi-objcopy -O binary "stm32_ips.elf" "stm32_ips.bin"
	ls -la | grep "stm32_ips.*"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
