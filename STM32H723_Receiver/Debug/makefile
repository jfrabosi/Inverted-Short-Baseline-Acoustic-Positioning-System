################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/lsm6dsox-pid/subdir.mk
-include Drivers/lis3mdl-pid/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Drivers/Fusion/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STM32H723_Receiver
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32H723_Receiver.elf \

MAP_FILES += \
STM32H723_Receiver.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32H723_Receiver.list \


# All Target
all: main-build

# Main-build Target
main-build: STM32H723_Receiver.elf secondary-outputs

# Tool invocations
STM32H723_Receiver.elf STM32H723_Receiver.map: $(OBJS) $(USER_OBJS) C:\Users\frabb\OneDrive\ -\ Cal\ Poly\Documents\ (Cloud)\0\ CALPOLY\000Thesis\STM32H723_Receiver\STM32H723ZGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32H723_Receiver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\frabb\OneDrive - Cal Poly\Documents (Cloud)\0 CALPOLY\000Thesis\STM32H723_Receiver\STM32H723ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32H723_Receiver.map" -Wl,--gc-sections -static -L"C:\Users\frabb\OneDrive - Cal Poly\Documents (Cloud)\0 CALPOLY\000Thesis\STM32H723_Receiver\Drivers\CMSIS\Lib" -L"C:\Users\frabb\OneDrive - Cal Poly\Documents (Cloud)\0 CALPOLY\000Thesis\STM32H723_Receiver\Drivers\lis3mdl-pid" -L"C:\Users\frabb\OneDrive - Cal Poly\Documents (Cloud)\0 CALPOLY\000Thesis\STM32H723_Receiver\Drivers\lsm6dsox-pid" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32H723_Receiver.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32H723_Receiver.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32H723_Receiver.elf STM32H723_Receiver.list STM32H723_Receiver.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
