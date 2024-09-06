################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Fusion/FusionAhrs.c \
../Drivers/Fusion/FusionCompass.c \
../Drivers/Fusion/FusionOffset.c 

OBJS += \
./Drivers/Fusion/FusionAhrs.o \
./Drivers/Fusion/FusionCompass.o \
./Drivers/Fusion/FusionOffset.o 

C_DEPS += \
./Drivers/Fusion/FusionAhrs.d \
./Drivers/Fusion/FusionCompass.d \
./Drivers/Fusion/FusionOffset.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Fusion/%.o Drivers/Fusion/%.su Drivers/Fusion/%.cyclo: ../Drivers/Fusion/%.c Drivers/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Frab/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/0 GITHUB/Inverted-Short-Baseline-Acoustic-Positioning-System/STM32H723_Receiver/Drivers/lis3mdl-pid" -I"C:/Users/Frab/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/0 GITHUB/Inverted-Short-Baseline-Acoustic-Positioning-System/STM32H723_Receiver/Drivers/lsm6dsox-pid" -I"C:/Users/Frab/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/0 GITHUB/Inverted-Short-Baseline-Acoustic-Positioning-System/STM32H723_Receiver/Drivers/Fusion" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Fusion

clean-Drivers-2f-Fusion:
	-$(RM) ./Drivers/Fusion/FusionAhrs.cyclo ./Drivers/Fusion/FusionAhrs.d ./Drivers/Fusion/FusionAhrs.o ./Drivers/Fusion/FusionAhrs.su ./Drivers/Fusion/FusionCompass.cyclo ./Drivers/Fusion/FusionCompass.d ./Drivers/Fusion/FusionCompass.o ./Drivers/Fusion/FusionCompass.su ./Drivers/Fusion/FusionOffset.cyclo ./Drivers/Fusion/FusionOffset.d ./Drivers/Fusion/FusionOffset.o ./Drivers/Fusion/FusionOffset.su

.PHONY: clean-Drivers-2f-Fusion

