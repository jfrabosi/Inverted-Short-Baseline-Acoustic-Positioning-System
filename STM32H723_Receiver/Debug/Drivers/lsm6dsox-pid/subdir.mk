################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lsm6dsox-pid/lsm6dsox_reg.c 

OBJS += \
./Drivers/lsm6dsox-pid/lsm6dsox_reg.o 

C_DEPS += \
./Drivers/lsm6dsox-pid/lsm6dsox_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lsm6dsox-pid/%.o Drivers/lsm6dsox-pid/%.su Drivers/lsm6dsox-pid/%.cyclo: ../Drivers/lsm6dsox-pid/%.c Drivers/lsm6dsox-pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/frabb/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/000Thesis/STM32H723_Receiver/Drivers/lis3mdl-pid" -I"C:/Users/frabb/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/000Thesis/STM32H723_Receiver/Drivers/lsm6dsox-pid" -I"C:/Users/frabb/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/000Thesis/STM32H723_Receiver/Drivers/Fusion" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lsm6dsox-2d-pid

clean-Drivers-2f-lsm6dsox-2d-pid:
	-$(RM) ./Drivers/lsm6dsox-pid/lsm6dsox_reg.cyclo ./Drivers/lsm6dsox-pid/lsm6dsox_reg.d ./Drivers/lsm6dsox-pid/lsm6dsox_reg.o ./Drivers/lsm6dsox-pid/lsm6dsox_reg.su

.PHONY: clean-Drivers-2f-lsm6dsox-2d-pid

