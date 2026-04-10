################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Soniclib/invn/soniclib/extra/ch_extra_display_utils.c 

OBJS += \
./Soniclib/invn/soniclib/extra/ch_extra_display_utils.o 

C_DEPS += \
./Soniclib/invn/soniclib/extra/ch_extra_display_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Soniclib/invn/soniclib/extra/%.o Soniclib/invn/soniclib/extra/%.su Soniclib/invn/soniclib/extra/%.cyclo: ../Soniclib/invn/soniclib/extra/%.c Soniclib/invn/soniclib/extra/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Soniclib -I../bsp -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Soniclib-2f-invn-2f-soniclib-2f-extra

clean-Soniclib-2f-invn-2f-soniclib-2f-extra:
	-$(RM) ./Soniclib/invn/soniclib/extra/ch_extra_display_utils.cyclo ./Soniclib/invn/soniclib/extra/ch_extra_display_utils.d ./Soniclib/invn/soniclib/extra/ch_extra_display_utils.o ./Soniclib/invn/soniclib/extra/ch_extra_display_utils.su

.PHONY: clean-Soniclib-2f-invn-2f-soniclib-2f-extra

