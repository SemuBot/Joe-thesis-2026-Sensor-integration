################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../invn/soniclib/extra/ch_extra_display_utils.c 

OBJS += \
./invn/soniclib/extra/ch_extra_display_utils.o 

C_DEPS += \
./invn/soniclib/extra/ch_extra_display_utils.d 


# Each subdirectory must supply rules for building sources it contributes
invn/soniclib/extra/%.o invn/soniclib/extra/%.su invn/soniclib/extra/%.cyclo: ../invn/soniclib/extra/%.c invn/soniclib/extra/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"C:/Users/aleks/STM32CubeIDE/workspace_1.16.0/Ultrasonic_reader/invn" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -I../invn -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-invn-2f-soniclib-2f-extra

clean-invn-2f-soniclib-2f-extra:
	-$(RM) ./invn/soniclib/extra/ch_extra_display_utils.cyclo ./invn/soniclib/extra/ch_extra_display_utils.d ./invn/soniclib/extra/ch_extra_display_utils.o ./invn/soniclib/extra/ch_extra_display_utils.su

.PHONY: clean-invn-2f-soniclib-2f-extra

