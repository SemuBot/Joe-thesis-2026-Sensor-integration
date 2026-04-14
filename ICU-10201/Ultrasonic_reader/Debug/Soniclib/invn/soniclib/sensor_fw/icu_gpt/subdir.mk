################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.c \
../Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.c \
../Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.c 

OBJS += \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.o \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.o \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.o 

C_DEPS += \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.d \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.d \
./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.d 


# Each subdirectory must supply rules for building sources it contributes
Soniclib/invn/soniclib/sensor_fw/icu_gpt/%.o Soniclib/invn/soniclib/sensor_fw/icu_gpt/%.su Soniclib/invn/soniclib/sensor_fw/icu_gpt/%.cyclo: ../Soniclib/invn/soniclib/sensor_fw/icu_gpt/%.c Soniclib/invn/soniclib/sensor_fw/icu_gpt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Soniclib -I../bsp -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt

clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt:
	-$(RM) ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.d ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.o ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.su ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.d ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.o ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt.su ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.d ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.o ./Soniclib/invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.su

.PHONY: clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt

