################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.c \
../invn/soniclib/sensor_fw/icu_gpt/icu_gpt.c \
../invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.c 

OBJS += \
./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.o \
./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.o \
./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.o 

C_DEPS += \
./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.d \
./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.d \
./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.d 


# Each subdirectory must supply rules for building sources it contributes
invn/soniclib/sensor_fw/icu_gpt/%.o invn/soniclib/sensor_fw/icu_gpt/%.su invn/soniclib/sensor_fw/icu_gpt/%.cyclo: ../invn/soniclib/sensor_fw/icu_gpt/%.c invn/soniclib/sensor_fw/icu_gpt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"C:/Users/aleks/STM32CubeIDE/workspace_1.16.0/Ultrasonic_reader/invn" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -I../invn -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt

clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt:
	-$(RM) ./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.cyclo ./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.d ./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.o ./invn/soniclib/sensor_fw/icu_gpt/_icu_gpt_plugin_version.su ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.cyclo ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.d ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.o ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt.su ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.cyclo ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.d ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.o ./invn/soniclib/sensor_fw/icu_gpt/icu_gpt_fw.su

.PHONY: clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_gpt

