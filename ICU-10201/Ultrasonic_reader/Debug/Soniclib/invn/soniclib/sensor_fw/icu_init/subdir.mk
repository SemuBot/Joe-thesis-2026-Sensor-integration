################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.c \
../Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.c \
../Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.c 

OBJS += \
./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.o \
./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.o \
./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.o 

C_DEPS += \
./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.d \
./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.d \
./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.d 


# Each subdirectory must supply rules for building sources it contributes
Soniclib/invn/soniclib/sensor_fw/icu_init/%.o Soniclib/invn/soniclib/sensor_fw/icu_init/%.su Soniclib/invn/soniclib/sensor_fw/icu_init/%.cyclo: ../Soniclib/invn/soniclib/sensor_fw/icu_init/%.c Soniclib/invn/soniclib/sensor_fw/icu_init/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Soniclib -I../bsp -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_init

clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_init:
	-$(RM) ./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.d ./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.o ./Soniclib/invn/soniclib/sensor_fw/icu_init/_icu_init_plugin_version.su ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.d ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.o ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init.su ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.cyclo ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.d ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.o ./Soniclib/invn/soniclib/sensor_fw/icu_init/icu_init_fw.su

.PHONY: clean-Soniclib-2f-invn-2f-soniclib-2f-sensor_fw-2f-icu_init

