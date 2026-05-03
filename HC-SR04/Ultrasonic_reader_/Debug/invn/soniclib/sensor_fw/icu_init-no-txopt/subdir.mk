################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.c \
../invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.c \
../invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.c 

OBJS += \
./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.o \
./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.o \
./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.o 

C_DEPS += \
./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.d \
./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.d \
./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.d 


# Each subdirectory must supply rules for building sources it contributes
invn/soniclib/sensor_fw/icu_init-no-txopt/%.o invn/soniclib/sensor_fw/icu_init-no-txopt/%.su invn/soniclib/sensor_fw/icu_init-no-txopt/%.cyclo: ../invn/soniclib/sensor_fw/icu_init-no-txopt/%.c invn/soniclib/sensor_fw/icu_init-no-txopt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"C:/Users/aleks/STM32CubeIDE/workspace_1.16.0/Ultrasonic_reader/invn" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -I../invn -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_init-2d-no-2d-txopt

clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_init-2d-no-2d-txopt:
	-$(RM) ./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.cyclo ./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.d ./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.o ./invn/soniclib/sensor_fw/icu_init-no-txopt/_icu_init-no-txopt_plugin_version.su ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.cyclo ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.d ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.o ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.su ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.cyclo ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.d ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.o ./invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt_fw.su

.PHONY: clean-invn-2f-soniclib-2f-sensor_fw-2f-icu_init-2d-no-2d-txopt

