################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/Src/measurement_config.c \
../Common/Src/obstacle_detection.c \
../Common/Src/sensor.c 

OBJS += \
./Common/Src/measurement_config.o \
./Common/Src/obstacle_detection.o \
./Common/Src/sensor.o 

C_DEPS += \
./Common/Src/measurement_config.d \
./Common/Src/obstacle_detection.d \
./Common/Src/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/%.o Common/Src/%.su Common/Src/%.cyclo: ../Common/Src/%.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Soniclib -I../bsp -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src

clean-Common-2f-Src:
	-$(RM) ./Common/Src/measurement_config.cyclo ./Common/Src/measurement_config.d ./Common/Src/measurement_config.o ./Common/Src/measurement_config.su ./Common/Src/obstacle_detection.cyclo ./Common/Src/obstacle_detection.d ./Common/Src/obstacle_detection.o ./Common/Src/obstacle_detection.su ./Common/Src/sensor.cyclo ./Common/Src/sensor.d ./Common/Src/sensor.o ./Common/Src/sensor.su

.PHONY: clean-Common-2f-Src

