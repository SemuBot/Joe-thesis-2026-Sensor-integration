################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Soniclib/invn/soniclib/ch_api.c \
../Soniclib/invn/soniclib/ch_common.c \
../Soniclib/invn/soniclib/ch_driver.c \
../Soniclib/invn/soniclib/ch_log.c \
../Soniclib/invn/soniclib/ch_math_utils.c \
../Soniclib/invn/soniclib/ch_rangefinder.c 

OBJS += \
./Soniclib/invn/soniclib/ch_api.o \
./Soniclib/invn/soniclib/ch_common.o \
./Soniclib/invn/soniclib/ch_driver.o \
./Soniclib/invn/soniclib/ch_log.o \
./Soniclib/invn/soniclib/ch_math_utils.o \
./Soniclib/invn/soniclib/ch_rangefinder.o 

C_DEPS += \
./Soniclib/invn/soniclib/ch_api.d \
./Soniclib/invn/soniclib/ch_common.d \
./Soniclib/invn/soniclib/ch_driver.d \
./Soniclib/invn/soniclib/ch_log.d \
./Soniclib/invn/soniclib/ch_math_utils.d \
./Soniclib/invn/soniclib/ch_rangefinder.d 


# Each subdirectory must supply rules for building sources it contributes
Soniclib/invn/soniclib/%.o Soniclib/invn/soniclib/%.su Soniclib/invn/soniclib/%.cyclo: ../Soniclib/invn/soniclib/%.c Soniclib/invn/soniclib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Soniclib -I../bsp -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Soniclib-2f-invn-2f-soniclib

clean-Soniclib-2f-invn-2f-soniclib:
	-$(RM) ./Soniclib/invn/soniclib/ch_api.cyclo ./Soniclib/invn/soniclib/ch_api.d ./Soniclib/invn/soniclib/ch_api.o ./Soniclib/invn/soniclib/ch_api.su ./Soniclib/invn/soniclib/ch_common.cyclo ./Soniclib/invn/soniclib/ch_common.d ./Soniclib/invn/soniclib/ch_common.o ./Soniclib/invn/soniclib/ch_common.su ./Soniclib/invn/soniclib/ch_driver.cyclo ./Soniclib/invn/soniclib/ch_driver.d ./Soniclib/invn/soniclib/ch_driver.o ./Soniclib/invn/soniclib/ch_driver.su ./Soniclib/invn/soniclib/ch_log.cyclo ./Soniclib/invn/soniclib/ch_log.d ./Soniclib/invn/soniclib/ch_log.o ./Soniclib/invn/soniclib/ch_log.su ./Soniclib/invn/soniclib/ch_math_utils.cyclo ./Soniclib/invn/soniclib/ch_math_utils.d ./Soniclib/invn/soniclib/ch_math_utils.o ./Soniclib/invn/soniclib/ch_math_utils.su ./Soniclib/invn/soniclib/ch_rangefinder.cyclo ./Soniclib/invn/soniclib/ch_rangefinder.d ./Soniclib/invn/soniclib/ch_rangefinder.o ./Soniclib/invn/soniclib/ch_rangefinder.su

.PHONY: clean-Soniclib-2f-invn-2f-soniclib

