################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../invn/soniclib/ch_api.c \
../invn/soniclib/ch_common.c \
../invn/soniclib/ch_driver.c \
../invn/soniclib/ch_log.c \
../invn/soniclib/ch_math_utils.c \
../invn/soniclib/ch_rangefinder.c \
../invn/soniclib/chbsp_dummy.c \
../invn/soniclib/chirp_bsp.c 

OBJS += \
./invn/soniclib/ch_api.o \
./invn/soniclib/ch_common.o \
./invn/soniclib/ch_driver.o \
./invn/soniclib/ch_log.o \
./invn/soniclib/ch_math_utils.o \
./invn/soniclib/ch_rangefinder.o \
./invn/soniclib/chbsp_dummy.o \
./invn/soniclib/chirp_bsp.o 

C_DEPS += \
./invn/soniclib/ch_api.d \
./invn/soniclib/ch_common.d \
./invn/soniclib/ch_driver.d \
./invn/soniclib/ch_log.d \
./invn/soniclib/ch_math_utils.d \
./invn/soniclib/ch_rangefinder.d \
./invn/soniclib/chbsp_dummy.d \
./invn/soniclib/chirp_bsp.d 


# Each subdirectory must supply rules for building sources it contributes
invn/soniclib/%.o invn/soniclib/%.su invn/soniclib/%.cyclo: ../invn/soniclib/%.c invn/soniclib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"C:/Users/aleks/STM32CubeIDE/workspace_1.16.0/Ultrasonic_reader/invn" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Common/Inc -I../invn -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-invn-2f-soniclib

clean-invn-2f-soniclib:
	-$(RM) ./invn/soniclib/ch_api.cyclo ./invn/soniclib/ch_api.d ./invn/soniclib/ch_api.o ./invn/soniclib/ch_api.su ./invn/soniclib/ch_common.cyclo ./invn/soniclib/ch_common.d ./invn/soniclib/ch_common.o ./invn/soniclib/ch_common.su ./invn/soniclib/ch_driver.cyclo ./invn/soniclib/ch_driver.d ./invn/soniclib/ch_driver.o ./invn/soniclib/ch_driver.su ./invn/soniclib/ch_log.cyclo ./invn/soniclib/ch_log.d ./invn/soniclib/ch_log.o ./invn/soniclib/ch_log.su ./invn/soniclib/ch_math_utils.cyclo ./invn/soniclib/ch_math_utils.d ./invn/soniclib/ch_math_utils.o ./invn/soniclib/ch_math_utils.su ./invn/soniclib/ch_rangefinder.cyclo ./invn/soniclib/ch_rangefinder.d ./invn/soniclib/ch_rangefinder.o ./invn/soniclib/ch_rangefinder.su ./invn/soniclib/chbsp_dummy.cyclo ./invn/soniclib/chbsp_dummy.d ./invn/soniclib/chbsp_dummy.o ./invn/soniclib/chbsp_dummy.su ./invn/soniclib/chirp_bsp.cyclo ./invn/soniclib/chirp_bsp.d ./invn/soniclib/chirp_bsp.o ./invn/soniclib/chirp_bsp.su

.PHONY: clean-invn-2f-soniclib

