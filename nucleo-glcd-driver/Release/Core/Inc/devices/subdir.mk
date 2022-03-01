################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/devices/AVR8.c \
../Core/Inc/devices/LPC111x.c \
../Core/Inc/devices/LPC11Uxx.c \
../Core/Inc/devices/PIC24H.c \
../Core/Inc/devices/PIC32.c \
../Core/Inc/devices/STM32F0xx.c \
../Core/Inc/devices/STM32F10x.c \
../Core/Inc/devices/STM32F4.c 

OBJS += \
./Core/Inc/devices/AVR8.o \
./Core/Inc/devices/LPC111x.o \
./Core/Inc/devices/LPC11Uxx.o \
./Core/Inc/devices/PIC24H.o \
./Core/Inc/devices/PIC32.o \
./Core/Inc/devices/STM32F0xx.o \
./Core/Inc/devices/STM32F10x.o \
./Core/Inc/devices/STM32F4.o 

C_DEPS += \
./Core/Inc/devices/AVR8.d \
./Core/Inc/devices/LPC111x.d \
./Core/Inc/devices/LPC11Uxx.d \
./Core/Inc/devices/PIC24H.d \
./Core/Inc/devices/PIC32.d \
./Core/Inc/devices/STM32F0xx.d \
./Core/Inc/devices/STM32F10x.d \
./Core/Inc/devices/STM32F4.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/devices/%.o: ../Core/Inc/devices/%.c Core/Inc/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-devices

clean-Core-2f-Inc-2f-devices:
	-$(RM) ./Core/Inc/devices/AVR8.d ./Core/Inc/devices/AVR8.o ./Core/Inc/devices/LPC111x.d ./Core/Inc/devices/LPC111x.o ./Core/Inc/devices/LPC11Uxx.d ./Core/Inc/devices/LPC11Uxx.o ./Core/Inc/devices/PIC24H.d ./Core/Inc/devices/PIC24H.o ./Core/Inc/devices/PIC32.d ./Core/Inc/devices/PIC32.o ./Core/Inc/devices/STM32F0xx.d ./Core/Inc/devices/STM32F0xx.o ./Core/Inc/devices/STM32F10x.d ./Core/Inc/devices/STM32F10x.o ./Core/Inc/devices/STM32F4.d ./Core/Inc/devices/STM32F4.o

.PHONY: clean-Core-2f-Inc-2f-devices

