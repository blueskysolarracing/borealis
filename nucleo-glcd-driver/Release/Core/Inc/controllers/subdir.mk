################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/controllers/NT75451.c \
../Core/Inc/controllers/PCD8544.c \
../Core/Inc/controllers/ST7565R.c 

OBJS += \
./Core/Inc/controllers/NT75451.o \
./Core/Inc/controllers/PCD8544.o \
./Core/Inc/controllers/ST7565R.o 

C_DEPS += \
./Core/Inc/controllers/NT75451.d \
./Core/Inc/controllers/PCD8544.d \
./Core/Inc/controllers/ST7565R.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/controllers/%.o: ../Core/Inc/controllers/%.c Core/Inc/controllers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-controllers

clean-Core-2f-Inc-2f-controllers:
	-$(RM) ./Core/Inc/controllers/NT75451.d ./Core/Inc/controllers/NT75451.o ./Core/Inc/controllers/PCD8544.d ./Core/Inc/controllers/PCD8544.o ./Core/Inc/controllers/ST7565R.d ./Core/Inc/controllers/ST7565R.o

.PHONY: clean-Core-2f-Inc-2f-controllers

