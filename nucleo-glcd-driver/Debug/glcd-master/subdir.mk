################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../glcd-master/glcd.c \
../glcd-master/graphics.c \
../glcd-master/graphs.c \
../glcd-master/text.c \
../glcd-master/text_tiny.c \
../glcd-master/unit_tests.c 

OBJS += \
./glcd-master/glcd.o \
./glcd-master/graphics.o \
./glcd-master/graphs.o \
./glcd-master/text.o \
./glcd-master/text_tiny.o \
./glcd-master/unit_tests.o 

C_DEPS += \
./glcd-master/glcd.d \
./glcd-master/graphics.d \
./glcd-master/graphs.d \
./glcd-master/text.d \
./glcd-master/text_tiny.d \
./glcd-master/unit_tests.d 


# Each subdirectory must supply rules for building sources it contributes
glcd-master/glcd.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/glcd.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/graphics.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/graphics.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/graphs.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/graphs.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/text.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/text.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/text_tiny.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/text_tiny.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/unit_tests.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/unit_tests.c glcd-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

