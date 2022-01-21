################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../glcd-master/devices/AVR8.c \
../glcd-master/devices/LPC111x.c \
../glcd-master/devices/LPC11Uxx.c \
../glcd-master/devices/PIC24H.c \
../glcd-master/devices/PIC32.c \
../glcd-master/devices/STM32F0xx.c \
../glcd-master/devices/STM32F10x.c \
../glcd-master/devices/STM32F4.c 

OBJS += \
./glcd-master/devices/AVR8.o \
./glcd-master/devices/LPC111x.o \
./glcd-master/devices/LPC11Uxx.o \
./glcd-master/devices/PIC24H.o \
./glcd-master/devices/PIC32.o \
./glcd-master/devices/STM32F0xx.o \
./glcd-master/devices/STM32F10x.o \
./glcd-master/devices/STM32F4.o 

C_DEPS += \
./glcd-master/devices/AVR8.d \
./glcd-master/devices/LPC111x.d \
./glcd-master/devices/LPC11Uxx.d \
./glcd-master/devices/PIC24H.d \
./glcd-master/devices/PIC32.d \
./glcd-master/devices/STM32F0xx.d \
./glcd-master/devices/STM32F10x.d \
./glcd-master/devices/STM32F4.d 


# Each subdirectory must supply rules for building sources it contributes
glcd-master/devices/AVR8.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/AVR8.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/LPC111x.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/LPC111x.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/LPC11Uxx.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/LPC11Uxx.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/PIC24H.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/PIC24H.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/PIC32.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/PIC32.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/STM32F0xx.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/STM32F0xx.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/STM32F10x.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/STM32F10x.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
glcd-master/devices/STM32F4.o: C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master/devices/STM32F4.c glcd-master/devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DGLCD_DEVICE_STM32F4XX -DGLCD_CONTROLLER_ST7565R -DGLCD_USE_SPI -DGLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/milon/STM32CubeIDE/workspace_1.6.0/nucleo-glcd-driver/glcd-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

