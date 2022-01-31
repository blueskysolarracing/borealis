################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/porting/lv_port_disp_template.c \
../Src/lvgl/porting/lv_port_fs_template.c \
../Src/lvgl/porting/lv_port_indev_template.c 

OBJS += \
./Src/lvgl/porting/lv_port_disp_template.o \
./Src/lvgl/porting/lv_port_fs_template.o \
./Src/lvgl/porting/lv_port_indev_template.o 

C_DEPS += \
./Src/lvgl/porting/lv_port_disp_template.d \
./Src/lvgl/porting/lv_port_fs_template.d \
./Src/lvgl/porting/lv_port_indev_template.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/porting/%.o: ../Src/lvgl/porting/%.c Src/lvgl/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

