################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/src/lv_core/lv_disp.c \
../Src/lvgl/src/lv_core/lv_group.c \
../Src/lvgl/src/lv_core/lv_indev.c \
../Src/lvgl/src/lv_core/lv_obj.c \
../Src/lvgl/src/lv_core/lv_refr.c \
../Src/lvgl/src/lv_core/lv_style.c 

OBJS += \
./Src/lvgl/src/lv_core/lv_disp.o \
./Src/lvgl/src/lv_core/lv_group.o \
./Src/lvgl/src/lv_core/lv_indev.o \
./Src/lvgl/src/lv_core/lv_obj.o \
./Src/lvgl/src/lv_core/lv_refr.o \
./Src/lvgl/src/lv_core/lv_style.o 

C_DEPS += \
./Src/lvgl/src/lv_core/lv_disp.d \
./Src/lvgl/src/lv_core/lv_group.d \
./Src/lvgl/src/lv_core/lv_indev.d \
./Src/lvgl/src/lv_core/lv_obj.d \
./Src/lvgl/src/lv_core/lv_refr.d \
./Src/lvgl/src/lv_core/lv_style.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/src/lv_core/%.o: ../Src/lvgl/src/lv_core/%.c Src/lvgl/src/lv_core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H753xx -DDEBUG -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/JENNY/Users/Jenny/gen11_blueskyelec/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

