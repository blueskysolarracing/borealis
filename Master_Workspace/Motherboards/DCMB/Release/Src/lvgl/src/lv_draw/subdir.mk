################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/src/lv_draw/lv_draw.c \
../Src/lvgl/src/lv_draw/lv_draw_arc.c \
../Src/lvgl/src/lv_draw/lv_draw_basic.c \
../Src/lvgl/src/lv_draw/lv_draw_img.c \
../Src/lvgl/src/lv_draw/lv_draw_label.c \
../Src/lvgl/src/lv_draw/lv_draw_line.c \
../Src/lvgl/src/lv_draw/lv_draw_rect.c \
../Src/lvgl/src/lv_draw/lv_draw_triangle.c \
../Src/lvgl/src/lv_draw/lv_img_cache.c \
../Src/lvgl/src/lv_draw/lv_img_decoder.c 

OBJS += \
./Src/lvgl/src/lv_draw/lv_draw.o \
./Src/lvgl/src/lv_draw/lv_draw_arc.o \
./Src/lvgl/src/lv_draw/lv_draw_basic.o \
./Src/lvgl/src/lv_draw/lv_draw_img.o \
./Src/lvgl/src/lv_draw/lv_draw_label.o \
./Src/lvgl/src/lv_draw/lv_draw_line.o \
./Src/lvgl/src/lv_draw/lv_draw_rect.o \
./Src/lvgl/src/lv_draw/lv_draw_triangle.o \
./Src/lvgl/src/lv_draw/lv_img_cache.o \
./Src/lvgl/src/lv_draw/lv_img_decoder.o 

C_DEPS += \
./Src/lvgl/src/lv_draw/lv_draw.d \
./Src/lvgl/src/lv_draw/lv_draw_arc.d \
./Src/lvgl/src/lv_draw/lv_draw_basic.d \
./Src/lvgl/src/lv_draw/lv_draw_img.d \
./Src/lvgl/src/lv_draw/lv_draw_label.d \
./Src/lvgl/src/lv_draw/lv_draw_line.d \
./Src/lvgl/src/lv_draw/lv_draw_rect.d \
./Src/lvgl/src/lv_draw/lv_draw_triangle.d \
./Src/lvgl/src/lv_draw/lv_img_cache.d \
./Src/lvgl/src/lv_draw/lv_img_decoder.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/src/lv_draw/%.o: ../Src/lvgl/src/lv_draw/%.c Src/lvgl/src/lv_draw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

