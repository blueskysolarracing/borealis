################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/src/lv_font/lv_font.c \
../Src/lvgl/src/lv_font/lv_font_fmt_txt.c \
../Src/lvgl/src/lv_font/lv_font_roboto_12.c \
../Src/lvgl/src/lv_font/lv_font_roboto_16.c \
../Src/lvgl/src/lv_font/lv_font_roboto_22.c \
../Src/lvgl/src/lv_font/lv_font_roboto_28.c \
../Src/lvgl/src/lv_font/lv_font_unscii_8.c 

OBJS += \
./Src/lvgl/src/lv_font/lv_font.o \
./Src/lvgl/src/lv_font/lv_font_fmt_txt.o \
./Src/lvgl/src/lv_font/lv_font_roboto_12.o \
./Src/lvgl/src/lv_font/lv_font_roboto_16.o \
./Src/lvgl/src/lv_font/lv_font_roboto_22.o \
./Src/lvgl/src/lv_font/lv_font_roboto_28.o \
./Src/lvgl/src/lv_font/lv_font_unscii_8.o 

C_DEPS += \
./Src/lvgl/src/lv_font/lv_font.d \
./Src/lvgl/src/lv_font/lv_font_fmt_txt.d \
./Src/lvgl/src/lv_font/lv_font_roboto_12.d \
./Src/lvgl/src/lv_font/lv_font_roboto_16.d \
./Src/lvgl/src/lv_font/lv_font_roboto_22.d \
./Src/lvgl/src/lv_font/lv_font_roboto_28.d \
./Src/lvgl/src/lv_font/lv_font_unscii_8.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/src/lv_font/%.o: ../Src/lvgl/src/lv_font/%.c Src/lvgl/src/lv_font/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H753xx -DDEBUG -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/JENNY/Users/Jenny/gen11_blueskyelec/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

