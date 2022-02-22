################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/src/lv_themes/lv_theme.c \
../Src/lvgl/src/lv_themes/lv_theme_alien.c \
../Src/lvgl/src/lv_themes/lv_theme_default.c \
../Src/lvgl/src/lv_themes/lv_theme_material.c \
../Src/lvgl/src/lv_themes/lv_theme_mono.c \
../Src/lvgl/src/lv_themes/lv_theme_nemo.c \
../Src/lvgl/src/lv_themes/lv_theme_night.c \
../Src/lvgl/src/lv_themes/lv_theme_templ.c \
../Src/lvgl/src/lv_themes/lv_theme_zen.c 

OBJS += \
./Src/lvgl/src/lv_themes/lv_theme.o \
./Src/lvgl/src/lv_themes/lv_theme_alien.o \
./Src/lvgl/src/lv_themes/lv_theme_default.o \
./Src/lvgl/src/lv_themes/lv_theme_material.o \
./Src/lvgl/src/lv_themes/lv_theme_mono.o \
./Src/lvgl/src/lv_themes/lv_theme_nemo.o \
./Src/lvgl/src/lv_themes/lv_theme_night.o \
./Src/lvgl/src/lv_themes/lv_theme_templ.o \
./Src/lvgl/src/lv_themes/lv_theme_zen.o 

C_DEPS += \
./Src/lvgl/src/lv_themes/lv_theme.d \
./Src/lvgl/src/lv_themes/lv_theme_alien.d \
./Src/lvgl/src/lv_themes/lv_theme_default.d \
./Src/lvgl/src/lv_themes/lv_theme_material.d \
./Src/lvgl/src/lv_themes/lv_theme_mono.d \
./Src/lvgl/src/lv_themes/lv_theme_nemo.d \
./Src/lvgl/src/lv_themes/lv_theme_night.d \
./Src/lvgl/src/lv_themes/lv_theme_templ.d \
./Src/lvgl/src/lv_themes/lv_theme_zen.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/src/lv_themes/%.o: ../Src/lvgl/src/lv_themes/%.c Src/lvgl/src/lv_themes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H753xx -DDEBUG -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/JENNY/Users/Jenny/gen11_blueskyelec/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

