################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/lvgl/src/lv_misc/lv_anim.c \
../Src/lvgl/src/lv_misc/lv_area.c \
../Src/lvgl/src/lv_misc/lv_async.c \
../Src/lvgl/src/lv_misc/lv_circ.c \
../Src/lvgl/src/lv_misc/lv_color.c \
../Src/lvgl/src/lv_misc/lv_fs.c \
../Src/lvgl/src/lv_misc/lv_gc.c \
../Src/lvgl/src/lv_misc/lv_ll.c \
../Src/lvgl/src/lv_misc/lv_log.c \
../Src/lvgl/src/lv_misc/lv_math.c \
../Src/lvgl/src/lv_misc/lv_mem.c \
../Src/lvgl/src/lv_misc/lv_task.c \
../Src/lvgl/src/lv_misc/lv_templ.c \
../Src/lvgl/src/lv_misc/lv_txt.c \
../Src/lvgl/src/lv_misc/lv_utils.c 

OBJS += \
./Src/lvgl/src/lv_misc/lv_anim.o \
./Src/lvgl/src/lv_misc/lv_area.o \
./Src/lvgl/src/lv_misc/lv_async.o \
./Src/lvgl/src/lv_misc/lv_circ.o \
./Src/lvgl/src/lv_misc/lv_color.o \
./Src/lvgl/src/lv_misc/lv_fs.o \
./Src/lvgl/src/lv_misc/lv_gc.o \
./Src/lvgl/src/lv_misc/lv_ll.o \
./Src/lvgl/src/lv_misc/lv_log.o \
./Src/lvgl/src/lv_misc/lv_math.o \
./Src/lvgl/src/lv_misc/lv_mem.o \
./Src/lvgl/src/lv_misc/lv_task.o \
./Src/lvgl/src/lv_misc/lv_templ.o \
./Src/lvgl/src/lv_misc/lv_txt.o \
./Src/lvgl/src/lv_misc/lv_utils.o 

C_DEPS += \
./Src/lvgl/src/lv_misc/lv_anim.d \
./Src/lvgl/src/lv_misc/lv_area.d \
./Src/lvgl/src/lv_misc/lv_async.d \
./Src/lvgl/src/lv_misc/lv_circ.d \
./Src/lvgl/src/lv_misc/lv_color.d \
./Src/lvgl/src/lv_misc/lv_fs.d \
./Src/lvgl/src/lv_misc/lv_gc.d \
./Src/lvgl/src/lv_misc/lv_ll.d \
./Src/lvgl/src/lv_misc/lv_log.d \
./Src/lvgl/src/lv_misc/lv_math.d \
./Src/lvgl/src/lv_misc/lv_mem.d \
./Src/lvgl/src/lv_misc/lv_task.d \
./Src/lvgl/src/lv_misc/lv_templ.d \
./Src/lvgl/src/lv_misc/lv_txt.d \
./Src/lvgl/src/lv_misc/lv_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Src/lvgl/src/lv_misc/%.o: ../Src/lvgl/src/lv_misc/%.c Src/lvgl/src/lv_misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H753xx -DDEBUG -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/JENNY/Users/Jenny/gen11_blueskyelec/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

