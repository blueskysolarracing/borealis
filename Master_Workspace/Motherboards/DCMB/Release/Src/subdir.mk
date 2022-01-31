################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/DISP_Ds_24_7F.c \
../Src/DISP_Ds_55_7F.c \
../Src/DISP_Mot_Eco_img.c \
../Src/DISP_Mot_Fwd_img.c \
../Src/DISP_Mot_Off_img.c \
../Src/DISP_Mot_On_img.c \
../Src/DISP_Mot_Pwr_img.c \
../Src/DISP_Mot_Rev_img.c \
../Src/DISP_Text_bg.c \
../Src/DISP_Thumb_8_7F.c \
../Src/DISP_bubble_img.c \
../Src/DISP_left_arrow.c \
../Src/DISP_main_bg.c \
../Src/DISP_mot_bg.c \
../Src/DISP_right_arrow.c \
../Src/DISP_stop_sign.c \
../Src/DISP_triangle_sign.c \
../Src/Hack_12_2FA1F.c \
../Src/Hack_16_2FA1F.c \
../Src/Hack_20_2FA1F.c \
../Src/Hack_24_2FA1F.c \
../Src/Hack_28_2FA1F.c \
../Src/Hack_32_2FA1F.c \
../Src/Hack_8_2FA1F.c \
../Src/SSD1322.c \
../Src/bgpio.c \
../Src/bspi.c \
../Src/disp_renderer.c \
../Src/freertos.c \
../Src/h7Boot.c \
../Src/main.c \
../Src/sprites.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_hal_timebase_tim.c \
../Src/stm32h7xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32h7xx.c 

OBJS += \
./Src/DISP_Ds_24_7F.o \
./Src/DISP_Ds_55_7F.o \
./Src/DISP_Mot_Eco_img.o \
./Src/DISP_Mot_Fwd_img.o \
./Src/DISP_Mot_Off_img.o \
./Src/DISP_Mot_On_img.o \
./Src/DISP_Mot_Pwr_img.o \
./Src/DISP_Mot_Rev_img.o \
./Src/DISP_Text_bg.o \
./Src/DISP_Thumb_8_7F.o \
./Src/DISP_bubble_img.o \
./Src/DISP_left_arrow.o \
./Src/DISP_main_bg.o \
./Src/DISP_mot_bg.o \
./Src/DISP_right_arrow.o \
./Src/DISP_stop_sign.o \
./Src/DISP_triangle_sign.o \
./Src/Hack_12_2FA1F.o \
./Src/Hack_16_2FA1F.o \
./Src/Hack_20_2FA1F.o \
./Src/Hack_24_2FA1F.o \
./Src/Hack_28_2FA1F.o \
./Src/Hack_32_2FA1F.o \
./Src/Hack_8_2FA1F.o \
./Src/SSD1322.o \
./Src/bgpio.o \
./Src/bspi.o \
./Src/disp_renderer.o \
./Src/freertos.o \
./Src/h7Boot.o \
./Src/main.o \
./Src/sprites.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_hal_timebase_tim.o \
./Src/stm32h7xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32h7xx.o 

C_DEPS += \
./Src/DISP_Ds_24_7F.d \
./Src/DISP_Ds_55_7F.d \
./Src/DISP_Mot_Eco_img.d \
./Src/DISP_Mot_Fwd_img.d \
./Src/DISP_Mot_Off_img.d \
./Src/DISP_Mot_On_img.d \
./Src/DISP_Mot_Pwr_img.d \
./Src/DISP_Mot_Rev_img.d \
./Src/DISP_Text_bg.d \
./Src/DISP_Thumb_8_7F.d \
./Src/DISP_bubble_img.d \
./Src/DISP_left_arrow.d \
./Src/DISP_main_bg.d \
./Src/DISP_mot_bg.d \
./Src/DISP_right_arrow.d \
./Src/DISP_stop_sign.d \
./Src/DISP_triangle_sign.d \
./Src/Hack_12_2FA1F.d \
./Src/Hack_16_2FA1F.d \
./Src/Hack_20_2FA1F.d \
./Src/Hack_24_2FA1F.d \
./Src/Hack_28_2FA1F.d \
./Src/Hack_32_2FA1F.d \
./Src/Hack_8_2FA1F.d \
./Src/SSD1322.d \
./Src/bgpio.d \
./Src/bspi.d \
./Src/disp_renderer.d \
./Src/freertos.d \
./Src/h7Boot.d \
./Src/main.d \
./Src/sprites.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_hal_timebase_tim.d \
./Src/stm32h7xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -I"C:/Users/goodb/Documents/code/stm/gen11_blueskyelec_new/Master_Workspace/Shared_Resources/BlueSkyTransmissionProtocol/btcp_inc" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

