################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libcanard/drivers/stm32/canard_stm32.c 

OBJS += \
./libcanard/drivers/stm32/canard_stm32.o 

C_DEPS += \
./libcanard/drivers/stm32/canard_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
libcanard/drivers/stm32/canard_stm32.o: ../libcanard/drivers/stm32/canard_stm32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../libcanard/drivers/stm32/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../libcanard -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"libcanard/drivers/stm32/canard_stm32.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
