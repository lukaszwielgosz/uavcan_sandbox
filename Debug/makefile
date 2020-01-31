################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include libcanard/drivers/stm32/subdir.mk
-include libcanard/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
uavcan_sandbox.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
uavcan_sandbox.list \

OBJCOPY_BIN += \
uavcan_sandbox.bin \


# All Target
all: uavcan_sandbox.elf secondary-outputs

# Tool invocations
uavcan_sandbox.elf: $(OBJS) $(USER_OBJS) /home/lukasz/workspace/stm32/uavcan_sandbox/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "uavcan_sandbox.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/lukasz/workspace/stm32/uavcan_sandbox/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="uavcan_sandbox.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

uavcan_sandbox.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "uavcan_sandbox.list"
	@echo 'Finished building: $@'
	@echo ' '

uavcan_sandbox.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "uavcan_sandbox.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
