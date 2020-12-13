################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
test_LCD_f107.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
test_LCD_f107.list \

OBJCOPY_BIN += \
test_LCD_f107.bin \


# All Target
all: test_LCD_f107.elf secondary-outputs

# Tool invocations
test_LCD_f107.elf: $(OBJS) $(USER_OBJS) C:\Users\Dat\ UTC\Documents\STM32\Cube_IDE\test_LCD_f107\STM32F107VCTX_FLASH.ld
	arm-none-eabi-gcc -o "test_LCD_f107.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Dat UTC\Documents\STM32\Cube_IDE\test_LCD_f107\STM32F107VCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="test_LCD_f107.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

test_LCD_f107.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "test_LCD_f107.list"
	@echo 'Finished building: $@'
	@echo ' '

test_LCD_f107.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "test_LCD_f107.bin"
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
