################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f107vchx.s 

OBJS += \
./Core/Startup/startup_stm32f107vchx.o 

S_DEPS += \
./Core/Startup/startup_stm32f107vchx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f107vchx.o: ../Core/Startup/startup_stm32f107vchx.s
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f107vchx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

