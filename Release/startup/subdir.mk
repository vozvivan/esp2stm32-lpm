################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f401xx.s 

OBJS += \
./startup/startup_stm32f401xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"/home/ivan/workspace/STM/Hello/Utilities/STM32F401-Discovery" -I"/home/ivan/workspace/STM/Hello/StdPeriph_Driver/inc" -I"/home/ivan/workspace/STM/Hello/inc" -I"/home/ivan/workspace/STM/Hello/CMSIS/device" -I"/home/ivan/workspace/STM/Hello/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


