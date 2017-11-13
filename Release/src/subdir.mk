################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f4xx.c \
../src/tm_stm32f4_gpio.c \
../src/tm_stm32f4_i2c.c 

OBJS += \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f4xx.o \
./src/tm_stm32f4_gpio.o \
./src/tm_stm32f4_i2c.o 

C_DEPS += \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f4xx.d \
./src/tm_stm32f4_gpio.d \
./src/tm_stm32f4_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F401VCTx -DSTM32F401C_DISCO -DSTM32F401xx -DUSE_STDPERIPH_DRIVER -I"/home/ivan/workspace/STM/Hello/Utilities/STM32F401-Discovery" -I"/home/ivan/workspace/STM/Hello/StdPeriph_Driver/inc" -I"/home/ivan/workspace/STM/Hello/inc" -I"/home/ivan/workspace/STM/Hello/CMSIS/device" -I"/home/ivan/workspace/STM/Hello/CMSIS/core" -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


