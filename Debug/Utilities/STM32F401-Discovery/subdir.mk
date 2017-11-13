################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/STM32F401-Discovery/stm32f401_discovery.c \
../Utilities/STM32F401-Discovery/stm32f401_discovery_l3gd20.c \
../Utilities/STM32F401-Discovery/stm32f401_discovery_lsm303dlhc.c 

OBJS += \
./Utilities/STM32F401-Discovery/stm32f401_discovery.o \
./Utilities/STM32F401-Discovery/stm32f401_discovery_l3gd20.o \
./Utilities/STM32F401-Discovery/stm32f401_discovery_lsm303dlhc.o 

C_DEPS += \
./Utilities/STM32F401-Discovery/stm32f401_discovery.d \
./Utilities/STM32F401-Discovery/stm32f401_discovery_l3gd20.d \
./Utilities/STM32F401-Discovery/stm32f401_discovery_lsm303dlhc.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/STM32F401-Discovery/%.o: ../Utilities/STM32F401-Discovery/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F401VCTx -DSTM32F401C_DISCO -DDEBUG -DSTM32F401xx -DUSE_STDPERIPH_DRIVER -I"/home/ivan/workspace/STM/Hello/Utilities/STM32F401-Discovery" -I"/home/ivan/workspace/STM/Hello/StdPeriph_Driver/inc" -I"/home/ivan/workspace/STM/Hello/inc" -I"/home/ivan/workspace/STM/Hello/CMSIS/device" -I"/home/ivan/workspace/STM/Hello/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


