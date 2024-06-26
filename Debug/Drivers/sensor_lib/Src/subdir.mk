################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/sensor_lib/Src/mma8452.c \
../Drivers/sensor_lib/Src/test_functions_mma8452.c 

OBJS += \
./Drivers/sensor_lib/Src/mma8452.o \
./Drivers/sensor_lib/Src/test_functions_mma8452.o 

C_DEPS += \
./Drivers/sensor_lib/Src/mma8452.d \
./Drivers/sensor_lib/Src/test_functions_mma8452.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/sensor_lib/Src/%.o Drivers/sensor_lib/Src/%.su Drivers/sensor_lib/Src/%.cyclo: ../Drivers/sensor_lib/Src/%.c Drivers/sensor_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/oakyol/STM32CubeIDE/DevDriver/Drivers/sensor_lib/Inc" -I"C:/Users/oakyol/STM32CubeIDE/DevDriver/Drivers/sensor_lib/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-sensor_lib-2f-Src

clean-Drivers-2f-sensor_lib-2f-Src:
	-$(RM) ./Drivers/sensor_lib/Src/mma8452.cyclo ./Drivers/sensor_lib/Src/mma8452.d ./Drivers/sensor_lib/Src/mma8452.o ./Drivers/sensor_lib/Src/mma8452.su ./Drivers/sensor_lib/Src/test_functions_mma8452.cyclo ./Drivers/sensor_lib/Src/test_functions_mma8452.d ./Drivers/sensor_lib/Src/test_functions_mma8452.o ./Drivers/sensor_lib/Src/test_functions_mma8452.su

.PHONY: clean-Drivers-2f-sensor_lib-2f-Src

