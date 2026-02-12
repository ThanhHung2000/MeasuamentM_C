################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/Common.c \
../modbus/RS232.c \
../modbus/delay.c \
../modbus/modbusSlave.c \
../modbus/modbus_crc.c 

OBJS += \
./modbus/Common.o \
./modbus/RS232.o \
./modbus/delay.o \
./modbus/modbusSlave.o \
./modbus/modbus_crc.o 

C_DEPS += \
./modbus/Common.d \
./modbus/RS232.d \
./modbus/delay.d \
./modbus/modbusSlave.d \
./modbus/modbus_crc.d 


# Each subdirectory must supply rules for building sources it contributes
modbus/%.o modbus/%.su modbus/%.cyclo: ../modbus/%.c modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/PhamVanHung/STM32/usb_to_com/modbus" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus

clean-modbus:
	-$(RM) ./modbus/Common.cyclo ./modbus/Common.d ./modbus/Common.o ./modbus/Common.su ./modbus/RS232.cyclo ./modbus/RS232.d ./modbus/RS232.o ./modbus/RS232.su ./modbus/delay.cyclo ./modbus/delay.d ./modbus/delay.o ./modbus/delay.su ./modbus/modbusSlave.cyclo ./modbus/modbusSlave.d ./modbus/modbusSlave.o ./modbus/modbusSlave.su ./modbus/modbus_crc.cyclo ./modbus/modbus_crc.d ./modbus/modbus_crc.o ./modbus/modbus_crc.su

.PHONY: clean-modbus

