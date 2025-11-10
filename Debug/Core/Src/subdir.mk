################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/b_u585i_iot02a_bus.c \
../Core/Src/b_u585i_iot02a_motion_sensors.c \
../Core/Src/iis2mdc.c \
../Core/Src/iis2mdc_reg.c \
../Core/Src/ism330dhcx.c \
../Core/Src/ism330dhcx_reg.c \
../Core/Src/main.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c 

OBJS += \
./Core/Src/b_u585i_iot02a_bus.o \
./Core/Src/b_u585i_iot02a_motion_sensors.o \
./Core/Src/iis2mdc.o \
./Core/Src/iis2mdc_reg.o \
./Core/Src/ism330dhcx.o \
./Core/Src/ism330dhcx_reg.o \
./Core/Src/main.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o 

C_DEPS += \
./Core/Src/b_u585i_iot02a_bus.d \
./Core/Src/b_u585i_iot02a_motion_sensors.d \
./Core/Src/iis2mdc.d \
./Core/Src/iis2mdc_reg.d \
./Core/Src/ism330dhcx.d \
./Core/Src/ism330dhcx_reg.d \
./Core/Src/main.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/HP/STM32Cube/Repository/STM32Cube_FW_U5_V1.8.0/Drivers/BSP/B-U585I-IOT02A" -I"C:/Users/HP/STM32Cube/Repository/STM32Cube_FW_U5_V1.8.0/Drivers/BSP/Components" -I"C:/Users/HP/STM32Cube/Repository/STM32Cube_FW_U5_V1.8.0/Drivers/BSP/Components/ism330dhcx" -I"C:/Users/HP/STM32Cube/Repository/STM32Cube_FW_U5_V1.8.0/Drivers/BSP/Components/iis2mdc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/b_u585i_iot02a_bus.cyclo ./Core/Src/b_u585i_iot02a_bus.d ./Core/Src/b_u585i_iot02a_bus.o ./Core/Src/b_u585i_iot02a_bus.su ./Core/Src/b_u585i_iot02a_motion_sensors.cyclo ./Core/Src/b_u585i_iot02a_motion_sensors.d ./Core/Src/b_u585i_iot02a_motion_sensors.o ./Core/Src/b_u585i_iot02a_motion_sensors.su ./Core/Src/iis2mdc.cyclo ./Core/Src/iis2mdc.d ./Core/Src/iis2mdc.o ./Core/Src/iis2mdc.su ./Core/Src/iis2mdc_reg.cyclo ./Core/Src/iis2mdc_reg.d ./Core/Src/iis2mdc_reg.o ./Core/Src/iis2mdc_reg.su ./Core/Src/ism330dhcx.cyclo ./Core/Src/ism330dhcx.d ./Core/Src/ism330dhcx.o ./Core/Src/ism330dhcx.su ./Core/Src/ism330dhcx_reg.cyclo ./Core/Src/ism330dhcx_reg.d ./Core/Src/ism330dhcx_reg.o ./Core/Src/ism330dhcx_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32u5xx_hal_msp.cyclo ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_it.cyclo ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.cyclo ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su

.PHONY: clean-Core-2f-Src

