################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/VLVs/src/VLVs.c 

OBJS += \
./VLVs/src/VLVs.o 

C_DEPS += \
./VLVs/src/VLVs.d 


# Each subdirectory must supply rules for building sources it contributes
VLVs/src/VLVs.o: C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/VLVs/src/VLVs.c VLVs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DDATA_IN_D2_SRAM -DUSE_HAL_DRIVER -DSTM32H725xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/ADS1120/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/LSM6DSO32XTR/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/MAX11128/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/MS5611/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/TCP/server/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/VLVs/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/board-drivers/M24256E/inc" -I"C:/Users/felix/Desktop/MASA/Rocket-Avionics/limestone-firmware/FLASH/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VLVs-2f-src

clean-VLVs-2f-src:
	-$(RM) ./VLVs/src/VLVs.cyclo ./VLVs/src/VLVs.d ./VLVs/src/VLVs.o ./VLVs/src/VLVs.su

.PHONY: clean-VLVs-2f-src

