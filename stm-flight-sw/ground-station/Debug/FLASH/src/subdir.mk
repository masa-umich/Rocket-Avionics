################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FLASH/src/W25N04KV.c 

OBJS += \
./FLASH/src/W25N04KV.o 

C_DEPS += \
./FLASH/src/W25N04KV.d 


# Each subdirectory must supply rules for building sources it contributes
FLASH/src/%.o FLASH/src/%.su FLASH/src/%.cyclo: ../FLASH/src/%.c FLASH/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DDATA_IN_D2_SRAM -DUSE_HAL_DRIVER -DSTM32H725xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"/Users/felix/Desktop/Rocket-Avionics/board-drivers/MAX11128/inc" -I"/Users/felix/Desktop/Rocket-Avionics/board-drivers/TCP/server/inc" -I"/Users/felix/Desktop/Rocket-Avionics/board-drivers/VLVs/inc" -I"/Users/felix/Desktop/Rocket-Avionics/board-drivers/M24256E/inc" -I"/Users/felix/Desktop/Rocket-Avionics/stm-flight-sw/ground-station/FLASH/inc" -I"/Users/felix/Desktop/Rocket-Avionics/system-errors" -I"/Users/felix/Desktop/Rocket-Avionics/board-drivers/TCP/LMP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FLASH-2f-src

clean-FLASH-2f-src:
	-$(RM) ./FLASH/src/W25N04KV.cyclo ./FLASH/src/W25N04KV.d ./FLASH/src/W25N04KV.o ./FLASH/src/W25N04KV.su

.PHONY: clean-FLASH-2f-src

