################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/BMI088Driver.cpp \
../Core/Src/BaroMagDriver.cpp \
../Core/Src/BatteryMon.cpp \
../Core/Src/BlinkLEDTask.cpp \
../Core/Src/Command.cpp \
../Core/Src/CompFilter.cpp \
../Core/Src/Console.cpp \
../Core/Src/I2CWrapper.cpp \
../Core/Src/LPF.cpp \
../Core/Src/Task.cpp \
../Core/Src/TaskMaster.cpp \
../Core/Src/Telem_out.cpp \
../Core/Src/main.cpp \
../Core/Src/uTopics.cpp 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/BMI088Driver.o \
./Core/Src/BaroMagDriver.o \
./Core/Src/BatteryMon.o \
./Core/Src/BlinkLEDTask.o \
./Core/Src/Command.o \
./Core/Src/CompFilter.o \
./Core/Src/Console.o \
./Core/Src/I2CWrapper.o \
./Core/Src/LPF.o \
./Core/Src/Task.o \
./Core/Src/TaskMaster.o \
./Core/Src/Telem_out.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uTopics.o 

CPP_DEPS += \
./Core/Src/BMI088Driver.d \
./Core/Src/BaroMagDriver.d \
./Core/Src/BatteryMon.d \
./Core/Src/BlinkLEDTask.d \
./Core/Src/Command.d \
./Core/Src/CompFilter.d \
./Core/Src/Console.d \
./Core/Src/I2CWrapper.d \
./Core/Src/LPF.d \
./Core/Src/Task.d \
./Core/Src/TaskMaster.d \
./Core/Src/Telem_out.d \
./Core/Src/main.d \
./Core/Src/uTopics.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMI088Driver.d ./Core/Src/BMI088Driver.o ./Core/Src/BaroMagDriver.d ./Core/Src/BaroMagDriver.o ./Core/Src/BatteryMon.d ./Core/Src/BatteryMon.o ./Core/Src/BlinkLEDTask.d ./Core/Src/BlinkLEDTask.o ./Core/Src/Command.d ./Core/Src/Command.o ./Core/Src/CompFilter.d ./Core/Src/CompFilter.o ./Core/Src/Console.d ./Core/Src/Console.o ./Core/Src/I2CWrapper.d ./Core/Src/I2CWrapper.o ./Core/Src/LPF.d ./Core/Src/LPF.o ./Core/Src/Task.d ./Core/Src/Task.o ./Core/Src/TaskMaster.d ./Core/Src/TaskMaster.o ./Core/Src/Telem_out.d ./Core/Src/Telem_out.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/uTopics.d ./Core/Src/uTopics.o

.PHONY: clean-Core-2f-Src

