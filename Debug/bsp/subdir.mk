################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds1307.c 

OBJS += \
./bsp/ds1307.o 

C_DEPS += \
./bsp/ds1307.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/ds1307.o: ../bsp/ds1307.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/bsp" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"bsp/ds1307.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

