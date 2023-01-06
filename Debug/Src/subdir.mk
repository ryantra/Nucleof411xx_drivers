################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009rtc_lcd.c 

OBJS += \
./Src/009rtc_lcd.o 

C_DEPS += \
./Src/009rtc_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Src/009rtc_lcd.o: ../Src/009rtc_lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/bsp" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/009rtc_lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

