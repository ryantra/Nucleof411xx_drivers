################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/nucleof41xx_gpio_driver.c \
../drivers/src/nucleof41xx_i2c_driver.c \
../drivers/src/nucleof41xx_rcc_driver.c \
../drivers/src/nucleof41xx_spi_driver.c \
../drivers/src/nucleof41xx_usart_driver.c 

OBJS += \
./drivers/src/nucleof41xx_gpio_driver.o \
./drivers/src/nucleof41xx_i2c_driver.o \
./drivers/src/nucleof41xx_rcc_driver.o \
./drivers/src/nucleof41xx_spi_driver.o \
./drivers/src/nucleof41xx_usart_driver.o 

C_DEPS += \
./drivers/src/nucleof41xx_gpio_driver.d \
./drivers/src/nucleof41xx_i2c_driver.d \
./drivers/src/nucleof41xx_rcc_driver.d \
./drivers/src/nucleof41xx_spi_driver.d \
./drivers/src/nucleof41xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/nucleof41xx_gpio_driver.o: ../drivers/src/nucleof41xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/nucleof41xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/nucleof41xx_i2c_driver.o: ../drivers/src/nucleof41xx_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/nucleof41xx_i2c_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/nucleof41xx_rcc_driver.o: ../drivers/src/nucleof41xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/nucleof41xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/nucleof41xx_spi_driver.o: ../drivers/src/nucleof41xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/nucleof41xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/nucleof41xx_usart_driver.o: ../drivers/src/nucleof41xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DDEBUG -c -I"/home/rupesh/Device_driver/nucleof41xx_drivers/drivers/Inc" -I"/home/rupesh/Device_driver/nucleof41xx_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/nucleof41xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

