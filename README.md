# Nucleof411xx_drivers

The goal of the project is create the device driver for STM32 NUCLEOF4xx without using any third parties libraries.

Everything is coded referring the technical refrence manual of NUCLEO board creating own library. 

Unlike Arduino quick prototyping, this project helped to Master the working mechanism of Microcontrollers and peripherals.

Major goal of the project were

   - Understand Right ways of Handling and programming MCU Peripherals
   - Understand complete Driver Development steps right from scratch for GPIO,SPI,I2C and USART
   - Learn Writing peripheral driver headers, prototyping APIs and implementation
   - Explore MCU data sheets, Reference manuals, start-up Codes to get things done
   - Right ways of handling/configuring Interrupts for various peripherals
   - Explore Peripheral IRQs/Vector table/NVIC interfaces 
   - Explore Configuration/status/Control registers of various Peripherals
   - Demystifying behind the scene working details of SPI,I2C,GPIOs,USART etc.
   - Explore hidden secretes of MCU bus interfaces, clock sources, MCU clock configurations, etc.
   - Understand right ways of enabling/configuring peripheral clocks/serial clocks/baud rates of various serial protocols
   - Learn about MCUs AHB, APB bus protocols
   - Learn about different MCU clocks like HCLK, PCLK, PLL,etc
   - Learn to capture/decode/analyze traces of serial protocols on Logic analyzer
   - Learn about Quick ways of debugging peripheral issues with case studies
  
  Hardware USED
  
  1. NUCLEOF411re development board https://www.st.com/en/evaluation-tools/nucleo-f411re.html
  2. USB Logic Analyser  https://www.sparkfun.com/products/18627 integrated with PulseView Software
  3. Different sensors and other electronics components devices were also used


  
  Software USED

  1.STM32CubeIDE Generic Linux Installer.

  Downloading link STMCUBE32 IDE

  https://www.st.com/en/development-tools/stm32cubeide.html

  Once zip downloaded and extracted you get a .sh script file.
  Make it executable opening in terminal.

       $ (chmod +x ***.sh)

  For Ubuntu, either:

      $ sudo ./st-stm32cubeide_1.4.0_7511_20200720_0928_amd64.deb_bundle​.sh

  or 

      $ ./st-stm32cubeide_1.4.0_7511_20200720_0928_amd64​.sh

   2. SIGROK (Pulse View) Logic Analysing Tool
       (https://sigrok.org/wiki/Linux)

   Host machine was Ubuntu 20.04, installation guide for Debian


        $ sudo apt-get install git-core g++ make cmake libtool pkg-config \
          libglib2.0-dev libboost-test-dev libboost-serialization-dev \
          libboost-filesystem-dev libboost-system-dev libqt5svg5-dev qtbase5-dev\
          qttools5-dev

   Building

        $ git clone git://sigrok.org/pulseview
        $ cd pulseview
        $ cmake .
        $ make
        $ sudo make install
 
