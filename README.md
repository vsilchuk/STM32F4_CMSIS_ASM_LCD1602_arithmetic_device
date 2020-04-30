## Simple arithmetic device based on the STM32F401RE MCU and LCD1602 display with the PCF8574 I/O expander [CMSIS, NUCLEO-F401RE].

###### NTUU KPI, The Faculty of Electronics, The Department of Design of Electronic Digital Equipment (DEDEC/KEOA).

This is a simple arithmetic device that performs operations of addition, subtraction, multiplication and division over 8-bit numbers **[1...255]** by using the inline assembler function. It was created as a university task and aimed to use the acquired skills to implement a simple project.

See the [project report][1] (in ukrainian) for the details of this project and its detailed description. See also the [circuit diagram][2] to understand how to assemble this circuit.

This project also uses a _simple_ `lcd1602` library, written by me to be used for the work with the LCD1602 display and the PCF8574 I/O expander. It works, but the timings are far from ideal.

## Device structure:

+ The main component of this device is the STM32F401RE MCU, which is the main part of the NUCLEO-F401RE — STM32 Nucleo-64 development board.
+ The control and data input unit is represented by three tactile switches.
+ The display unit implements a visual indication of the operation of the device, and is represented by LCD1602 display (based on the HD44780 controller) and the PCF8574 I/O extender.
	
## Buttons and device configuration:

This device consists of three tactile switches for the device configuration: [SB1, SB2, SB3][2]. These buttons are connected 
to the PB13, PB14 and PB15 MCU pins and internally connected to the MCU VCC **(+3V3)** (_circuit diagram has a mistake here_) by the internal R2, R4 and R5 resistors.

There is also software buttons debounce implemented here.

+ **SB1** tactile switch increments the value of the variable X.
+ **SB2** tactile switch increments the value of the variable Y.
+ **SB3** tactile switch can be used for the device work mode selection. The duration of its pressing determines the choice of the current device operation mode:
  + SB3 pressing duration time > 2s — reset: `X = 1`, `Y = 1`, `mode = ADD`; 
  + SB3 pressing duration time < 2s — change mode: ``ADD`` —> ``SUB`` —> ``MUL`` —> ``DIV``;

The current values of `X`, `Y`, `mode` and result `Z` are constantly displayed.

## Clock configuration:

The startup configuration file for the microcontroller according to the required microcontroller clock timing settings was generated using the [STM32F4xx_Clock_Configuration_V1.1.0][3] tool provided on the STMicroelectronics website:

![Clock configuration](https://github.com/vsilchuk/STM32F4_CMSIS_ASM_LCD1602_arithmetic_device/blob/master/img/clock_configuration.png "Clock configuration")

Therefore, for the correct operation of the given code, make sure that:

+ the system frequency (**SYSCLK**) is **16MHz**;
+ the **APB** and **AHB** buses are clocked at **16MHz**;
+ the **SysTick** system timer is clocked at **16MHz**;

## A few photos of the device:

+ Connection:

	![Connection](https://github.com/vsilchuk/STM32F4_CMSIS_ASM_LCD1602_arithmetic_device/blob/master/img/connection.png "Connection")
	
+ Device in work:

	![Device in work](https://github.com/vsilchuk/STM32F4_CMSIS_ASM_LCD1602_arithmetic_device/blob/master/img/display.jpg "Device in work")
	
## Links:

+ [RM0368 Reference manual — STM32F401xB/C and STM32F401xD/E advanced Arm®-based 32-bit MCUs][4].
+ [UM1724 User manual — STM32 Nucleo-64 boards (MB1136)][5].

[1]: https://github.com/vsilchuk/STM32F4_CMSIS_ASM_LCD1602_arithmetic_device/blob/master/doc/UA_Report.pdf
[2]: https://github.com/vsilchuk/STM32F4_CMSIS_ASM_LCD1602_arithmetic_device/blob/master/doc/UA_Circuit_diagram.pdf 
[3]: https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stsw-stm32091.html
[4]: https://www.st.com/resource/en/reference_manual/dm00096844-stm32f401xb-c-and-stm32f401xd-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
[5]: https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf
