# Robot
This is the firmware that all robots (NUCLEO-H755ZI-Q) are running. It receives data from the [basestation](https://github.com/LiU-SeeGoals/basestation) and acts upon this.

## Contributing
Make sure to follow the [firmware standard](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#seegoal---firmware-standard) and the [feature branch](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#feature-branch-integration) concept.

## Building and flashing
This is a CMake project.

### CLI approach
You'll need [stlink](https://github.com/stlink-org/stlink#installation), usually available through your package manager.

Building the project is done with CMake:
~~~bash
# from project root, done once:
$ cmake -B build . -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE
# every time you want to build:
$ cd build && make
~~~

Flashing can be done when the NUCLEO card is connected through USB (marked `USB PWR`)
~~~bash
# from project root:
$ cd build && make flash_cm7
~~~

### GUI approach
Install the GUI program stmcube32prog.

Click Erasing & programming choose the .bin file create from make process as file path.

Enable Run after programming

Connect to board at the top right corner and click start programming

# Documentation

## Sheets
[NUCLEO-H755ZI-Q](https://www.st.com/resource/en/user_manual/um2408-stm32h7-nucleo144-boards-mb1363-stmicroelectronics.pdf)

## Pins

### CN7

| Zio pin | MCU pin | STM32 function | Label    | Cable colour | NRF Pin |
| ------- | ------- | -------------- | -------- | ------------ | ------- |
| 1       | PC6     | GPIO_Output    | NRF_CE   | Yellow       | CE      |
| 2       | PB8     | GPIO_Output    | NRF_CSN  | Orange       | CSN     |
| 3       | PB15    | GPIO_EXTI15    | NRF_IRQ  | Gray         | IRQ     |
| 6       | VDD     | VDD            | -        | Red          | VDD     |
| 8       | GND     | GND            | -        | Black        | GND     |
| 10      | PA5     | SPI1_SCK       | NRF_SCK  | Green        | SCK     |
| 12      | PA6     | SPI1_MISO      | NRF_MISO | Purple       | M1      |
| 14      | PB5     | SPI1_MOSI      | NRF_MOSI | Blue         | M0      |

### CN9

| Zio pin | MCU pin | STM32 function | Label         | Cable colour |
| ------- | ------- | -------------- | ------------- | ------------ |
| 10      | PD3     | GPIO_Output    | SPI_CS_OUTPUT | -            |
| 8       | PD4     | GPIO_Output    | SPI_CS_RESET  | -            |
| 6       | PD5     | GPIO_Output    | SPI_CS_STORE  | -            |
| 4       | PD6     | GPIO_Output    | SPI_CS_SHIFT  | -            |
| 2       | PD7     | GPIO_Output    | SPI_CS_CONF   | -            |

### CN10
| Zio pin | MCU pin | STM32 function | Label            | Cable colour |
|---------|---------|----------------|------------------|--------------|
| 4       | PA8     | TIM1_CH1       | MOTOR1_PWM       | -            |
| 6       | PE11    | TIM1_CH2       | MOTOR2_PWM       | -            |
| 7       | PF6     | GPIO_Input     | MOTOR1_ENCODER   | -            |
| 8       | PE14    | TIM1_CH4       | MOTOR3_PWM       | -            |
| 9       | PF10    | GPIO_Input     | MOTOR2_ENCODER   | -            |
| 10      | PE13    | TIM1_CH3       | MOTOR4_PWM       | -            |
| 11      | PA2     | GPIO_Input     | MOTOR3_ENCODER   | -            |
| 13      | PG6     | GPIO_Input     | MOTOR4_ENCODER   | -            |
| 14      | PB6     | GPIO_Output    | MOTOR1_REVERSE   | -            |
| 15      | PB2     | GPIO_Output    | MOTOR1_BREAK     | -            |
| 18      | PE8     | GPIO_Output    | MOTOR2_REVERSE   | -            |
| 19      | PD13    | GPIO_Output    | MOTOR2_BREAK     | -            |
| 20      | PE7     | GPIO_Output    | MOTOR3_REVERSE   | -            |
| 21      | PD12    | GPIO_Output    | MOTOR3_BREAK     | -            |
| 24      | PE10    | GPIO_Output    | MOTOR4_REVERSE   | -            |
| 25      | PE2     | GPIO_Output    | MOTOR4_BREAK     | -            |
| 32      | PB10    | GPIO_Output    | KICKER_DISCHARGE | -            |
| 34      | PB11    | GPIO_Output    | KICKER_CHARGE    | -            |

### Internal
| Zio pin | MCU pin | STM32 function | Label          | Cable colour |
|---------|---------|----------------|----------------|--------------|
| -       | PC13    | GPIO_EXTI13    | BTN_USER       | -            |
| -       | PD8     | USART3_TX      | USART3_TX      | -            |
| -       | PD9     | USART3_RX      | USART3_RX      | -            |
| -       | PB0     | GPIO_Output    | LED_GREEN      | -            |
| -       | PE1     | GPIO_Output    | LED_YELLOW     | -            |
| -       | PB14    | GPIO_Output    | LED_RED        | -            |

## Creating an `compile_command.json`
~~~bash
# from project root
$ cd Makefile && bear --output ../compile_commands.json -- make
~~~

## MX configuration
If for some reason a new `robot.ioc` has to be created from scratch, these are the changes needed in of STM32CubeMX.

### General stuff
...

### Connectivity

#### USART3
Mode: Asynchronous

#### SPI1
Mode: Full-Duplex Master
Parameter Settings:
~~~
Data Size: 8 Bits
Prescaler: 32
~~~


