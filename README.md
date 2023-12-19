# Robot
This is the firmware that all robots (NUCLEO-H755ZI-Q) are running. It receives data from the [basestation](https://github.com/LiU-SeeGoals/basestation) and acts upon this.

## Contributing
Make sure to follow the [firmware standard](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#seegoal---firmware-standard) and the [feature branch](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#feature-branch-integration) concept.

## Building and flashing
This is a Makefile project generated with STM32CubeMX.

### CLI approach
You'll need [stlink](https://github.com/stlink-org/stlink#installation), usually available through your package manager.

Building the project is done from the `Makefile` directory by running `make`:
~~~bash
# from project root
$ cd Makefile && make
~~~

Flashing can be done when the NUCLEO card is connected through USB (marked `USB PWR`).
~~~bash
# from project root
$ cd Makefile && st-flash --reset write CM7/build/robot_CM7.bin 0x08000000
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
| Zio pin | MCU pin | STM32 function | Label          | Cable colour | NRF Pin |
|---------|---------|----------------|----------------|--------------|---------|
| CN7-1   | PC6     | GPIO_Output    | NRF_CE         | Yellow       | CE      |
| CN7-2   | PB8     | GPIO_Output    | NRF_CSN        | Orange       | CSN     |
| CN7-3   | PB15    | GPIO_EXTI15    | NRF_IRQ        | Gray         | IRQ     |
| CN7-6   | VDD     | VDD            | -              | Red          | VDD     |
| CN7-8   | GND     | GND            | -              | Black        | GND     |
| CN7-10  | PA5     | SPI1_SCK       | NRF_SCK        | Green        | SCK     |
| CN7-12  | PA6     | SPI1_MISO      | NRF_MISO       | Purple       | M1      |
| CN7-14  | PB5     | SPI1_MOSI      | NRF_MOSI       | Blue         | M0      |
| CN10-4  | PA8     | TIM1_CH1       | MOTOR1_PWM     | -            | -       |
| CN10-6  | PE11    | TIM1_CH2       | MOTOR2_PWM     | -            | -       |
| CN10-7  | PF6     | GPIO_Input     | MOTOR1_ENCODER | -            | -       |
| CN10-8  | PE14    | TIM1_CH4       | MOTOR3_PWM     | -            | -       |
| CN10-9  | PF10    | GPIO_Input     | MOTOR2_ENCODER | -            | -       |
| CN10-10 | PE13    | TIM1_CH3       | MOTOR4_PWM     | -            | -       |
| CN10-11 | PA2     | GPIO_Input     | MOTOR3_ENCODER | -            | -       |
| CN10-13 | PG6     | GPIO_Input     | MOTOR4_ENCODER | -            | -       |
| CN10-14 | PB6     | GPIO_Output    | MOTOR1_REVERSE | -            | -       |
| CN10-15 | PB2     | GPIO_Output    | MOTOR1_BREAK   | -            | -       |
| CN10-18 | PE8     | GPIO_Output    | MOTOR2_REVERSE | -            | -       |
| CN10-19 | PD13    | GPIO_Output    | MOTOR2_BREAK   | -            | -       |
| CN10-20 | PE7     | GPIO_Output    | MOTOR3_REVERSE | -            | -       |
| CN10-21 | PD12    | GPIO_Output    | MOTOR3_BREAK   | -            | -       |
| CN10-24 | PE10    | GPIO_Output    | MOTOR4_REVERSE | -            | -       |
| CN10-25 | PE2     | GPIO_Output    | MOTOR4_BREAK   | -            | -       |
| -       | PC13    | GPIO_EXTI13    | BTN_USER       | -            | -       |
| -       | PD8     | USART3_TX      | USART3_TX      | -            | -       |
| -       | PD9     | USART3_RX      | USART3_RX      | -            | -       |
| -       | PB0     | GPIO_Output    | LED_GREEN      | -            | -       |
| -       | PE1     | GPIO_Output    | LED_YELLOW     | -            | -       |
| -       | PB14    | GPIO_Output    | LED_RED        | -            | -       |

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


