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
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       | Cable colour |
|---------------|---------|----------------|---------|-------------|--------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      | Yellow       |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     | Orange       |
| 3             | PB15    | GPIO_EXTI15    | IRQ     | NRF_IRQ     | Gray         |
| 6             | VDD     | VDD            | VDD     | -           | Red          |
| 8             | GND     | GND            | GND     | -           | Black        |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     | Green        |
| 12            | PA6     | SPI1_MISO      | M1      | NRF_MISO    | Purple       |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    | Blue         |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER    | -            |
| -             | PD8     | USART3_TX      | -       | USART3_TX   | -            |
| -             | PD9     | USART3_RX      | -       | USART3_RX   | -            |
| -             | PB0     | GPIO_Output    | -       | LED_GREEN   | -            |
| -             | PE1     | GPIO_Output    | -       | LED_YELLOW  | -            |
| -             | PB14    | GPIO_Output    | -       | LED_RED     | -            |

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


