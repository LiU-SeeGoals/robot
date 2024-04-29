# Robot
This is the firmware that all robots (NUCLEO-H755ZI-Q) are running. It receives data from the [basestation](https://github.com/LiU-SeeGoals/basestation) and acts upon this.

## Contributing
Make sure to follow the [firmware standard](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#seegoal---firmware-standard) and the [feature branch](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#feature-branch-integration) concept.

## Building and flashing
This is a Makefile project generated with STM32CubeMX.

### CLI approach
You'll need [stlink](https://github.com/stlink-org/stlink#installation), usually available through your package manager.

You need a arm cross compiler, sometimes this comes by default

~~~bash
# Install arm cross compiler
$ sudo apt install gcc-arm-none-eabi
~~~

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
| 8       | PE14    | TIM1_CH4       | MOTOR4_PWM       | -            |
| 9       | PF10    | GPIO_Input     | MOTOR2_ENCODER   | -            |
| 10      | PE13    | TIM1_CH3       | MOTOR3_PWM       | -            |
| 11      | PA2     | GPIO_Input     | MOTOR3_ENCODER   | -            |
| 13      | PG6     | GPIO_Input     | MOTOR4_ENCODER   | -            |
| 14      | PB6     | GPIO_Output    | MOTOR1_REVERSE   | -            |
| 15      | PB2     | GPIO_Output    | MOTOR1_BREAK     | -            |
| 18      | PE8     | GPIO_Output    | MOTOR2_REVERSE   | -            |
| 19      | PD13    | GPIO_Output    | MOTOR2_BREAK     | -            |
| 20      | PE7     | GPIO_Output    | MOTOR3_REVERSE   | -            |
| 21      | PD12    | GPIO_Output    | MOTOR3_BREAK     | -            |
| 24      | PE10    | -              |-                 | -            |
| 30      | PE15    | GPIO_Output    | MOTOR4_REVERSE   | -            |
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
### Debugging

How to use GDB
run st-util in folder with .elf file, this starts gdb server
in MakeFile/CM*/Build
run
```
st-util
```

somewhere else run

```
gdb robot_CM7.elf (or other name for the .elf file)
```

in gdb run 

```
target remote localhost:4242
```

The :4242 port can in theory change so check the output from the st-util command to be sure

in gdb you can for example run 

```
b main:140
```

to create a breakpoint at line 140 in the main.c file
and you can check the surrounding code by running

```
l
```

peace be with you for feeling this desperate, good luck...


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


## How to use motor driver

Motor driver has 5 pins, set these signals in STM32CubeMX and check the devboard documentation to find the corresponding pin.

```
PWM, - Takes PWM signal to decide speed of motor, generate from devboard with timers.

FGOUT, - Hall sensor tick output, work like a wheel encoder and ticking as the motor spins so that we know how much the motor has spun.

nFault, - Gives fault messages when fault states are entered by going low. Currently does not work, set this pin HIGH 5v or motor driver can enter test mode.

brake - HIGH sets all motor coils high breaking the motor hard.

dir - Decides direction for motor
```

## How to run motor

This will be a overview to give a feeling for the system as things might change with time

What do you need to understand?

* How hardware interuppts work
* How hardware timers work 
* PWM signals.

PWM signals are important to understand to be able to run the motors. And the basics will help to understand how the control loop works. Best is to find some tutorials on youtube

Each motor has a timer which ticks each time FGOUT ticks, this is used in an interrupt which is set to interrupt with a fixed frequency. In the interrupt the delta ticks are counted from the previous interrupt called, this way we know how much/fast the motor has moved.

The control loop is currently a PI loop in MOTOR_SetSpeed (name subject to change) which uses the delta ticks to set a control signal, which is sent as a signal between 0 - 1, this is then scaled and a PWM signal is sent.


## Short about timers

Timers can be set in STM32CubeMX with prescaler and period which decides how fast it will tick compared to the system clock. Sometimes this systemclock is prescaled before coming to the timers, meaning you have to check clock configuration in STM32CubeMX to find the exact frequency, likely this value is 200Mhz (400Mhz prescaled by 2).

The timers are TIM1-12 and LPTIM1-4, LPTIM has less functionality then TIM

EXAMPLE:

TIM1 has prescaler of 10, period of 100

System clock is 400Mhz, however looking at clock config we see that before coming to the timer this is caled by two, meaning the timer gets tick frequncy of 200Mhz.

Now 200Mhz is divided by prescaler 200/10 = 20Mhz and the timer overflows when this has ticked 100 times. 

Now for PWM signals the HIGH part of the pulse width is set by calling

```
  __HAL_TIM_SET_COMPARE(motor->pwm_htim, motor->channel, pwm_speed);
```
  
where pwm_speed is a value between 0 - period.

So if we set pwm_speed = period, the motor will run as fast as possible
and pwm_speed = 0 will turn off the motor.