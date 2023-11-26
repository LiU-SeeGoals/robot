# Robot
This is a Makefile project generated with STM32CubeMX.

## Sheets
[NUCLEO-H755ZI-Q](https://www.st.com/resource/en/user_manual/um2408-stm32h7-nucleo144-boards-mb1363-stmicroelectronics.pdf)

## Building
`cd Makefile && make`

## Programming
There are two option one for command line and one magic gui (that works)

### Terminal based
```
sudo apt install stlink-tools
```

Now from
```
cd Makefile && st-flash --reset write CM7/build/robot_CM7.bin 0x08000000
```

### GUI
Or install the GUI program stmcube32prog which is easier to use.

Click Erasing & programming choose the .bin file create from make process as file path.

Enable Run after programming

Connect to board at the top right corner and click start programming

## Pins
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       |
|---------------|---------|----------------|---------|-------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     |
| 4             | PB9     | GPIO_EXTI9     | IRQ     | NRF_IRQ     |
| 6             | VDD     | VDD            | VDD     | -           |
| 8             | GND     | GND            | GND     | -           |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     |
| 12            | PG9     | SPI1_MISO      | M1      | NRF_MISO    |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER    |
| -             | PD8     | USART3_RX      | -       | USART3_RX   |
| -             | PD9     | USART3_TX      | -       | USART3_TX   |
| -             | PB0     | GPIO_Output    | -       | LED_GREEN   |
| -             | PE1     | GPIO_Output    | -       | LED_YELLOW  |
| -             | PB14    | GPIO_Output    | -       | LED_RED     |
| -             | PC1     | ETH_MDC        | -       | ETH_MDC     |
| -             | PA1     | ETH_REF_CLK    | -       | ETH_REF_CLK |
| -             | PA2     | ETH_MDIO       | -       | ETH_MDIO    |
| -             | PA7     | ETH_CRS_DV     | -       | ETH_CRS_DV  |
| -             | PC4     | ETH_RXD0       | -       | ETH_RXD0    |
| -             | PC5     | ETH_RXD1       | -       | ETH_RXD1    |
| -             | PB15    | ETH_TXD1       | -       | ETH_TXD1    |
| -             | PG11    | ETH_TX_EN      | -       | ETH_TX_EN   |
| -             | PG13    | ETH_TXD0       | -       | ETH_TXD0    |

## MX configuration

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


