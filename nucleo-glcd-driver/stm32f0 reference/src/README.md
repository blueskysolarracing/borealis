# Demo of GLCD library with STM32F0

## Hardware

* STM32F0DISCOVERY with STM32F051R8T6
* PCD8544 LCD (Nokia 5110 84x48 LCD)

## Hardware interconnects

	SPI1 SCK  PA5
	SPI1 MISO PA6
	SPI1 MOSI PA7
	SS        PA1
	DC        PA2
	RST       PA3

* Push button 'user': `PA0`.
* Blue LED: `PC8`
* Green LED: `PC9`

USART pins (not used currently)

	USART1 Tx PA9
	USART1 Rx PA10
