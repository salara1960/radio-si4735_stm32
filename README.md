# radio-si4735_stm32
radio device project : si4735 chip + stm32f103 arm + st7789 display + NS8002 power amplifier chip

Hardware components :

```
* SI4735-D60-GU, RF receiver AM/FM/LW/SW
* STM32F103 (Black Pill board)
* ST7789 1.3 inch IPS display 240x240 RGB
* NS8002 chip - audio power amplifier
* speaker 8 Om 3 W
* 3-button module
* Valkoder module
```

Software components :

```
* STM32CubeMX - graphic package for creating projects (in C) for microcontrollers of the STM32 family
  (https://www.st.com/en/development-tools/stm32cubemx.html).
* System Workbench for STM32 - IDE software development environment for STM32 microcontrollers
  (https://www.st.com/en/development-tools/sw4stm32.html).
* stm32flash - utility for writing firmware to flash memory of STM32 microcontrollers
   via the built-in USART1 port (https://sourceforge.net/projects/stm32flash/files/)
* STM32CubeProgrammer - utility for writing firmware to flash memory of STM32 microcontrollers
  (https://www.st.com/en/development-tools/stm32cubeprog.html).
* ST-LINK V2 - usb debugger for STM8 / STM32 microcontrollers.
* Saleae USB Logic 8ch - logic signal analyzer, 8 channels, max. sampling rate 24MHz
  (https://www.saleae.com/ru/downloads/).
```


P.S.

This project is under development now and will replenish the functionality,
while only the FM mode works in the first approximation.

