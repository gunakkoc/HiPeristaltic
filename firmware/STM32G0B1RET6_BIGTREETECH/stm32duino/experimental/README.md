The firmware in this folder is experimental and not fully tested.

If the STM platform is using an stm32duino compatible bootloader, a common practice for 3D printer boards, follow the steps below:

1) Install [Arduino IDE 2](https://www.arduino.cc/en/software/)
2) Add stm32duino compatibility by following the [official guide](https://github.com/stm32duino/Arduino_Core_STM32/wiki/Getting-Started)
3) Modify `boards.txt`
- Usually located in (or somewhere similar with the latest version number) for Windows:

`C:\Users\YOUR_USERNAME\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.11.0`
- In the boards.txt, locate and change `GenG0.build.flash_offset=0x2000`
4) Open HiPeristaltic.ino with the Arduino IDE 2
5) In Arduino IDE 2: Tools -> Board -> STM32 MCU based boards -> Generic STM32G0 series
6) In Arduino IDE 2: Tools -> Board part number -> Generic G0B1RETx
7) In Arduino IDE 2: Tools -> USB Support -> CDC (generic 'Serial' supersede U(S)ART)
8) In Arduino IDE 2: Tools -> U(S)ART Support -> Enabled (no generic 'Serial')
9) In Arduino IDE 2: Tools -> Optimize -> Fastest (-O3) with LTO
10) In Arduino IDE 2: Sketch -> Verify/Compile
11) In Arduino IDE 2: Sketch -> Export Compiled Binary
12) Go to `build\STMicroelectronics.stm32.GenG0` folder within the sketch's folder and locate `HiPeristaltic.ino.binary`
13) Rename this file as `FIRMWARE.bin` and copy the file to SD card
14) Turn off the board (power-off) then insert the SD card, power on and wait for 20 seconds.