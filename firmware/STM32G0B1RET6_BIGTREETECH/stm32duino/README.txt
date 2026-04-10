If the STM platform is using an stm32duino compatible bootloader, a common practice for 3D printer boards, follow the steps below.

Afer installing stm32duino on your Arduino IDE

1)Modify board.txt:
Usually located as in C:\Users\user\AppData\Arduino15\hardware\Arduino_STM32\boards.txt
In the board.txt, set GenG0.build.flash_offset=0x2000

2)From the Arduino IDE, under Tools menu:
Set U(S)ART support to enabled
Set USB support to none

3)Verify/Compile.

4)Export compiled Library.

5)Copy the binary to an SD card, insert into the board and wait for 10-15 seconds.