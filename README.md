# LaFortunaMandelbrot

This is a mandelbrot generator for the La Fortuna, to build just use 

avr-gcc mandel.c lcd\lcd.c -o mandel.elf -Os -Wall -DF_CPU=8000000 -mmcu=at90usb1286
avr-objcopy -O ihex mandel.elf mandel.hex
dfu-programmer at90usb1286 erase
dfu-programmer at90usb1286 flash mandel.hex
