## AVR RGBWA Controller
### Overview
This project was born as an ambient light for a living room. I wanted five controllable LED channels and compatibility with 12 V or 24 V LED stripes and an easy to use interface with a single rotary encoder (and it's push button). As a controller I chose an "Arduino pro Micro" from China, because I wanted to be able to add USB support later on and the arduino features an ATmega32U4 with all necessary external components. It was, however, programmed entirely in C without any Arduino-headers or library.

### Schematic and Layout
I tried to only use cheap and easily available standard components, all of which should be available at my usual component shop, Reichelt.de.
While the Layout was made using almost entirely SMD components, all footprints are large enough to be easily soldered by hand without SMD soldering experience or expensive equipment.
The double sided layout itself needed a fair amount of VIAs, but it should be possible to manufacture it at home with a bit of experience in making PCBs.

Eagle files, Gerber Files and pdf layouts, as well as pngs of both layers and the schematic can be found in the PCB folder.

The Arduino can be directly soldered onto the PCB using pin headers and programmed with a 6pin ISP port

### Code
The code is written in plain C, without relying on any non-standard libraries. I have written and compiled it with the most recent version of Atmel Studio, which handles most standard library includes and compiler options by itself, so I will only upload the plain .c file without a makefile apart from the compiled .hex.

I am using the [rotary encoder example of Peter Dannegger](https://www.mikrocontroller.net/articles/Drehgeber "microcontroller.net"). I slightly adapted it to my needs, but left it mostly unchanged.

The push-button as well as the rotary encoder are software-debounced and checked using interrupts.
Some values can be set in the variable definitions at the beginning of the file (initial WA value, initial RGB value, initial brightness, initial colow-sweep speed).

The LEDs are controlled using a soft PWM loop.

### Use
The controller must be fed with a voltage between 8 V and 30V DC, I would recommend using 12 V or 24 V and Common Anode LED stripes with the same operating voltage. Stripes are connected to the terminals next to the power jack. The closest terminal is the positive voltage, the others are the open drain outputs, which are supposed to be connected with the negative wire(s) of the stripes in the following order: R-G-B-W-A-X (X being an unused channel at constant 50% duty cycle in my code, brightness is applied to it as well).
It is controlled with a single rotary encoder and it's push button. A short click on the encoder cycles between the three available lighting modes.

Modes:
1. White: Rotating the encoder changes the balance between warm white and cold white (startup value being both at full power). It is possible to scroll "endlessly" - the colors will loop and no hard transition is visible.
2. RGB Manual: Rotating the encoder cycles through the RGB spectrum with a simple algorithm that never uses more than 2 colors at the same time. It is possible to scroll "endlessly" - the colors will loop and no hard transition is visible.
3. RGB Automatic: The controller automatically cycles through the RGB spectrum and behaves as if the user was constantly rotating the encoder. Operating the encoder in this mode changes the speed with which the colors fade (clockwise -> faster).
A short click in the third mode leads back to the first.

All modes can be interrupted at any point by a long click (~.5s), which will lead to the brightness control function. Here, the encoder controls the brightness which is globally applied to all channels and modes. A short click exits the brightness control and leads back to the mode that was active before.