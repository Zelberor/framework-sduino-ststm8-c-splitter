#

# Sduino: Small Devices Arduino

### ***This repository only contains modified sduino core files for use with PlatformIO***
Go [here](https://github.com/tenbaht/sduino) for the complete project.

#

#### **Getting started on the STM8 the easy way.**

An Arduino-like programming API that can be used from within the Arduino IDE
or for Makefile-controlled builds.

 * [Project website](https://tenbaht.github.io/sduino/) for more
   information on supported hardware and the programming API

Since this project is based on the SDCC Small Devices C compiler, I called
it "Small Devices -uino" or "Small-duino". It is entirely based on free
tools that are available for Linux, MacOS, and Windows: SDCC, make, and
stm8flash.

This project is not supposed to be “better than Arduino”. It’s purpose
is to give you a head start into a different CPU architecture if you happen
to have a professional need or a private desire for it.


## Included libraries

Most parts of the Arduino core system and some Arduino libraries are already
ported to C-syntax. The resulting API is still very close to the C++ version
and porting an existing application is not hard. Check out the [migration
guide](https://tenbaht.github.io/sduino/api/migration/) for details.


#### Communication

* SPI: Real hardware-SPI up to 10MHz.
* Wire: Port of the stock Wire library for I2C communication (with
  improvements)
* I2C: Port of the I2C master library by Wayne Truchsess
* HardwareSerial: The standard serial interface.

#### Displays

* LiquidCrystal: HD44780 based text LCDs
* LiquidCrystal_I2C: HD44780 based text LCDs with I2C converter backpack
* LiquidCrystal_pcf2119: PCF2119 based text LCDs with I2C connection
* PCD8544: Monochrome graphical LCD based on the PCD8544 controller like the
  Nokia 5110 display. SPI mode only.
* Mini_SSD1306: SSD1306-based monochrome OLED displays with 128x64 pixels.
  I2C support only.

#### Storage

* EEPROM: Port of the stock EEPROM library for accessing the buildin EEPROM

#### Motor control

* Stepper: Stepper motors with 2, 4 or 5 phases.
* Servo: Up to 12 servos using only 1 timer.


## Compatibility with the Arduino world

Since there is no free C++ compiler for the STM8, it is impossible to do a
full 1:1 port of the whole enviroment as is has been done for the STM32 and
the ESP8266.

This is not a drop-in replacement for an AVR, but thanks to some C
preprocessor magic the programming API is still very, very similar and it is
often enough to just move over the opening bracket of the class
instanciation statement and to replace the dot in a method call for an
underscore. Check the [migration
guide](https://tenbaht.github.io/sduino/api/migration/) for an overview.
