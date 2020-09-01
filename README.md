Argon servo drive I/O microcontroller firmware
==============================================

The I/O side ARM MCU firmware for [Granite Devices](http://granitedevices.com) [ARGON](http://granitedevices.com/digital-servo-drive-argon) servo motor drive.

DISCLAIMER
==========

Granite Devices or any of it's personnel doesn't take any responsibility of damages or losses caused by customized or user compiled firmware files! It is well possible to compile a firmware a way that breaks the hardware or causes danger or damage to users or their property. If unsure about what you're doing, leave it to others.

HOW TO COMPILE
==============

*NOTE* This guide is partially work in progress. More information and actual FW modification examples shall follow later.

Tools needed
------------

a) GCC compiler for ARM

A GCC compiler toolchain for ARM is needed to compile the source code. Get GCC ARM toolchain from https://launchpad.net/gcc-arm-embedded

b) GNU Make

Source code of firmware and makefirmware utily is easily compiled by `make` command. For that, the GNU make must be installed in system. 

For Windows system, an easy way to obtain make is to install "msys-base" and "msys-make" packages from MinGW Installation Manager. MinGW site: http://www.mingw.org/ Download the installation manager at: http://sourceforge.net/projects/mingw/files/Installer/ . You may also add MSys binaries to system environment variable called PATH to have make command to work from command prompt without typing full executable path every time. If the default folder was used, make.exe is located at: C:\MinGW\msys\1.0\bin.

Ready-to-use build environment for Windows
------------------------------------------

For Windows users, there is also alternative way to get ready-to-launch build environment with all necessary tools from https://github.com/GraniteDevices/ArgonServoDriveFirmware-windows-build-environment.

Tips
----
1) Make sure that system path is set so that make and arm-none-eabi-gcc are found "globally" in the system. I.e. when you enter in command `make` or `arm-none-eabi-gcc` in a command prompt, you don't get any kind of "command not found" message.

2) If you're compiling firmware on Linux, you may need to build makefirmware utily before firmware compilation. You can do it by entering in subdirectory utils/makefirmware and running `make` command (assuming that native gcc toolchain in addition to ARM toolchain is installed). You can check installation of gcc by entering command `gcc` alone on command prompt and verify the response like you did in the step 1.

Compiling firmware
------------------

- Open command prompt and go to project directory where Makefile lies.
- Run make by entering command: `make` (Compiler outputs lots of stuff)
- When comipation is succeed, something like following text should become as last output lines:

		Create file 'argon_bootload_rom.gdf'
		Input size=39684 bytes, output=39684 bytes
		arm-none-eabi-size argon_bootload_rom.elf
   		text    data     bss     dec     hex filename
  		37516    2164    2468   42148    a4a4 argon_bootload_rom.elf
		text+data = FLASH usage, text = code, data = init vars, bss = RAM usage (incl stack), dec & hex = total

If so, then just generated argon_bootload_rom.gdf is the binary file going to Argon STM32 MCU. 

Installing firmware
===================

Before uploading the newly compiled FW, make sure that you already have the latest available [official Argon FW release](http://granitedevices.com/wiki/Argon_firmware_releases) installed in the drive because the custom I/O side firmware will not affect or upgrade GraniteCore version in the drive. Once latest official FW is present in the drive, continue uploading your newly built argon.gdf by following the FW upgrade [instructions](http://granitedevices.com/wiki/Firmware). 

Be informed that Granite Devices doesn't take any responsibility of damages or losses caused by customzed or user compiled firmware files! It is possible to compile firmware a way that breaks the hardware or causes danger or damage to users or their property.

Development tips
================

Avoid the usage of following C/C++ functionality due to embedded system restrictions (memory footprint, performance reasons etc):

- new and delete operators - make only static instances of classes
- virtual methods
- free() function (however malloc() is ok for static buffer allocation)
- math functions (sin, cos, sqrt) due to slow execution, however basic single precision floating point arithmetics is pretty fast (+,-,/,*)

SUPPORT
=======

The source is released "as is" without a promise to provide support on any topic related to it. To inquire development support service or customization jobs, send a message through http://granitedevices.com/support. Granite Devices offers I/O side, as well GraniteCore, firmware customization work on hourly rated fee.

