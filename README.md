Argon servo drive I/O microcontroller firmware
==============================================

The I/O side ARM MCU firmware for [Granite Devices](http://granitedevices.com) [ARGON](http://granitedevices.com/digital-servo-drive-argon) servo motor drive.

DISCLAIMER
==========

Granite Devices or any of it's personnel doesn't take any responsibility of damages or losses caused by customized or user compiled firmware files! It is well possible to compile a firmware a way that breaks the hardware or causes danger or damage to users or their property. If unsure about what you're doing, leave it to others.

HOW TO COMPILE
==============

*NOTE* This guide is work in progress and incomplete. More information and actual examples will follow later.

Tools needed
------------

a) Sourcery CodeBench Lite Edition for ARM EABI.

This is the GCC compiler for ARM that actually compiles the source code

- Go to http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/
- Follow the link "Download the EABI Release"
- Register to Mentor site to get download access
- Download and install the CodeSourcery Lite

b) GNU Make

Source code of firmware and makefirmware utily is easily compiled by make command. For that, the GNU make must be installed in system. http://www.gnu.org/software/make/

c) Makefirmware utily

This tool converts the generated binary file into .gdf file that may be loaded to Argon drive with Granity software.

The soucre code of this program is included in this repository at utils/makefirmware. Compile it with GCC by running command `make` on the same folder (GCC that compiles C code into target system must be installed to the system).

TODO: add compiled makefirmware executable for convenience.

Compiling firmware
------------------

- Open command prompt and go to project directory where Makefile lies.
- Run make by entering command: `make` (Compiler outputs lots of stuff)
- When comipation is succeed, something like following text should become as last output lines:

		text    data     bss     dec     hex filename
		38480    1340    2380   42200    a4d8 argon_bootload_rom.elf
		text+data = FLASH usage, text = code, data = init vars, bss = RAM usage (incl stack), dec & hex = total

If so, then just generated argon_bootload_rom.bin is the binary file going to Argon STM32 MCU. Before it can be loaded into drive, it needs to be converted as .gdf file so Granity can use it.

Convert .bin to .gdf with command:
`makefirmware argon.gdf agron_bootload_rom.bin`

A sucecssfull run of makefirmware will output something like:

		Create file 'argon.gdf'
		Input size=39824 bytes, output=39824 bytes

Installing firmware
===================

Before uploading the newly compiled FW, make sure that you already have the latest available [official Argon FW release](http://granitedevices.com/wiki/Argon_firmware_releases) installed in the drive because the custom I/O side firmware will not affect or upgrade GraniteCore version in the drive. Once latest official FW is present in the drive, continue uploading your newly built argon.gdf by following the FW upgrade [instructions](http://granitedevices.com/wiki/Firmware). 

Be informed that Granite Devices doesn't take any responsibility of damages or losses caused by customzed or user compiled firmware files! It is possible to compile firmware a way that breaks the hardware or causes danger or damage to users or their property.
