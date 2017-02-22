#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!    
#
##############################################################################################
# 
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# NOTE: MUST run "make clean" and rebuild with "make" if change BOOTLOADABLE or OPTIMIZATIONS settings!
#
##############################################################################################
# Start of default section
#

#NOTE: MUST run "make clean" and rebuild with "make" if change BOOTLOADABLE or OPTIMIZATIONS settings!

#0=run on bare metal (upload with JTAG/SWD), 1=make bootloadable binary (upload over RS485). 
#recompile if modified because system_stm32f2xx.c needs to be recompiled
BOOTLOADABLE=1

#0=no optimized and full assert on. size typ 72kb. better for debugging
#1=optimized for size and assert off. size typ 36kb
OPTIMIZATIONS=1

#O3 & no assert: 43620b, O3 ASSERT: 54140b, O0 ASSERT: ~65k

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CCP   = $(TRGT)g++
CP   = $(TRGT)objcopy
SIZE   = $(TRGT)size
AS   = $(TRGT)gcc -x assembler-with-cpp
BIN  = $(CP) -O srec 
MCU  = cortex-m3

ifeq ($(OS),Windows_NT)
	RM  = rm
	CAT  = cat
	MAKEFIRMWARE = utils\\makefirmware\\makefirmware.exe
else
	RM  = rm
	CAT  = cat
	MAKEFIRMWARE = utils/makefirmware/makefirmware
endif


# List all default C defines here, like -D_DEBUG=1
ifeq ($(OPTIMIZATIONS),0)
	DDEFS = -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT
	OPT = -O0
else
	DDEFS = -DUSE_STDPERIPH_DRIVER
	OPT = -Os
endif

# List all default ASM defines here, like -D_DEBUG=1
DADEFS = 

# List all default directories to look for include files here
DINCDIR = 

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lstdc++
#DLIBS =

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

# 
# Define project name and Ram/Flash mode here
PROJECT        = argon
RUN_FROM_FLASH = 1

#
# Define linker script file here
#

ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = ./prj/stm32_ram.ld
FULL_PRJ = $(PROJECT)_ram
else
ifeq ($(BOOTLOADABLE),1)
	LDSCRIPT = ./prj/stm32_flash_bootloadable.ld
	FULL_PRJ = $(PROJECT)_bootload_rom
else
	LDSCRIPT = ./prj/stm32_flash.ld
	FULL_PRJ = $(PROJECT)_rom
endif
endif

# List all user C define here, like -D_DEBUG=1
ifeq ($(BOOTLOADABLE),1)
UDEFS = -DPROGRAM_FLASH_OFFSET=0x00008000
else
UDEFS = -DPROGRAM_FLASH_OFFSET=0x00000000
endif 

# Define ASM defines here
UADEFS = 

# List C source files here
# ./src/syscalls.c 
SRC  = ./cmsis/core/core_cm3.c \
 ./cmsis/device/system_stm32f2xx.c \
 ./periphdriver/src/stm32f2xx_dma.c \
 ./periphdriver/src/stm32f2xx_dcmi.c \
 ./periphdriver/src/stm32f2xx_tim.c \
 ./periphdriver/src/stm32f2xx_wwdg.c \
 ./periphdriver/src/stm32f2xx_syscfg.c \
 ./periphdriver/src/stm32f2xx_can.c \
 ./periphdriver/src/stm32f2xx_cryp_aes.c \
 ./periphdriver/src/stm32f2xx_usart.c \
 ./periphdriver/src/stm32f2xx_exti.c \
 ./periphdriver/src/stm32f2xx_rtc.c \
 ./periphdriver/src/stm32f2xx_sdio.c \
 ./periphdriver/src/stm32f2xx_i2c.c \
 ./periphdriver/src/stm32f2xx_cryp.c \
 ./periphdriver/src/stm32f2xx_dac.c \
 ./periphdriver/src/stm32f2xx_iwdg.c \
 ./periphdriver/src/stm32f2xx_pwr.c \
 ./periphdriver/src/stm32f2xx_spi.c \
 ./periphdriver/src/stm32f2xx_hash_sha1.c \
 ./periphdriver/src/misc.c \
 ./periphdriver/src/stm32f2xx_hash.c \
 ./periphdriver/src/stm32f2xx_dbgmcu.c \
 ./periphdriver/src/stm32f2xx_rcc.c \
 ./periphdriver/src/stm32f2xx_adc.c \
 ./periphdriver/src/stm32f2xx_gpio.c \
 ./periphdriver/src/stm32f2xx_cryp_tdes.c \
 ./periphdriver/src/stm32f2xx_cryp_des.c \
 ./periphdriver/src/stm32f2xx_crc.c \
 ./periphdriver/src/stm32f2xx_flash.c \
 ./periphdriver/src/stm32f2xx_hash_md5.c \
 ./periphdriver/src/stm32f2xx_fsmc.c \
 ./periphdriver/src/stm32f2xx_rng.c  \
 ./freertos/portable/MemMang/heap_3.c \
 ./freertos/portable/GCC/ARM_CM3/port.c \
 ./freertos/croutine.c \
 ./freertos/list.c \
 ./freertos/queue.c \
 ./freertos/tasks.c \
 ./freertos/timers.c \
 ./src/syscalls.c 
         
# C++ sources here
CPPSRC =  ./src/AnalogIn.cpp \
 ./src/main.cpp \
 ./src/Serial.cpp \
 ./src/stm32f2xx_it.cpp \
 ./src/mccommunication.cpp \
 ./src/EncoderIn.cpp \
 ./src/globals.cpp \
 ./src/sm485.cpp \
 ./src/utils.cpp \
 ./src/RingBuffer.cpp \
 ./src/SMCommandQueue.cpp \
 ./src/DSCPowerTask.cpp \
 ./src/SMCommandInterpreter.cpp \
 ./src/DigitalInputPin.cpp \
 ./src/DigitalOutPin.cpp \
 ./src/Device.cpp \
 ./src/DigitalCounterInput.cpp \
 ./src/LedBlinkTask.cpp \
 ./src/ResolverIn.cpp \
 ./src/System.cpp \
 ./src/SinCosEncoder.cpp \
 ./src/ProductionTester.cpp
 



# List ASM source files here
ASRC = ./src/startup_stm32f2xx.s 

# List all user directories here
UINCDIR = ./src \
          ./cmsis/core \
          ./cmsis/device \
          ./periphdriver/inc \
          ./freertos/include \
          ./freertos/portable/GCC/ARM_CM3 
           

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =  


#
# End of user defines
##############################################################################################


INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

ifeq ($(RUN_FROM_FLASH), 0)
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM
else
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif

OBJDIR = output/
ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(ASRC:.s=.o) $(SRC:.c=.o) $(CPPSRC:.cpp=.o)
#changes obj paths to output/
#OBJS    = $(addprefix $(OBJDIR),$(notdir $(ASRC:.s=.o))) $(addprefix $(OBJDIR),$(notdir $(SRC:.c=.o)))   $(addprefix $(OBJDIR),$(notdir $(CPPSRC:.cpp=.o)))
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)
# normal debug level 
DEBUGLVL = -g2 -gdwarf-2
# high debug level. -g3 includes ALL symbols for variable watch, including SFR structs 
# from headers such as "ADC1" etc, however produces huge >10M binary (no effect on flash content though)
#DEBUGLVL = -g3 -gdwarf-2 

#arm-none-eabi-objcopy -O binary -j .text -j .data main.elf main.bin

#ASFLAGS = $(MCFLAGS)  -mthumb -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
#CPFLAGS = $(MCFLAGS) $(OPT)  -mthumb  -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
#LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)


ASFLAGS = $(MCFLAGS) $(DEBUGLVL) -mthumb -Wa,-ahlms=$(addprefix $(OBJDIR),$(notdir $<.lst)) $(ADEFS)

CPFLAGS = $(MCFLAGS) $(OPT) $(DEBUGLVL) -ffunction-sections -fdata-sections -fomit-frame-pointer -mthumb  -Wall -fverbose-asm -Wa,-ahlms=$(addprefix $(OBJDIR),$(notdir $<.lst)) $(DEFS)
CPPFLAGS = $(MCFLAGS) $(OPT) $(DEBUGLVL) -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fno-threadsafe-statics -fdata-sections -fomit-frame-pointer -mthumb  -Wall -fverbose-asm -Wa,-ahlms=$(addprefix $(OBJDIR),$(notdir $<.lst)) $(DEFS)
#CPPFLAGS = $(MCFLAGS) $(OPT) $(DEBUGLVL) -fno-rtti -fno-exceptions -ffunction-sections -fno-threadsafe-statics -fdata-sections -fomit-frame-pointer -mthumb -Wall -fverbose-asm -Wa,-ahlms=$<.lst $(DEFS)

#CPPFLAGS = $(MCFLAGS) $(OPT) $(DEBUGLVL)  -mthumb -Wall -fverbose-asm -Wa,-ahlms=$<.lst $(DEFS)
#CPFLAGS = $(MCFLAGS) $(OPT) $(DEBUGLVL)  -mthumb -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)

#LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)
#LDFLAGS = -static -Wl,-cref,-u,Reset_Handler -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x1000 -Wl,-Map=$(FULL_PRJ).map $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT)  $(LIBDIR)
LDFLAGS = -static -Wl,-cref,-u,Reset_Handler -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x1000 -Wl,-Map=$(FULL_PRJ).map $(MCFLAGS) -mthumb  -T$(LDSCRIPT)  $(LIBDIR)

# Generate dependency information
CPFLAGS += -MD -MP -MF .dep/$(@F).d
CPPFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#


all: $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex
	arm-none-eabi-objcopy -O binary  $(FULL_PRJ).elf $(FULL_PRJ).bin
	$(MAKEFIRMWARE)  $(FULL_PRJ).gdf $(FULL_PRJ).bin
	$(SIZE) $(FULL_PRJ).elf
	@echo text+data = FLASH usage, text = code, data = init vars, bss = RAM usage \(incl stack\), dec \& hex = total

%.o : %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $(@)
	
#$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@	

%.o : %.cpp
	$(CCP) -c $(CPPFLAGS) -I . $(INCDIR) $< -o $(@)
	
%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $(@)

%elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@
  
%hex: %elf
	$(BIN) $< $@


	
clean:
	-rm -f $(OBJS)
	-rm -f $(FULL_PRJ).elf
	-rm -f $(FULL_PRJ).map
	-rm -f $(FULL_PRJ).hex
	-rm -f $(SRC:.c=.c.bak)
#	-rm -f $(SRC:.c=.lst)
	-rm -f $(CPPSRC:.cpp=.cpp.bak)
#	-rm -f $(CPPSRC:.cpp=.cpp.lst)
#	-rm -f $(ASRC:.s=.s.bak)
#	-rm -f $(ASRC:.s=.lst)
#	-rm -f output/*.o
	-rm -f output/*.lst
	-rm -f .dep/*.d


# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***