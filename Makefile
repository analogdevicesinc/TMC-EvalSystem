### Generic settings ###
#VERSION			= 3.06
include version.txt
#DEVICE			= Startrampe
#DEVICE			= Landungsbruecke
LINK			= BL
#LINK			= NOBL
OUTDIR 			= _build_$(DEVICE)
TARGET 			= $(DEVICE)_v$(VERSION)_$(LINK)

# Force IDs (IDs found in tmc/BoardAssignment.h)
# If OVERRIDE is set, any autodetected ID will be discarded
# Else, autodetected IDs preceed ID_CH1_DEFAULT / ID_CH2_DEFAULT
ID_CH1_DEFAULT	?= 0
ID_CH1_OVERRIDE	?= false
ID_CH2_DEFAULT	?= 0
ID_CH2_OVERRIDE	?= false

### Source File Selection ###
# Evalboards
SRC				+= boards/SelfTest_$(DEVICE).c
SRC 			+= boards/Board.c
SRC 			+= boards/TMCDriver.c
SRC 			+= boards/TMCMotionController.c

SRC 			+= boards/Rhino_standalone.c
SRC				+= boards/TMC2041_eval.c
SRC				+= boards/TMC2100_eval.c
SRC				+= boards/TMC2130_eval.c
SRC				+= boards/TMC2160_eval.c
SRC				+= boards/TMC2208_eval.c
SRC				+= boards/TMC2209_eval.c
SRC				+= boards/TMC2224_eval.c
SRC				+= boards/TMC2225_eval.c
SRC				+= boards/TMC2240_eval.c
SRC				+= boards/TMC2590_eval.c
SRC				+= boards/TMC2660_eval.c
SRC				+= boards/TMC2300_eval.c
SRC				+= boards/TMC4330_eval.c
SRC				+= boards/TMC4331_eval.c
SRC				+= boards/TMC4361_eval.c
SRC				+= boards/TMC4361A_eval.c
SRC				+= boards/TMC4670_eval.c
SRC				+= boards/TMC4671_eval.c
SRC				+= boards/TMC5031_eval.c
SRC				+= boards/TMC5041_eval.c
SRC				+= boards/TMC5062_eval.c
SRC				+= boards/TMC5072_eval.c
SRC 			+= boards/TMC5130_eval.c
SRC 			+= boards/TMC5160_eval.c
SRC 			+= boards/TMC6100_eval.c
SRC 			+= boards/TMC6200_eval.c
SRC				+= boards/TMC7300_eval.c
SRC				+= boards/TMC8461_eval.c
SRC				+= boards/TMC8462_eval.c
ifeq ($(DEVICE),Landungsbruecke)
SRC				      += boards/MAX22216_eval.c
SRC             += boards/TMC2226_eval.c
SRC             += boards/TMC6140_eval.c
SRC             += boards/TMC6300_eval.c
endif

# System and hardware abstraction layer
SRC 			+= hal/$(DEVICE)/tmc/SysTick.c
SRC 			+= hal/$(DEVICE)/tmc/IOs.c
SRC 			+= hal/$(DEVICE)/tmc/IOMap.c
SRC 			+= hal/$(DEVICE)/tmc/HAL.c
SRC 			+= hal/$(DEVICE)/tmc/SPI.c
SRC 			+= hal/$(DEVICE)/tmc/USB.c
SRC 			+= hal/$(DEVICE)/tmc/ADCs.c
SRC 			+= hal/$(DEVICE)/tmc/LEDs.c
SRC 			+= hal/$(DEVICE)/tmc/RS232.c
SRC 			+= hal/$(DEVICE)/tmc/WLAN.c
SRC 			+= hal/$(DEVICE)/tmc/Timer.c
SRC 			+= hal/$(DEVICE)/tmc/UART.c
SRC 			+= hal/$(DEVICE)/tmc/RXTX.c

# Control
SRC 			+= main.c
SRC 			+= tmc/TMCL.c
SRC 			+= tmc/RAMDebug.c
SRC 			+= tmc/IdDetection_$(DEVICE).c
SRC				+= tmc/EEPROM.c
SRC 			+= tmc/BoardAssignment.c
SRC 			+= tmc/VitalSignsMonitor.c
SRC 			+= tmc/StepDir.c
ifeq ($(DEVICE),Landungsbruecke)
SRC             += tmc/BLDC.c
endif

# TMC_API
SRC				+= TMC-API/tmc/helpers/Functions.c
SRC				+= TMC-API/tmc/helpers/CRC.c
SRC				+= TMC-API/tmc/ramp/LinearRamp.c
SRC				+= TMC-API/tmc/ramp/LinearRamp1.c
SRC				+= TMC-API/tmc/ramp/Ramp.c
SRC 			+= TMC-API/tmc/ic/MAX22216/MAX22216.c
SRC 			+= TMC-API/tmc/ic/TMC2041/TMC2041.c
SRC 			+= TMC-API/tmc/ic/TMC2130/TMC2130.c
SRC 			+= TMC-API/tmc/ic/TMC2160/TMC2160.c
SRC 			+= TMC-API/tmc/ic/TMC2208/TMC2208.c
SRC 			+= TMC-API/tmc/ic/TMC2209/TMC2209.c
SRC 			+= TMC-API/tmc/ic/TMC2224/TMC2224.c
SRC 			+= TMC-API/tmc/ic/TMC2225/TMC2225.c
SRC 			+= TMC-API/tmc/ic/TMC2226/TMC2226.c
SRC 			+= TMC-API/tmc/ic/TMC2240/TMC2240.c
SRC 			+= TMC-API/tmc/ic/TMC2590/TMC2590.c
SRC 			+= TMC-API/tmc/ic/TMC2660/TMC2660.c
SRC 			+= TMC-API/tmc/ic/TMC2300/TMC2300.c
SRC				+= TMC-API/tmc/ic/TMC4330/TMC4330.c
SRC				+= TMC-API/tmc/ic/TMC4331/TMC4331.c
SRC				+= TMC-API/tmc/ic/TMC4361/TMC4361.c
SRC				+= TMC-API/tmc/ic/TMC4361A/TMC4361A.c
SRC				+= TMC-API/tmc/ic/TMC4670/TMC4670.c
SRC				+= TMC-API/tmc/ic/TMC4671/TMC4671.c
SRC				+= TMC-API/tmc/ic/TMC5031/TMC5031.c
SRC				+= TMC-API/tmc/ic/TMC5041/TMC5041.c
SRC				+= TMC-API/tmc/ic/TMC5062/TMC5062.c
SRC				+= TMC-API/tmc/ic/TMC5072/TMC5072.c
SRC 			+= TMC-API/tmc/ic/TMC5130/TMC5130.c
SRC 			+= TMC-API/tmc/ic/TMC5160/TMC5160.c
SRC				+= TMC-API/tmc/ic/TMC6100/TMC6100.c
SRC				+= TMC-API/tmc/ic/TMC6200/TMC6200.c
SRC				+= TMC-API/tmc/ic/TMC7300/TMC7300.c
SRC				+= TMC-API/tmc/ic/TMC8461/TMC8461.c
SRC				+= TMC-API/tmc/ic/TMC8462/TMC8462.c

EXTRAINCDIRS 	+= hal/$(DEVICE)

### Chip-specific variables and files ###
# Startrampe
ifeq ($(DEVICE),Startrampe)
    CDEFS = -DStartrampe
    MCU      			= cortex-m3
    SUBMDL   			= STM32F205VCT6
    CHIP     			= $(SUBMDL)
    BOARD    			= STARTRAMPE
    STMLIBDIR 			= hal/$(DEVICE)/st
    STMSPDDIR 			= $(STMLIBDIR)/lib
    STMSPDSRCDIR 		= $(STMSPDDIR)/src
    STMSPDINCDIR 		= $(STMSPDDIR)/inc
    #CMSISDIR 			= $(STMLIBDIR)/CMSIS/Core/CM3
    #STMEEEMULDIR 		= $(STMLIBDIR)/EEPROMEmulation_AN
    #STMEEEMULSRCDIR 	= $(STMEEEMULDIR)/source
    #STMEEEMULINCDIR 	= $(STMEEEMULDIR)/include
    INCLUDE_DIRS 		= -I$(STMSPDINCDIR)

  	SRC 				+= $(STMLIBDIR)/system_stm32f2xx.c

    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_adc.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_can.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_dma.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_exti.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_flash.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_gpio.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_i2c.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_it.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_iwdg.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_rcc.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_rtc.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_spi.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_syscfg.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_tim.c
    SRC 				+= $(STMSPDSRCDIR)/stm32f2xx_usart.c
    SRC					+= $(STMSPDSRCDIR)/stm32f2xx_wwdg.c
    SRC 				+= $(STMSPDSRCDIR)/misc.c
    SRC 				+= $(STMSPDSRCDIR)/usbd_core.c
    SRC 				+= $(STMSPDSRCDIR)/usbd_cdc_core.c
    SRC 				+= $(STMSPDSRCDIR)/usbd_ioreq.c
    SRC 				+= $(STMSPDSRCDIR)/usbd_req.c
    SRC 				+= $(STMSPDSRCDIR)/usb_core.c
    SRC 				+= $(STMSPDSRCDIR)/usb_dcd.c
    SRC 				+= $(STMSPDSRCDIR)/usb_dcd_int.c

    ASRC 				+= $(STMLIBDIR)/startup.S
   	EXTRAINCDIRS  		+= $(STMSPDINCDIR)

   	ifeq ($(LINK),BL)
		LDFLAGS +=-T $(STMLIBDIR)/stm32f2xx-tmcm.ld
	else
		LDFLAGS +=-T $(STMLIBDIR)/stm32f2xx.ld
	endif

# Landungsbrücke
else ifeq ($(DEVICE),Landungsbruecke)
    CDEFS = -DLandungsbruecke
  	CPU					= K20DN512
    MCU      			= cortex-m4
    SUBMDL   			= MK20DN512
    CHIP     			= $(SUBMDL)
    BOARD    			= LANDUNGSBRUECKE
    LPCLIBDIR 			= hal/$(DEVICE)/freescale
    INCLUDE_DIRS 		= -I$(LPCLIBDIR)

    SRC 				+= $(LPCLIBDIR)/Cpu.c
    SRC 				+= $(LPCLIBDIR)/kinetis_sysinit.c
    SRC 				+= $(LPCLIBDIR)/nvic-m4.c

    SRC 				+= $(LPCLIBDIR)/USB_CDC/CDC1.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/CS1.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/Rx1.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/Tx1.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_cdc.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_cdc_pstn.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_class.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_dci_kinetis.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_descriptor.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_driver.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/usb_framework.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/USB0.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/USB1.c
    SRC 				+= $(LPCLIBDIR)/USB_CDC/wdt_kinetis.c
    ASRC 				+= $(LPCLIBDIR)/startup.S
    EXTRAINCDIRS  		+= $(LPCLIBDIR)

    ifeq ($(LINK),BL)
		LDFLAGS +=-T $(LPCLIBDIR)/MK20DN512-TMCM.ld
	else
		LDFLAGS +=-T $(LPCLIBDIR)/MK20DN512.ld
	endif
else
    $(error You need to set the DEVICE parameter to either "Landungsbruecke" or "Startrampe". When calling make directly, do this by adding DEVICE=Landungsbruecke or DEVICE=Startrampe to the commandline)
endif

CDEFS += -DID_CH1_DEFAULT=$(ID_CH1_DEFAULT) -DID_CH1_OVERRIDE=$(ID_CH1_OVERRIDE)
CDEFS += -DID_CH2_DEFAULT=$(ID_CH2_DEFAULT) -DID_CH2_OVERRIDE=$(ID_CH2_OVERRIDE)

CDEFS += -DBUILD_VERSION=$(subst .,,$(VERSION))

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
## just for testing, timer.c could be compiled in thumb-mode too
SRCARM =

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC =

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM =

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.

ASRC +=

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM =

# List any extra directories to look for include files here.
#    Each directory must be separated by a space.
EXTRAINCDIRS  += TMC-API
# EXTRAINCDIRS  += "${GCC_HOME}/lib/gcc/arm-none-eabi/6.3.1/include"

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be separated by a space.
EXTRA_LIBDIRS +=

# Extra libraries
#    Each library-name must be separated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a:
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS =

# Path to Linker-Scripts
LINKERSCRIPTPATH = .

### Toolchain ###
#TCHAIN_PREFIX 			= arm-eabi-
#TCHAIN_PREFIX 			= arm-elf-
TCHAIN_PREFIX 			= arm-none-eabi-
REMOVE_CMD				= rm
FLASH_TOOL 				= OPENOCD
#FLASH_TOOL 			= LPC21ISP
#FLASH_TOOL 			= UVISION
USE_THUMB_MODE 			= YES
#USE_THUMB_MODE 		= NO
RUN_MODE				= ROM_RUN
#RUN_MODE				= RAM_RUN
VECTOR_TABLE_LOCATION	= VECT_TAB_ROM
#VECTOR_TABLE_LOCATION	= VECT_TAB_RAM

### Build Settings ###
# Output selection
#LOADFORMAT 	= ihex
#LOADFORMAT 	= binary
LOADFORMAT 		= both

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s
#OPT = 2
#OPT = 3
#OPT = 0

# Using the Atmel AT91_lib produces warnings with
# the default warning-levels.
#  yes - disable these warnings
#  no  - keep default settings
#AT91LIBNOWARN = yes
AT91LIBNOWARN = no

# Debugging format.
#DEBUG = stabs
DEBUG = dwarf-2

# Place project-specific -D (define) and/or
# -U options for C here.
#CDEFS = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
#CDEFS += -DUSE_EK_STM32F -DSTARTUP_DELAY
#CDEFS += -DSTM32_USE_DMA
# enable modifications in STM's libraries
#CDEFS += -DMOD_MTHOMAS_STMLIB
# enable modifications in ChaN's FAT-module
##CDEFS += -DMOD_MTHOMAS_FFAT
# enable debug-support in STM's library
#CDEFS += -DUSE_FULL_ASSERT

ifeq ($(LINK),BL)
	CDEFS += -DBOOTLOADER
endif

# Place project-specific -D and/or -U options for
# Assembler with preprocessor here.
#ADEFS = -DUSE_IRQ_ASM_WRAPPER
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-----

ifdef VECTOR_TABLE_LOCATION
    CDEFS += -D$(VECTOR_TABLE_LOCATION)
    ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif

CDEFS += -D$(RUN_MODE) -D$(CHIP)
ADEFS += -D$(RUN_MODE) -D$(CHIP)

### Compiler flags ###

ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
### no for CM3 THUMB_IW = -mthumb-interwork
else
THUMB    =
THUMB_IW =
endif

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW)
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
##CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used
##CFLAGS += -mapcs-frame
##CFLAGS += -fomit-frame-pointer
#CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
#CFLAGS += -pedantic
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MMD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs
CONLYFLAGS += $(CSTANDARD)

ifeq ($(AT91LIBNOWARN),yes)
# compiling the AT91-lib thows warnings with the followins settings
# so they are enabled only if AT91LIBNOWARN is set to "no"
CFLAGS += -Wno-cast-qual
CONLYFLAGS += -Wno-missing-prototypes
CONLYFLAGS += -Wno-strict-prototypes
CONLYFLAGS += -Wno-missing-declarations
endif

# flags only for C++ (arm-elf-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS =

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS += -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
#CPLUSPLUS_LIB = -lstdc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -Wl,--gc-sections,-Map=$(OUTDIR)/$(TARGET).map,-cref
LDFLAGS += -u,Reset_Handler
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += -lc
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += -lc -lgcc
LDFLAGS += $(INCLUDE_DIRS)

# ---------------------------------------------------------------------------
# Options for lpc21isp by Martin Maurer
# lpc21isp only supports NXP LPC and Analog ADuC ARMs though the
# integrated uart-bootloader (ISP)
#
# Settings and variables:
LPC21ISP = lpc21isp
LPC21ISP_FLASHFILE = $(OUTDIR)/$(TARGET).hex
LPC21ISP_PORT = com1
LPC21ISP_BAUD = 57600
LPC21ISP_XTAL = 14746
# other options:
# -debug: verbose output
# -control: enter bootloader via RS232 DTR/RTS (only if hardware
#           supports this feature - see NXP AppNote)
#LPC21ISP_OPTIONS = -control
#LPC21ISP_OPTIONS += -debug
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Options for OpenOCD flash-programming
# see openocd.pdf/openocd.texi for further information
#
OOCD_LOADFILE+=$(OUTDIR)/$(TARGET).elf
# if OpenOCD is in the $PATH just set OPENOCDEXE=openocd
OOCD_EXE=OpenOCD/bin/openocd
# debug level
OOCD_CL=-d0
#OOCD_CL=-d3
# interface and board/target settings (using the OOCD target-library here)
## OOCD_CL+=-c "fast enable"
OOCD_CL+=-f interface/jtagkey.cfg -f board/stm32f10x_128k_eval.cfg
OOCD_CL+=-c init -c targets
# commands to prepare flash-write
OOCD_CL+=-c "halt"
# flash-write and -verify
OOCD_CL+=-c "flash write_image erase $(OOCD_LOADFILE)" -c "verify_image $(OOCD_LOADFILE)"
# reset target
OOCD_CL+=-c "reset run"
# terminate OOCD after programming
OOCD_CL+=-c shutdown
# ---------------------------------------------------------------------------


# Define programs and commands.
SHELL   = sh
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
##COPY    = cp
REMOVE  = $(REMOVE_CMD) -f

# Define Messages
# English
MSG_BEGIN = "-------- begin (mode: $(RUN_MODE)) --------"
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = "**** Linking :"
MSG_COMPILING = "**** Compiling C :"
MSG_COMPILING_ARM = "**** Compiling C (ARM-only):"
MSG_COMPILINGCPP = "Compiling C++ :"
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = "**** Assembling:"
MSG_ASSEMBLING_ARM = "****Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

### Make targets ###
# Default target. Build everything
all: begin gccversion build size end

# Target: clean project.
clean: begin clean_list end

# Helper targets: This makes selecting all output types easier (see build target below)
elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

# Select output file types
ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)

# Display sizes of sections.
ELFSIZE = $(SIZE) -A -x  $(OUTDIR)/$(TARGET).elf

size : elf
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)

# Display compiler version information.
gccversion :
	@$(CC) --version

# Program the device.
ifeq ($(FLASH_TOOL),UVISION)
# Program the device with Keil's uVision (needs configured uVision-workspace).
program: hex
##  @echo
	@echo "Programming with uVision"
	C:\Keil\uv3\Uv3.exe -f uvisionflash.Uv2 -ouvisionflash.txt
else
ifeq ($(FLASH_TOOL),OPENOCD)
# Program the device with Dominic Rath's OPENOCD in "batch-mode", needs cfg and "reset-script".
program: elf
	@echo "Programming with OPENOCD"
	$(OOCD_EXE) $(OOCD_CL)
else
# Program the device using lpc21isp (for NXP2k and ADuC UART bootloader)
program: hex
##  @echo
	@echo $(MSG_LPC21_RESETREMINDER)
	-$(LPC21ISP) $(LPC21ISP_OPTIONS) $(LPC21ISP_FLASHFILE) $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)
endif
endif

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
##  @echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@

# Create final output file (.bin) from ELF output file.
%.bin: %.elf
##  @echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
##  @echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@
# $(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
##  @echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
# $(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)

clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)/$(TARGET).map
	$(REMOVE) $(OUTDIR)/$(TARGET).elf
	$(REMOVE) $(OUTDIR)/$(TARGET).hex
	$(REMOVE) $(OUTDIR)/$(TARGET).bin
	$(REMOVE) $(OUTDIR)/$(TARGET).sym
	$(REMOVE) $(OUTDIR)/$(TARGET).lss
	$(REMOVE) $(ALLOBJ)
	$(REMOVE) $(LSTFILES)
	$(REMOVE) $(DEPFILES)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(CPPSRC:.cpp=.s)
	$(REMOVE) $(CPPSRCARM:.cpp=.s)


### Make recipe templates for all source file types ###
# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_ASSEMBLING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_ASSEMBLING_ARM) $$< "->" $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src))))


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_COMPILING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_COMPILING_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src))))


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_COMPILINGCPP) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) Makefile
##  @echo
	@echo $(MSG_COMPILINGCPP_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src))))


# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Create output directories
# Store the result to avoid printing warning messages
tmp := $(shell mkdir $(OUTDIR) 2>&1)
tmp := $(shell cd $(OUTDIR) && mkdir dep 2>&1)

# Include the dependency files.
-include $(wildcard $(OUTDIR)/dep/*)

# Listing of phony targets.
.PHONY : all begin end size gccversion \
build elf hex bin lss sym clean clean_list program
