### Generic settings ###
#VERSION			= 3.06
include version.txt
#DEVICE			= Landungsbruecke
#DEVICE			= LandungsbrueckeSmall
#DEVICE			= LandungsbrueckeV3
LINK			= BL
#LINK			= NOBL
BUILD_DIR       = _build
DEVICE_DIR 	    = $(BUILD_DIR)/_build_$(DEVICE)
TARGET 			= $(DEVICE)_v$(VERSION)_$(LINK)

### Build output configuration #################################################
DEP_DIR     := $(DEVICE_DIR)/dep
OBJ_DIR     := $(DEVICE_DIR)/obj

# Force IDs (IDs found in tmc/BoardAssignment.h)
# If OVERRIDE is set, any autodetected ID will be discarded
# Else, autodetected IDs preceed ID_CH1_DEFAULT / ID_CH2_DEFAULT
ID_CH1_DEFAULT	?= 0
ID_CH1_OVERRIDE	?= false
ID_CH2_DEFAULT	?= 0
ID_CH2_OVERRIDE	?= false

# Select whether USB should report a unique serial number or not
# Legacy behaviour for the Landungsbruecke <= v2 is to not have a serial string.
# For when this option is enabled for compatibility, on Landungsbruecke v3 the
# serial string will always have the value "TMCEVAL", giving the same behaviour.
# Note: This behaviour will eventually be changed to proper serial number strings.
USB_USE_UNIQUE_SERIAL_NUMBER ?= false

### Source File Selection ###
# Evalboards
SRC 			+= boards/Board.c
SRC 			+= boards/TMCDriver.c
SRC 			+= boards/TMCMotionController.c

SRC 			+= boards/Rhino_standalone.c
SRC				+= boards/TMC2100_eval.c
SRC				+= boards/TMC2130_eval.c
SRC				+= boards/TMC2160_eval.c
SRC				+= boards/TMC2208_eval.c
SRC				+= boards/TMC2210_eval.c
SRC				+= boards/TMC2224_eval.c
SRC				+= boards/TMC2240_eval.c
SRC				+= boards/TMC2660_eval.c
SRC				+= boards/TMC4361A_eval.c
SRC				+= boards/TMC4671_eval.c
SRC				+= boards/TMC5031_eval.c
SRC				+= boards/TMC5041_eval.c
SRC				+= boards/TMC5062_eval.c
SRC				+= boards/TMC5072_eval.c
SRC 			+= boards/TMC5130_eval.c
SRC 			+= boards/TMC5160_eval.c
SRC 			+= boards/TMC5240_eval.c
SRC 			+= boards/TMC5271_eval.c
SRC 			+= boards/TMC5272_eval.c
SRC 			+= boards/TMC6100_eval.c
SRC 			+= boards/TMC6200_eval.c
SRC				+= boards/TMC7300_eval.c
ifeq ($(DEVICE),$(filter $(DEVICE),Landungsbruecke LandungsbrueckeSmall LandungsbrueckeV3))
SRC				+= boards/MAX22216_eval.c
SRC				+= boards/MAX22204_eval.c
SRC				+= boards/MAX22210_eval.c
SRC				+= boards/TMC2209_eval.c
SRC				+= boards/TMC2225_eval.c
SRC             += boards/TMC2226_eval.c
SRC				+= boards/TMC2300_eval.c
SRC             += boards/TMC6300_eval.c
SRC             += boards/TMC6140_eval.c
SRC             += boards/TMC8100_eval.c
SRC				+= boards/TMC5262_eval.c
SRC				+= boards/TMC2262_eval.c
SRC             += boards/TMC9660_3PH_eval.c
SRC             += boards/TMC9660_STEPPER_eval.c

endif

# Control
SRC 			+= main.c
SRC 			+= tmc/TMCL.c
SRC 			+= tmc/RAMDebug.c
SRC				+= tmc/EEPROM.c
SRC 			+= tmc/BoardAssignment.c
SRC 			+= tmc/VitalSignsMonitor.c
SRC 			+= tmc/StepDir.c
ifeq ($(DEVICE),$(filter $(DEVICE),Landungsbruecke LandungsbrueckeSmall))
SRC             += tmc/BLDC_Landungsbruecke.c
endif
ifeq ($(DEVICE),$(filter $(DEVICE),LandungsbrueckeV3))
SRC             += tmc/BLDC_LandungsbrueckeV3.c
endif

# TMC_API
SRC				+= TMC-API/tmc/helpers/Functions.c
SRC				+= TMC-API/tmc/helpers/CRC.c
SRC				+= TMC-API/tmc/ramp/LinearRamp.c
SRC				+= TMC-API/tmc/ramp/LinearRamp1.c
SRC				+= TMC-API/tmc/ramp/Ramp.c
SRC 			+= TMC-API/tmc/ic/MAX22216/MAX22216.c
SRC 			+= TMC-API/tmc/ic/TMC2130/TMC2130.c
SRC 			+= TMC-API/tmc/ic/TMC2160/TMC2160.c
SRC 			+= TMC-API/tmc/ic/TMC2208/TMC2208.c
SRC 			+= TMC-API/tmc/ic/TMC2209/TMC2209.c
SRC 			+= TMC-API/tmc/ic/TMC2224/TMC2224.c
SRC 			+= TMC-API/tmc/ic/TMC2225/TMC2225.c
SRC 			+= TMC-API/tmc/ic/TMC2226/TMC2226.c
SRC 			+= TMC-API/tmc/ic/TMC2240/TMC2240.c
SRC 			+= TMC-API/tmc/ic/TMC2660/TMC2660.c
SRC 			+= TMC-API/tmc/ic/TMC2300/TMC2300.c
SRC				+= TMC-API/tmc/ic/TMC4361A/TMC4361A.c
SRC				+= TMC-API/tmc/ic/TMC4671/TMC4671.c
SRC				+= TMC-API/tmc/ic/TMC5031/TMC5031.c
SRC				+= TMC-API/tmc/ic/TMC5041/TMC5041.c
SRC				+= TMC-API/tmc/ic/TMC5062/TMC5062.c
SRC				+= TMC-API/tmc/ic/TMC5072/TMC5072.c
SRC 			+= TMC-API/tmc/ic/TMC5130/TMC5130.c
SRC 			+= TMC-API/tmc/ic/TMC5160/TMC5160.c
SRC 			+= TMC-API/tmc/ic/TMC5240/TMC5240.c
SRC 			+= TMC-API/tmc/ic/TMC5271/TMC5271.c
SRC 			+= TMC-API/tmc/ic/TMC5272/TMC5272.c
SRC				+= TMC-API/tmc/ic/TMC6100/TMC6100.c
SRC				+= TMC-API/tmc/ic/TMC6200/TMC6200.c
SRC				+= TMC-API/tmc/ic/TMC7300/TMC7300.c
SRC 			+= TMC-API/tmc/ic/TMC5262/TMC5262.c
SRC 			+= TMC-API/tmc/ic/TMC2262/TMC2262.c

EXTRAINCDIRS 	+= $(TMC_HAL_SRC)

### Chip-specific variables and files ###
# Landungsbrücke
ifeq ($(DEVICE),Landungsbruecke)
    CDEFS = -DLandungsbruecke
    MCU      			= cortex-m4
    SUBMDL   			= MK20DN512
    CHIP     			= $(SUBMDL)
    BOARD    			= LANDUNGSBRUECKE
    TMC_HAL_SRC         = hal/Landungsbruecke
    LPCLIBDIR 			= $(TMC_HAL_SRC)/freescale
    INCLUDE_DIRS 		= -I$(LPCLIBDIR)

    SRC                 += boards/SelfTest_Landungsbruecke.c

    SRC                 += tmc/IdDetection_Landungsbruecke.c

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
		LD_SCRIPT = $(LPCLIBDIR)/MK20DN512-TMCM.ld
	else
		LD_SCRIPT = $(LPCLIBDIR)/MK20DN512.ld
	endif
# Landungsbrücke (small)
else ifeq ($(DEVICE),LandungsbrueckeSmall)
    CDEFS = -DLandungsbrueckeSmall
    MCU      			= cortex-m4
    SUBMDL   			= MK20DX256
    CHIP     			= $(SUBMDL)
    BOARD    			= LANDUNGSBRUECKE
    TMC_HAL_SRC         = hal/Landungsbruecke
    LPCLIBDIR 			= $(TMC_HAL_SRC)/freescale
    INCLUDE_DIRS 		= -I$(LPCLIBDIR)

    SRC                 += boards/SelfTest_Landungsbruecke.c

    SRC                 += tmc/IdDetection_Landungsbruecke.c

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
		LD_SCRIPT = $(LPCLIBDIR)/MK20DX256-TMCM.ld
	else
		LD_SCRIPT = $(LPCLIBDIR)/MK20DX256.ld
	endif
else ifeq ($(DEVICE),LandungsbrueckeV3)
    CDEFS = -DLandungsbrueckeV3
    MCU      			= cortex-m4
    SUBMDL   			= GD32F425
    CHIP     			= $(SUBMDL)
    BOARD    			= LandungsbrueckeV3
    TMC_HAL_SRC         = hal/Landungsbruecke_V3
    STMLIBDIR 			= $(TMC_HAL_SRC)/GigaDevice
    STMSPDDIR 			= $(STMLIBDIR)/lib
    LIBSRCDIR 			= $(STMSPDDIR)/src
    STMSPDINCDIR 		= $(STMSPDDIR)/inc
    #CMSISDIR 			= $(STMLIBDIR)/CMSIS/Core/CM3
    #STMEEEMULDIR 		= $(STMLIBDIR)/EEPROMEmulation_AN
    #STMEEEMULSRCDIR 	= $(STMEEEMULDIR)/source
    #STMEEEMULINCDIR 	= $(STMEEEMULDIR)/include
    INCLUDE_DIRS 		= -I$(STMSPDINCDIR)

    SRC                 += boards/SelfTest_LandungsbrueckeV3.c
#
    SRC                 += tmc/IdDetection_LandungsbrueckeV3.c


	SRC += $(LIBSRCDIR)/system_gd32f4xx.c
	SRC += $(LIBSRCDIR)/gd32f4xx_adc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_can.c
	SRC += $(LIBSRCDIR)/gd32f4xx_crc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_ctc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dac.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dbg.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dci.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dma.c
	SRC += $(LIBSRCDIR)/gd32f4xx_enet.c
	SRC += $(LIBSRCDIR)/gd32f4xx_exmc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_exti.c
	SRC += $(LIBSRCDIR)/gd32f4xx_fmc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_fwdgt.c
	SRC += $(LIBSRCDIR)/gd32f4xx_gpio.c
    SRC += $(LIBSRCDIR)/gd32f4xx_hw.c
	SRC += $(LIBSRCDIR)/gd32f4xx_i2c.c
	SRC += $(LIBSRCDIR)/gd32f4xx_ipa.c
	SRC += $(LIBSRCDIR)/gd32f4xx_iref.c
	SRC += $(LIBSRCDIR)/gd32f4xx_misc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_pmu.c
	SRC += $(LIBSRCDIR)/gd32f4xx_rcu.c
	SRC += $(LIBSRCDIR)/gd32f4xx_rtc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_sdio.c
	SRC += $(LIBSRCDIR)/gd32f4xx_spi.c
	SRC += $(LIBSRCDIR)/gd32f4xx_syscfg.c
	SRC += $(LIBSRCDIR)/gd32f4xx_timer.c
	SRC += $(LIBSRCDIR)/gd32f4xx_tli.c
	SRC += $(LIBSRCDIR)/gd32f4xx_trng.c
	SRC += $(LIBSRCDIR)/gd32f4xx_usart.c
	SRC += $(LIBSRCDIR)/gd32f4xx_wwdgt.c
	SRC += $(LIBSRCDIR)/usb/cdc_acm_core.c
	SRC += $(LIBSRCDIR)/usb/drv_usbd_int.c
	SRC += $(LIBSRCDIR)/usb/drv_usb_core.c
	SRC += $(LIBSRCDIR)/usb/drv_usb_dev.c
	SRC += $(LIBSRCDIR)/usb/usbd_core.c
	SRC += $(LIBSRCDIR)/usb/usbd_enum.c
	SRC += $(LIBSRCDIR)/usb/usbd_transc.c
  	

	ASRC +=  $(LIBSRCDIR)/startup_gd32f405_425_gas.S
   	EXTRAINCDIRS  		+= $(STMSPDINCDIR)
    EXTRAINCDIRS  		+= $(STMSPDINCDIR)/usb
    EXTRAINCDIRS  		+= $(TMC_HAL_SRC)/tmc

   	ifeq ($(LINK),BL)
		LD_SCRIPT = $(STMLIBDIR)/gd32f425-tmcm.ld
	else
		LD_SCRIPT = $(STMLIBDIR)/gd32f425.ld
	endif
	LDFLAGS += -specs=nosys.specs
else
	ifeq (,$(filter clean,$(MAKECMDGOALS)))
		$(error You need to set the DEVICE parameter to "Landungsbruecke", "LandungsbrueckeSmall" or "LandungsbrueckeV3". When calling make directly, do this by adding DEVICE=Landungsbruecke, DEVICE=LandungsbrueckeV3 or DEVICE=LandungsbrueckeSmall to the commandline)
	endif
endif

# System and hardware abstraction layer
SRC 			+= $(TMC_HAL_SRC)/tmc/SysTick.c
SRC 			+= $(TMC_HAL_SRC)/tmc/IOs.c
SRC 			+= $(TMC_HAL_SRC)/tmc/IOMap.c
SRC 			+= $(TMC_HAL_SRC)/tmc/HAL.c
SRC 			+= $(TMC_HAL_SRC)/tmc/SPI.c
SRC 			+= $(TMC_HAL_SRC)/tmc/USB.c
SRC 			+= $(TMC_HAL_SRC)/tmc/ADCs.c
SRC 			+= $(TMC_HAL_SRC)/tmc/LEDs.c
SRC 			+= $(TMC_HAL_SRC)/tmc/RS232.c
SRC 			+= $(TMC_HAL_SRC)/tmc/WLAN.c
SRC 			+= $(TMC_HAL_SRC)/tmc/Timer.c
SRC 			+= $(TMC_HAL_SRC)/tmc/UART.c
SRC 			+= $(TMC_HAL_SRC)/tmc/RXTX.c


CDEFS += -DID_CH1_DEFAULT=$(ID_CH1_DEFAULT) -DID_CH1_OVERRIDE=$(ID_CH1_OVERRIDE)
CDEFS += -DID_CH2_DEFAULT=$(ID_CH2_DEFAULT) -DID_CH2_OVERRIDE=$(ID_CH2_OVERRIDE)

CDEFS += -DUSB_USE_UNIQUE_SERIAL_NUMBER=$(USB_USE_UNIQUE_SERIAL_NUMBER)

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
ifeq ($(OS),Windows_NT)
	ifeq ($(shell echo $$COMSPEC),$$COMSPEC)
#       We are on windows and make is using cmd.exe as shell
		create_dir = mkdir $(subst /,\,$(1))
		RM_DIR_CMD = rmdir /S /Q
	else
#       We are on windows but make found a sh.exe from a Git Bash or Cygwin
		create_dir = mkdir -p $(1)
		RM_DIR_CMD = rm -rf
	endif
else
#   Assueme we are on a linux machine
	create_dir = mkdir -p $(1)
	RM_DIR_CMD = rm -rf
endif
USE_THUMB_MODE 			= YES
#USE_THUMB_MODE 		= NO
RUN_MODE				= ROM_RUN
#RUN_MODE				= RAM_RUN
VECTOR_TABLE_LOCATION	= VECT_TAB_ROM
#VECTOR_TABLE_LOCATION	= VECT_TAB_RAM

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
OPT = s

# Debugging format.
#DEBUG = stabs
DEBUG = dwarf-2

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
CFLAGS += -DTMC_API_EXTERNAL_CRC_TABLE=1
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
CFLAGS += -Wa,-adhlns=$(addprefix $(OBJ_DIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MMD -MP -MF $(DEP_DIR)/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-elf-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS =

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS += -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OBJ_DIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
#CPLUSPLUS_LIB = -lstdc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -T $(LD_SCRIPT)
LDFLAGS += -nostdlib
LDFLAGS += -Wl,--gc-sections,-Map=$(OUTDIR)/$(TARGET).map,-cref
LDFLAGS += -u,Reset_Handler
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += -lc
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += -lc -lgcc
LDFLAGS += $(INCLUDE_DIRS)

# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm

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
ALLOBJ     = $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OBJ_DIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(DEP_DIR)/, $(addsuffix .o.d, $(ALLSRCBASE)))

### Make targets ###
# Default target. Build everything
all: begin gccversion build size end

# Target: clean project.
clean: begin clean_list end

# Helper targets: This makes selecting all output types easier (see build target below)
elf: $(DEVICE_DIR)/$(TARGET).elf
lss: $(DEVICE_DIR)/$(TARGET).lss
sym: $(DEVICE_DIR)/$(TARGET).sym
hex: $(DEVICE_DIR)/$(TARGET).hex
bin: $(DEVICE_DIR)/$(TARGET).bin

build: elf hex bin lss sym

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)

# Display sizes of sections.
ELFSIZE = $(SIZE) -A -x  $(DEVICE_DIR)/$(TARGET).elf

size : elf
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)

# Display compiler version information.
gccversion :
	@$(CC) --version

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@

# Create final output file (.bin) from ELF output file.
%.bin: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf $(ALLOBJ)
%.elf:  $(ALLOBJ) $(LD_SCRIPT)
	@echo
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
# $(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)

clean_list :
	@echo $(MSG_CLEANING)
	$(RM_DIR_CMD) $(BUILD_DIR)

### Make recipe templates for all source file types ###
# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
	@echo $(MSG_ASSEMBLING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
	@echo $(MSG_ASSEMBLING_ARM) $$< "->" $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src))))

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
	@echo $(MSG_COMPILING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
	@echo $(MSG_COMPILING_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src))))

# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
	@echo $(MSG_COMPILINGCPP) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OBJ_DIR)/$(notdir $(basename $(1))).o : $(1) Makefile
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
tmp := $(shell $(call create_dir,$(BUILD_DIR)) 2>&1)
tmp := $(shell $(call create_dir,$(DEVICE_DIR)) 2>&1)
tmp := $(shell $(call create_dir,$(OBJ_DIR)) 2>&1)
tmp := $(shell $(call create_dir,$(DEP_DIR)) 2>&1)

# Include the dependency files.
-include $(wildcard $(DEP_DIR)/*)

# Listing of phony targets.
.PHONY : all begin end size gccversion \
build elf hex bin lss sym clean clean_list program
