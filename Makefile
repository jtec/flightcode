# Makefile for the compilation and linking of a c++ project for
# the STM32 microcontroller

# Toolchain prefix (i.e arm-elf- -> arm-elf-gcc.exe)
#TCHAIN_PREFIX = arm-eabi-
#TCHAIN_PREFIX = arm-elf-
TCHAIN_PREFIX = arm-none-eabi-
#REMOVE_CMD=rm
REMOVE_CMD=rm

# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC and -mthumb-interwork option for all source
USE_THUMB_MODE = YES
#USE_THUMB_MODE = NO

# MCU name, submodel and board
# - MCU used for compiler-option (-mcpu)
MCU = cortex-m4
# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTPUT_DIRECTORY = ./bin

# Target file name (without extension).
TARGET = main

# define Paths
CODE_DIRECTORY = ./code

APPLICATION_LAYER_DIRECTORY = $(CODE_DIRECTORY)/applicationLayer
CONVENIENCE_LAYER_DIRECTORY = ./code/convenienceLayer
HARDWAREACCESS_LAYER_DIRECTORY = ./code/hardwareAccessLayer

STM32_LIBRARY_INC_DIRECTORY = $(HARDWAREACCESS_LAYER_DIRECTORY)/STM32F4xx_StdPeriph_Driver/inc
STM32_LIBRARY_SRC_DIRECTORY = $(HARDWAREACCESS_LAYER_DIRECTORY)/STM32F4xx_StdPeriph_Driver/src
LIBRARIES_DIRECTORY = $(CODE_DIRECTORY)/libraries
CPP_LIB_DIRECTORY = $(LIBRARIES_DIRECTORY)/CPP
CPP_LIB_SRC_DIRECTORY = $(CPP_LIB_DIRECTORY)/src
CPP_LIB_INC_DIRECTORY = $(CPP_LIB_DIRECTORY)/inc
CMSIS_DIRECTORY  = $(HARDWAREACCESS_LAYER_DIRECTORY)/CMSIS

# List C source files here.
SRC = $(HARDWAREACCESS_LAYER_DIRECTORY)/startup_STM32F4.c
SRC += $(LIBRARIES_DIRECTORY)/syscalls.c 
SRC += $(HARDWAREACCESS_LAYER_DIRECTORY)/CMSIS/ST/STM32F4xx/Source/Templates/system_stm32f4xx_8MHz.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/misc.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_rcc.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_usart.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_gpio.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_dma.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_tim.c
SRC += $(STM32_LIBRARY_SRC_DIRECTORY)/stm32f4xx_i2c.c

# List C++ source files here.
# use file-extension .cpp for C++-files.
CPPSRC = $(APPLICATION_LAYER_DIRECTORY)/src/main.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/init.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alMAVLink.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alBus.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alStopwatch.cpp

CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alMainNode.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alStandaloneNode.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alServoNode.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alGroundStation.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alGroundLinkNode.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/alPeriodicPingNode.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/PID.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/ParameterManager.cpp
CPPSRC += $(APPLICATION_LAYER_DIRECTORY)/src/GroundLink.cpp

CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clCircularBuffer.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clLED.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clGPIOPin.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clAssistant.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clTimeBase.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clInterruptManager.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clSerialPort.cpp
CPPSRC += $(CPP_LIB_SRC_DIRECTORY)/mini_cpp.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clPWMServoBank.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clSpektrumSatellite.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clFactory.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clHVD78.cpp
CPPSRC += $(CONVENIENCE_LAYER_DIRECTORY)/src/clMPU6050.cpp


# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  = $(STM32_LIBRARY_INC_DIRECTORY) $(CMSIS_DIRECTORY) $(FATSDDIR) $(MININIDIR) $(STMEEEMULINCDIR)
EXTRAINCDIRS += $(CODE_DIRECTORY) $(SWIMSRCDIR) $(CODE_DIRECTORY)/applicationLayer/inc $(CODE_DIRECTORY)/convenienceLayer/inc

# Extra libraries
# Each library-name must be separated by a space.
# a parameter 'xyz' makes the linker look for files of the name libxyz.a
#EXTRA_LIBS = c gcc m 
EXTRA_LIBDIRS =  ./code/libraries  $(STM32_LIBRARY_INC_DIRECTORY)  

# Path to Linker-Scripts
LINKERSCRIPTPATH = ./code/hardwareAccessLayer/linkerskripts

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 0

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format i.e. for lpc21isp
#LOADFORMAT = ihex
#LOADFORMAT = binary
LOADFORMAT = both

# Debugging format.
#DEBUG = stabs
DEBUG = dwarf-2

# Place project-specific -D (define) and/or 
# -U options for C here.
#CDEFS += -DUSE_FULL_ASSERT
#HSE_VALUE = 12000000
#CDEFS += -DHSE_VALUE


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

# Compiler flags.

ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
THUMB_IW = -mthumb-interwork
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
CFLAGS += $(CDEFS)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW) 
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used 
#CFLAGS += -mapcs-frame 
#CFLAGS += -fomit-frame-pointer
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
# CFLAGS += -pedantic
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTPUT_DIRECTORY)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(OUTPUT_DIRECTORY)/dep/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-*-g++)
CPPFLAGS = -fno-rtti -fno-exceptions -Wno-pmf-conversions -fno-threadsafe-statics

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
CPLUSPLUS_LIB = -lstdc++
#CPLUSPLUS_LIB += -lsupc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = 
LDFLAGS += -Wl,-Map=$(OUTPUT_DIRECTORY)/$(TARGET).map,--cref,--gc-sections
#not in CPP
#LDFLAGS += -nostartfiles
LDFLAGS += -lc
LDFLAGS += $(MATH_LIB)
LDFLAGS += -lc -lgcc
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += $(patsubst %,-L%, $(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS)) 
LDFLAGS += -nostartfiles -nodefaultlibs
#LDFLAGS += -v

# Set linker-script name depending on selected run-mode and submodel name
ifeq ($(RUN_MODE),RAM_RUN)
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP)_ram.ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/sram.lds
else 
LDFLAGS +=-T$(LINKERSCRIPTPATH)/stm32F407VG_flash.ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/flash.lds
endif


# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
REMOVE  = $(REMOVE_CMD) -f
SHELL   = sh
###COPY    = cp
ifneq ($(or $(COMSPEC), $(ComSpec)),)
$(info COMSPEC detected $(COMSPEC) $(ComSpec))
ifeq ($(findstring cygdrive,$(shell env)),)
SHELL:=$(or $(COMSPEC),$(ComSpec))
SHELL_IS_WIN32=1
else
$(info cygwin detected)
endif
endif
$(info SHELL is $(SHELL))

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = --------  begin, mode: $(RUN_MODE)  --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = ---- Linking :
MSG_COMPILING = ---- Compiling C :
MSG_COMPILING_ARM = ---- Compiling C ARM-only:
MSG_COMPILINGCPP = ---- Compiling C++ :
MSG_COMPILINGCPP_ARM = ---- Compiling C++ ARM-only:
MSG_ASSEMBLING = ---- Assembling:
MSG_ASSEMBLING_ARM = ---- Assembling ARM-only:
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
ALLOBJ     = $(addprefix $(OUTPUT_DIRECTORY)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTPUT_DIRECTORY)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTPUT_DIRECTORY)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

# Default target.
all: begin createdirs gccversion g++version build sizeafter dump finished end

elf: $(OUTPUT_DIRECTORY)/$(TARGET).elf
lss: $(OUTPUT_DIRECTORY)/$(TARGET).lss 
sym: $(OUTPUT_DIRECTORY)/$(TARGET).sym
hex: $(OUTPUT_DIRECTORY)/$(TARGET).hex
bin: $(OUTPUT_DIRECTORY)/$(TARGET).bin


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

# Create output directories.
createdirs:
	-@md $(OUTPUT_DIRECTORY) >NUL 2>&1 || echo "" >NUL
	-@md $(OUTPUT_DIRECTORY)\dep >NUL 2>&1 || echo "" >NUL

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

finished:
##	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)

# Display sizes of sections.
ELFSIZE = $(SIZE) -A  $(OUTPUT_DIRECTORY)/$(TARGET).elf

sizeafter:
#	@if [ -f  $(OUTPUT_DIRECTORY)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)
	
# Display gcc compiler version information.
gccversion : 
	@$(CC) --version
#	@echo $(ALLOBJ)

# Display g++ compiler version information.
g++version : 
	@$(CPP) --version
#	@echo $(ALLOBJ)

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
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
ifeq "$(strip $(CPPSRC)$(CPPARM))" ""
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ -nostartfiles $(LDFLAGS)
else
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
endif

# Display detailed structure of .elf file.
dump:
	$(OBJDUMP) -h $(OUTPUT_DIRECTORY)/$(TARGET).elf


# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	@echo "hallo"
	@echo $(MSG_ASSEMBLING) $$< to $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING_ARM) $$< to $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$< to $$@
	$(info using gcc)
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING_ARM) $$< to $$@
	$(info using gcc ARM-only)	
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	$(info using g++)
	@echo $(MSG_COMPILINGCPP) $$< to $$@
	$(CPP) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTPUT_DIRECTORY)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP_ARM) $$< to $$@
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

# Target: clean project.
clean: begin clean_list finished end

clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).map
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).elf
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).hex
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).bin
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).sym
	$(REMOVE) $(OUTPUT_DIRECTORY)/$(TARGET).lss
	$(REMOVE) $(ALLOBJ)
	$(REMOVE) $(LSTFILES)
	$(REMOVE) $(DEPFILES)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(CPPSRC:.cpp=.s)
	$(REMOVE) $(CPPSRCARM:.cpp=.s)

## Create object files directory - now done if special make target
##$(shell mkdir $(OBJDIR) 2>/dev/null)

# Include the dependency files.
##-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)
-include $(wildcard dep/*)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program createdirs

