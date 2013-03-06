#******************************************************************************
# @file      Makefile
# @author    Stefano Oliveri (software@stf12.net)
# @version   V2.0
# @date      22/06/2009
# @copy
#
# THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING USERS
# WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
# TIME. AS A RESULT, STEFANO OLIVERI SHALL NOT BE HELD LIABLE FOR ANY
# DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
# FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
# CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
#
# <h2><center>&copy; COPYRIGHT 2009 Stefano Oliveri</center></h2>
#******************************************************************************

# Project name
PROJECT_NAME=RTOSDemo


# Directory definition.
RTOS_SOURCE_DIR=./FreeRTOS/Source
DEMO_COMMON_DIR=./FreeRTOS/Common/Minimal
DEMO_INCLUDE_DIR=./FreeRTOS/Common/include
ST_LIB_DIR=./FreeRTOS/Common/drivers/ST/STM32F10x_StdPeriph_Lib_V3.5.0/STM32F10x_StdPeriph_Driver
ARM_CMSIS_DIR=./FreeRTOS/Common/drivers/ST/STM32F10x_StdPeriph_Lib_V3.5.0/CMSIS
RTOS_IO_SOURCE_DIR = ./FreeRTOS/FreeRTOS-Plus-IO

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.).
OUTDIR = ./Debug

# Toolchain definition.
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-size
NM = arm-none-eabi-nm

LDSCRIPT = ./Setup/stm32_flash.ld

# Startup file without extension!
STARTUP_FILE = startup_stm32f10x

# should use --gc-sections but the debugger does not seem to be able to cope with the option.
LINKER_FLAGS += -nostartfiles 
LINKER_FLAGS += -Xlinker -o$(OUTDIR)/$(PROJECT_NAME).axf 
LINKER_FLAGS += -Xlinker -M 
LINKER_FLAGS += -Xlinker -Map=$(PROJECT_NAME).map 
LINKER_FLAGS += -Xlinker --no-gc-sections
LINKER_FLAGS += -T$(LDSCRIPT)



# Debugging format.
#DEBUG = stabs
#DEBUG = dwarf-2
DEBUG= gdb

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 0


# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = gnu89


# Compiler flags definition.
CFLAGS += -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -std=$(CSTANDARD)
#CFLAGS += -T$(LDSCRIPT)
CFLAGS += -I .
CFLAGS += -I ./STCode
CFLAGS += -I $(RTOS_SOURCE_DIR)/include
CFLAGS += -I $(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3
CFLAGS += -I $(DEMO_INCLUDE_DIR)
CFLAGS += -I $(ST_LIB_DIR)/inc
CFLAGS += -I $(ARM_CMSIS_DIR)/CM3/CoreSupport
CFLAGS += -I $(ARM_CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CFLAGS += -I ./Setup
CFLAGS += -I ./Lcd
CFLAGS += -I $(RTOS_IO_SOURCE_DIR)/Include
CFLAGS += -I $(RTOS_IO_SOURCE_DIR)/Device/STM32F10x/BoardSupport
CFLAGS += -I ./Console
CFLAGS += -I ./FreeRTOS/FreeRTOS-Plus-CLI
CFLAGS += -I ./NandFlash
CFLAGS += -I ./NorFlash
CFLAGS += -I ./FSMC
CFLAGS += -I ./FSMC/fsmc_sram
CFLAGS += -I ./TouchScreen
CFLAGS += -I ./FatFs
CFLAGS += -I ./ADC
CFLAGS += -I ./RTC
CFLAGS += -D STM32F10X_HD
CFLAGS += -D USE_STDPERIPH_DRIVER
CFLAGS += -D VECT_TAB_FLASH
CFLAGS += -D GCC_ARMCM3
CFLAGS += -D inline=
CFLAGS += -D PACK_STRUCT_END=__attribute\(\(packed\)\)
CFLAGS += -D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\)
CFLAGS += -mthumb
CFLAGS += -mcpu=cortex-m3
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections

# The project is re-built when these files change
CONFIG_HEADER += FreeRTOSConfig.h
CONFIG_HEADER += FreeRTOSIOConfig.h
CONFIG_HEADER += FreeRTOS_IO_BSP.h
CONFIG_HEADER += stm32f10x_conf.h
CONFIG_HEADER += Makefile

# Source files
SOURCE += main.c
#SOURCE += timertest.c
SOURCE += ./Setup/stm32f10x_it.c
#SOURCE += stf_syscalls_minimal.c
SOURCE += printf-stdarg.c
SOURCE += ./ParTest/ParTest.c
#SOURCE += ./STCode/stm3210e_lcd.c
#SOURCE += ./serial/serial.c
SOURCE += delay_us.c

# ST Library source files.
ST_LIB_SOURCE += $(ARM_CMSIS_DIR)/CM3/CoreSupport/core_cm3.c 
ST_LIB_SOURCE += $(ARM_CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/misc.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_rcc.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_gpio.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_spi.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_tim.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_usart.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_fsmc.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_flash.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_exti.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_dma.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_sdio.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_adc.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_rtc.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_pwr.c 
ST_LIB_SOURCE += $(ST_LIB_DIR)/src/stm32f10x_bkp.c 

# FreeRTOS common demo source files.
FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/flash.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/blocktim.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/death.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/integer.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/PollQ.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/semtest.c
#FREERTOS_COMMON_DEMO_SOURCE += $(DEMO_COMMON_DIR)/comtest.c

# FreeRTOS source files.
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/list.c 
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/queue.c 
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/tasks.c 
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/timers.c 
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3/port.c
FREERTOS_SOURCE += $(RTOS_SOURCE_DIR)/portable/MemMang/heap_2.c
# FreeRTOS IO source files
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Device/STM32F10x/FreeRTOS_stm32f10x_DriverInterface.c
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Device/STM32F10x/FreeRTOS_stm32f10x_usart.c
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Common/FreeRTOS_DriverInterface.c
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Common/FreeRTOS_IOUtils.c
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Common/IOUtils_CharQueueTxAndRx.c 
FREERTOS_IO_SOURCE += $(RTOS_IO_SOURCE_DIR)/Common/IOUtils_CircularBufferRx.c 

FREERTOS_CLI_SOURCE += ./FreeRTOS/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c

LCD_SOURCE += ./Lcd/lcd.c
LCD_SOURCE += ./Lcd/LcdTask.c

CONSOLE_SOURCE += ./Console/CLI-commands.c
CONSOLE_SOURCE += ./Console/console.c

NAND_FLASH += ./NandFlash/nand_flash.c
NOR_FLASH  += ./NorFlash/nor_flash.c
#SRAM  += ./FSMC/fsmc_sram/fsmc_sram.c

FSMC_SOURCE += ./FSMC/FSMC_config.c

TOUCH_SOURCE += ./TouchScreen/touch_driver.c
TOUCH_SOURCE += ./TouchScreen/TouchTask.c

FATFS_SOURCE += ./FatFs/diskio.c
FATFS_SOURCE += ./FatFs/ff.c
#FATFS_SOURCE += ./FatFs/sdcard.c
FATFS_SOURCE += ./FatFs/stm32_sdio_sd.c
FATFS_SOURCE += ./FatFs/stm32_sdio_low_level.c

ADC_SOURCE += ./ADC/ADC_Task.c
RTC_SOURCE += ./RTC/RTC_Task.c

SOURCE += $(ST_LIB_SOURCE)
SOURCE += $(FREERTOS_COMMON_DEMO_SOURCE)
SOURCE += $(FREERTOS_SOURCE)
SOURCE += $(FREERTOS_IO_SOURCE)
SOURCE += $(FREERTOS_CLI_SOURCE)
SOURCE += $(LCD_SOURCE)
SOURCE += $(CONSOLE_SOURCE)
SOURCE += $(NAND_FLASH)
SOURCE += $(NOR_FLASH)
SOURCE += $(FSMC_SOURCE)
SOURCE += $(TOUCH_SOURCE)
SOURCE += $(FATFS_SOURCE)
SOURCE += $(ADC_SOURCE)
SOURCE += $(RTC_SOURCE)
SOURCE += $(SRAM)


# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(SOURCE)))


LIBS=

# List of all objects files.
OBJS = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))


# Define Messages.
# English
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------


# Rules definition. ***********************************************************

all: begin gccversion $(OUTDIR)/$(PROJECT_NAME).bin post end
	 
$(OUTDIR)/$(PROJECT_NAME).bin : $(OUTDIR)/$(PROJECT_NAME).axf
	$(OBJCOPY) $(OUTDIR)/$(PROJECT_NAME).axf -O binary $(OUTDIR)/$(PROJECT_NAME).bin

$(OUTDIR)/$(PROJECT_NAME).axf : $(OBJS) $(OUTDIR)/$(STARTUP_FILE).o
	$(CC) $(CFLAGS) $(OBJS) $(OUTDIR)/$(STARTUP_FILE).o $(LIBS) $(LINKER_FLAGS)
	
$(OBJS) :  $(CONFIG_HEADER)


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $$< "->" $$@
	$(CC) -c  $$(CFLAGS) $$< -o $$@
endef
$(foreach src, $(SOURCE), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

$(OUTDIR)/$(STARTUP_FILE).o : ./Setup/$(STARTUP_FILE).c $(CONFIG_H_FILES)
	$(CC) -c $(CFLAGS) -O1 ./Setup/$(STARTUP_FILE).c -o $(OUTDIR)/$(STARTUP_FILE).o

clean :
	-cs-rm $(OBJS)
	-cs-rm $(OUTDIR)/$(STARTUP_FILE).o
	-cs-rm $(OUTDIR)/$(PROJECT_NAME).axf
	-cs-rm $(OUTDIR)/$(PROJECT_NAME)_SymbolTable.txt
	-cs-rm $(OUTDIR)/$(PROJECT_NAME)_MemoryListingSummary.txt
	cs-rm $(OUTDIR)/$(PROJECT_NAME)_MemoryListingDetails.txt

log : $(OUTDIR)/$(PROJECT_NAME).axf
	$(NM) -n $(OUTDIR)/$(PROJECT_NAME).axf > $(OUTDIR)/$(PROJECT_NAME)_SymbolTable.txt
	$(OBJDUMP) --format=SysV $(OUTDIR)/$(PROJECT_NAME).axf > $(OUTDIR)/$(PROJECT_NAME)_MemoryListingSummary.txt
	$(OBJDUMP) $(OBJS) > $(OUTDIR)/$(PROJECT_NAME)_MemoryListingDetails.txt

# Eye candy.
begin:
##	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
##	@echo

post:
	arm-none-eabi-size --format=berkeley $(OUTDIR)/$(PROJECT_NAME).axf 

# Display compiler version information.
gccversion :
	@$(CC) --version

$(shell mkdir $(OUTDIR) 2>NUL)