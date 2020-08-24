##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.10.0-B14] date: [Mon Aug 24 19:55:58 EDT 2020] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = stepper


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/freertos.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Src/stm32f4xx_hal_timebase_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Src/system_stm32f4xx.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c

# /* USER CODE BEGIN */

C_SOURCES += Ourwares/SerialTaskSend.c 
#C_SOURCES += Ourwares/cdc_txbuff.c
#C_SOURCES += Ourwares/cdc_rxbuff.c
#C_SOURCES += Ourwares/cdc_rxbuffTaskCAN.c
C_SOURCES += Ourwares/DTW_counter.c
C_SOURCES += Ourwares/CanTask.c
C_SOURCES += Ourwares/can_iface.c
C_SOURCES += Ourwares/canfilter_setup.c
C_SOURCES += Ourwares/getserialbuf.c
C_SOURCES += Ourwares/yprintf.c
C_SOURCES += Ourwares/USB_PC_gateway.c
C_SOURCES += Ourwares/PC_gateway_comm.c
C_SOURCES += Ourwares/gateway_comm.c
C_SOURCES += Ourwares/gateway_CANtoPC.c
C_SOURCES += Ourwares/SerialTaskReceive.c
C_SOURCES += Ourwares/yscanf.c
C_SOURCES += Ourwares/gateway_PCtoCAN.c
C_SOURCES += Ourwares/morse.c
C_SOURCES += Ourwares/payload_extract.c
C_SOURCES += Ourwares/MailboxTask.c
C_SOURCES += Ourwares/GatewayTask.c
C_SOURCES += Ourwares/adctask.c
C_SOURCES += Ourwares/ADCTask.c

C_SOURCES += Ourtasks/stackwatermark.c
C_SOURCES += Ourtasks/DMOCchecksum.c
C_SOURCES += Ourtasks/adcfastsum16.c
C_SOURCES += Ourtasks/adcextendsum.c
C_SOURCES += Ourtasks/adcparams.c
C_SOURCES += Ourtasks/adcparamsinit.c
C_SOURCES += Ourtasks/adc_idx_v_struct.c
C_SOURCES += Ourtasks/iir_f1.c
C_SOURCES += Ourtasks/iir_f2.c
C_SOURCES += Ourtasks/BeepTask.c
C_SOURCES += Ourtasks/lcdprintf.c
C_SOURCES += Ourtasks/GevcuTask.c
C_SOURCES += Ourtasks/GevcuStates.c
C_SOURCES += Ourtasks/GevcuEvents.c
C_SOURCES += Ourtasks/GevcuUpdates.c
C_SOURCES += Ourtasks/gevcu_idx_v_struct.c
C_SOURCES += Ourtasks/gevcu_func_init.c
C_SOURCES += Ourtasks/iir_filter_lx.c
C_SOURCES += Ourtasks/spiserialparallelSW.c
C_SOURCES += Ourtasks/SpiOutTask.c
C_SOURCES += Ourtasks/calib_control_lever.c
C_SOURCES += Ourtasks/4x20lcd.c
C_SOURCES += Ourtasks/contactor_control.c
C_SOURCES += Ourtasks/dmoc_control.c
C_SOURCES += Ourtasks/paycnvt.c
C_SOURCES += Ourtasks/LEDTask.c
C_SOURCES += Ourtasks/led_chasing.c
C_SOURCES += Ourtasks/contactor_control_msg.c
C_SOURCES += Ourtasks/lcdmsg.c
C_SOURCES += Ourtasks/control_law_v1.c
C_SOURCES += Ourtasks/lcd_hd44780_i2c.c
C_SOURCES += Ourtasks/LcdTask.c
#C_SOURCES += Ourtasks/LcdmsgsTask.c
C_SOURCES += Ourtasks/LcdmsgsetTask.c
C_SOURCES += Ourtasks/stepper_items.c


# /* USER CODE END */ 


# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# AS includes
AS_INCLUDES =  \
-I/Inc

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

# /* USER CODE BEGIN */

C_INCLUDES += -IOurwares 
C_INCLUDES += -IOurtasks

# /* USER CODE END */ 



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
