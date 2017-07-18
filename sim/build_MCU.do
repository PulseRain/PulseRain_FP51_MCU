###############################################################################
# Copyright (c) 2017, PulseRain Technology LLC 
#
# This program is distributed under a dual license: an open source license, 
# and a commercial license. 
# 
# The open source license under which this program is distributed is the 
# GNU Public License version 3 (GPLv3).
#
# And for those who want to use this program in ways that are incompatible
# with the GPLv3, PulseRain Technology LLC offers commercial license instead.
# Please contact PulseRain Technology LLC (www.pulserain.com) for more detail.
#
###############################################################################

file delete -force work
vlib work
vmap work work

set common "../common"
set CRC "../submodules/CRC/source"


vlog -work work -sv $common/common_pkg.sv




vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/ADC ../submodules/PulseRain_rtl_lib/ADC/wb_MAX10_ADC.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/chip_ID ../submodules/PulseRain_rtl_lib/chip_ID/wb_MAX10_chip_ID.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/codec/Si3000 ../submodules/PulseRain_rtl_lib/codec/Si3000/Si3000.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/codec/Si3000 ../submodules/PulseRain_rtl_lib/codec/Si3000/wb_Si3000.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/debug_counter_led ../submodules/PulseRain_rtl_lib/debug_counter_led/debug_counter_led.sv


vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/FASM_register ../submodules/PulseRain_rtl_lib/FASM_register/FASM_register.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/FASM_register ../submodules/PulseRain_rtl_lib/FASM_register/wb_register.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/block_memory +incdir+../submodules/PulseRain_rtl_lib/flash_loader ../submodules/PulseRain_rtl_lib/flash_loader/wb_flash_loader.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/interrupt ../submodules/PulseRain_rtl_lib/interrupt/interrupt.sv


vlog -work work -sv +incdir+$common +incdir+$CRC +incdir+../submodules/PulseRain_rtl_lib/SD ../submodules/PulseRain_rtl_lib/SD/SD_SPI.sv
vlog -work work -sv +incdir+$common +incdir+$CRC +incdir+../submodules/PulseRain_rtl_lib/SD ../submodules/PulseRain_rtl_lib/SD/wb_SD.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/SRAM/M23XX1024 ../submodules/PulseRain_rtl_lib/SRAM/M23XX1024/M23XX1024.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/SRAM/M23XX1024 ../submodules/PulseRain_rtl_lib/SRAM/M23XX1024/wb_M23XX1024.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/timer/FP51 ../submodules/PulseRain_rtl_lib/timer/FP51/timer_unit_pulse.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/timer/FP51 ../submodules/PulseRain_rtl_lib/timer/FP51/timer.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/timer/FP51 ../submodules/PulseRain_rtl_lib/timer/FP51/wb_timer_8051.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/UART/FP51 ../submodules/PulseRain_rtl_lib/UART/FP51/UART_RX_FIFO.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/UART/FP51 ../submodules/PulseRain_rtl_lib/UART/FP51/Serial_8051.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/UART/FP51 ../submodules/PulseRain_rtl_lib/UART/FP51/wb_Serial_8051.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/I2C ../submodules/PulseRain_rtl_lib/I2C/I2C_Master.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/I2C ../submodules/PulseRain_rtl_lib/I2C/I2C_Slave.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/I2C ../submodules/PulseRain_rtl_lib/I2C/wb_I2C.sv

vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/PWM ../submodules/PulseRain_rtl_lib/PWM/PWM_core.sv
vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/PWM ../submodules/PulseRain_rtl_lib/PWM/wb_PWM.sv


vlog -work work -sv +incdir+$common +incdir+../submodules/PulseRain_rtl_lib/ADC \
     +incdir+../submodules/PulseRain_rtl_lib/chip_ID \
     +incdir+../submodules/PulseRain_rtl_lib/codec/Si3000 \
     +incdir+../submodules/PulseRain_rtl_lib/debug_counter_led \
     +incdir+../submodules/PulseRain_rtl_lib/FASM_register \
     +incdir+../submodules/PulseRain_rtl_lib/block_memory +incdir+../submodules/PulseRain_rtl_lib/flash_loader \
     +incdir+../submodules/PulseRain_rtl_lib/interrupt \
     +incdir+../submodules/PulseRain_rtl_lib/SD \
     +incdir+../submodules/PulseRain_rtl_lib/SRAM/M23XX1024 \
     +incdir+../submodules/PulseRain_rtl_lib/timer/FP51 \
     +incdir+../submodules/PulseRain_rtl_lib/UART/FP51 \
     +incdir+../submodules/PulseRain_rtl_lib/I2C \
     +incdir+../submodules/PulseRain_rtl_lib/PWM \
        +incdir+../peripherals ../peripherals/peripherals.sv
        

vlog -work work -sv +incdir+$common +incdir+../peripherals \
     +incdir+../submodules/PulseRain_rtl_lib/ADC \
     +incdir+../submodules/PulseRain_rtl_lib/chip_ID \
     +incdir+../submodules/PulseRain_rtl_lib/codec/Si3000 \
     +incdir+../submodules/PulseRain_rtl_lib/debug_counter_led \
     +incdir+../submodules/PulseRain_rtl_lib/FASM_register \
     +incdir+../submodules/PulseRain_rtl_lib/block_memory +incdir+../submodules/PulseRain_rtl_lib/flash_loader \
     +incdir+../submodules/PulseRain_rtl_lib/interrupt \
     +incdir+../submodules/PulseRain_rtl_lib/SD \
     +incdir+../submodules/PulseRain_rtl_lib/SRAM/M23XX1024 \
     +incdir+../submodules/PulseRain_rtl_lib/timer/FP51 \
     +incdir+../submodules/PulseRain_rtl_lib/UART/FP51 \
     +incdir+../submodules/PulseRain_rtl_lib/I2C \
     +incdir+../submodules/PulseRain_rtl_lib/PWM \
        ../source/PulseRain_FP51_MCU.sv

