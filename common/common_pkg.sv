/*
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
*/



//=============================================================================
// Remarks:
//      package for common constant definitions
//=============================================================================


package common_pkg;
    
    parameter unsigned [7 : 0] MCU_REVISION = 8'hA0;
    
    parameter int PC_BITWIDTH = 16;
    
    parameter unsigned [7 : 0] BIT_REG_START_ADDR   = 8'h20;
    parameter unsigned [7 : 0] BIT_REG_END_ADDR     = 8'h2F;
    parameter unsigned [3 : 0] BIT_REG_MSB          = 4'h2;
    parameter int              NUM_OF_BIT_REG       = 16;
    
    parameter int              NUM_OF_GP_RAM_CELLS  = 80;
    parameter int              ON_CHIP_DATA_RAM_SIZE_IN_BYTES = 8 * 1024;
    parameter int              XRAM_ADDR_OFFSET_IN_BYTES = 256;
    parameter int              ON_CHIP_CODE_RAM_SIZE_IN_BYTES = 32 * 1024;
    
    parameter int              DEFAULT_STACK_START = 127;
    
    parameter int              DATA_WIDTH = 8;
    parameter int              BIT_SHIFT_WIDTH = $clog2(DATA_WIDTH);
    parameter int              ADDR_WIDTH = 16;
    parameter int              SP_INC_BITS = 3;
    
    parameter int              NUM_OF_INTx  = 2;
    parameter int              NUM_OF_TIMER = 2;
    parameter int              NUM_OF_UART  = 1;
    parameter int              NUM_OF_ADC   = 1;
    parameter int              NUM_OF_CODEC = 1;
    parameter int              NUM_OF_PWM   = 6;
    parameter int              NUM_OF_INT   = NUM_OF_INTx + NUM_OF_TIMER + NUM_OF_UART + NUM_OF_ADC + NUM_OF_CODEC;
    
    //parameter int              CLASSIC_8051_OSC = 11059200;  // 11.0592 MHz
    //parameter int              ACTUAL_CLK_RATE  = 100454400; // 100.4544 MHz
    
    parameter int              CLASSIC_8051_OSC =  12000000;  // 12.0000 MHz
    parameter int              ACTUAL_CLK_RATE  = 96000000; // 96.0000 MHz
    
    parameter int              TIMER_UNIT_CLASSIC_PULSE_PERIOD =  ACTUAL_CLK_RATE / (CLASSIC_8051_OSC / 12);
    parameter int              TIMER_UNIT_OCD_PERIOD =  ACTUAL_CLK_RATE / (CLASSIC_8051_OSC / 3);
    
    parameter int              MAX_UART_BAUD_RATE = 921600;
    parameter int              MIN_UART_BAUD_RATE = 9600;
    parameter int              MAX_UART_BAID_PERIOD = ACTUAL_CLK_RATE  / MIN_UART_BAUD_RATE;
    parameter int              MIN_UART_BAID_PERIOD = ACTUAL_CLK_RATE / MAX_UART_BAUD_RATE; 
    parameter int              UART_STABLE_COUNT =  ACTUAL_CLK_RATE  / MAX_UART_BAUD_RATE / 2;
endpackage : common_pkg
    