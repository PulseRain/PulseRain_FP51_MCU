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
//      head file for common constant definitions and module prototypes
//=============================================================================


`ifndef COMMON_SVH
`define COMMON_SVH

    import common_pkg::*;
    

extern module FP51_fast_core #(parameter FOR_SIM = 0) (
        
    //=======================================================================
    // clock / reset
    //=======================================================================
    
        input wire                                  clk,                             // clock input
        input wire                                  reset_n,                         // reset, active low
        
    //=======================================================================
    // instruction memory external r/w 
    //=======================================================================
        
        input wire                                  inst_mem_we,
        input wire unsigned [PC_BITWIDTH - 3 : 0]   inst_mem_wr_addr,
        input wire unsigned [31 : 0]                inst_mem_data_in,
        
        input wire                                  inst_mem_re,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   inst_mem_re_addr,
        
        output wire                                 inst_mem_re_enable_out,
        output logic unsigned [31 : 0]              inst_mem_data_out,
        
                    
    //=======================================================================
    // interrupt 
    //=======================================================================
        
        input wire                                  int_gen,
        input wire unsigned [7 : 0]                 int_addr,
        output wire                                 interrupt_return,

    //=======================================================================
    // Wishbone Host Interface 
    //=======================================================================
        output wire                                 WB_RD_CYC_O,
        output wire                                 WB_RD_STB_O,
        output wire  unsigned [DATA_WIDTH - 1 : 0]  WB_RD_ADR_O,
        input  wire  unsigned [DATA_WIDTH - 1 : 0]  WB_RD_DAT_I,
        input  wire                                 WB_RD_ACK_I,
        
        output wire                                 WB_WR_CYC_O,
        output wire                                 WB_WR_STB_O,
        output wire                                 WB_WR_WE_O,
        output wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_ADR_O,
        output wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_DAT_O,
        input  wire                                 WB_WR_ACK_I,
        
    //=======================================================================
    // ports 
    //=======================================================================
            
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P0,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P1,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P2,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P3,
        
    //=======================================================================
    // debug 
    //=======================================================================
        input wire                                  pause,
        input wire                                  break_on,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_A,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_B,
        input wire                                  run_pulse,
        
        output logic                                debug_stall,
        output logic unsigned [PC_BITWIDTH - 1 : 0] debug_PC,
                
        input wire                                  debug_data_read,
        input wire                                  debug_rd_indirect1_direct0,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   debug_data_read_addr,
        input wire                                  debug_data_read_restore,
        
        input wire                                  debug_data_write,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   debug_data_write_addr,
        input wire unsigned                         debug_wr_indirect1_direct0,
        input wire unsigned [DATA_WIDTH - 1 : 0]    debug_data_write_data,
        
        
        output logic                                debug_read_data_enable_out,
        output logic unsigned [DATA_WIDTH - 1 : 0]  debug_read_data_out     
        
);

extern module PulseRain_FP51_MCU 
        #(parameter FOR_SIM = 0, FAST0_SMALL1 = 0) (
    
    //=======================================================================
    // clock / reset
    //=======================================================================
        
        input wire                                  clk,                             // clock input
        input wire                                  reset_n,                         // reset, active low
    
    //=======================================================================
    // Instruction Memory r/w
    //=======================================================================
        
        input wire                                  inst_mem_we,
        input wire unsigned [PC_BITWIDTH - 3 : 0]   inst_mem_wr_addr,
        input wire unsigned [31 : 0]                inst_mem_data_in,
        
        input wire                                  inst_mem_re,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   inst_mem_re_addr,
        
        output wire                                 inst_mem_re_enable_out,
        output wire unsigned [31 : 0]               inst_mem_data_out,
        
    //=======================================================================
    // External Interrupt
    //=======================================================================
                
        input wire  unsigned [NUM_OF_INTx - 1 : 0]  INTx,
        
    //=======================================================================
    // UART
    //=======================================================================
        input wire                                  UART_RXD,
        output wire                                 UART_TXD,
        
        
    //=======================================================================
    // Ports
    //=======================================================================
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P0,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P1,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P2,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P3,

    //=======================================================================
    // Debug
    //=======================================================================
        
        input wire                                  pause,
        input wire                                  break_on,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_A,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_B,
        
        input wire                                  run_pulse,      
                                
        output logic                                debug_stall,
        output wire unsigned [PC_BITWIDTH - 1 : 0]  debug_PC,
                
        input wire                                  debug_data_read,
        input wire                                  debug_rd_indirect1_direct0,
        input wire unsigned [PC_BITWIDTH - 1 : 0]                debug_data_read_addr,
        input wire                                  debug_data_read_restore,
        
        input wire                                              debug_data_write,
        input wire unsigned [PC_BITWIDTH - 1 : 0]               debug_data_write_addr,
        input wire unsigned                                     debug_wr_indirect1_direct0,
        input wire unsigned [DATA_WIDTH - 1 : 0]                debug_data_write_data,
        
        output wire                                 debug_read_data_enable_out,
        output wire unsigned [DATA_WIDTH - 1 : 0]   debug_read_data_out,
        output wire                                 timer_pulse_out,
        
        output wire                                 debug_led,
        output wire                                 debug_counter_pulse,
        
        
    //=======================================================================
    // I2C
    //=======================================================================
        
        input wire                                  sda_in, 
        input wire                                  scl_in,
        
        output wire                                 sda_out,
        output wire                                 scl_out,
        
    //=======================================================================
    // Rotary Encoder
    //=======================================================================
     	input   wire                                encoder_clk,
        input   wire                                encoder_dt,
        input   wire                                encoder_sw,   

    //=======================================================================
    // PS2
    //=======================================================================
        input   wire                                ps2_clk,
        input   wire                                ps2_dat,
        
    //=======================================================================
    // PWM
    //=======================================================================
        output wire unsigned [NUM_OF_PWM - 1 : 0]   pwm_out     
            
);

    
    
    
parameter int SIM_BIST              = 2;
parameter int SIM_PRELOAD_CODE      = 1;
parameter unsigned [7:0]  SYS_VER   = 8'h01;
    
`endif