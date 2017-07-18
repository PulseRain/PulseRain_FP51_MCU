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
//      Top module for PulseRain FP51 MCU
//=============================================================================

`include "common.svh"
`include "peripherals.svh"

`default_nettype none
module PulseRain_FP51_MCU 
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
        
        input wire                                  UART_AUX_RXD,
        output wire                                 UART_AUX_TXD,
        
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
    // M23XX1024
    //=======================================================================
        input   wire                                mem_so,
        output  wire                                mem_si,
        output  wire                                mem_hold_n,
        output  wire                                mem_cs_n, 
        output  wire                                mem_sck,
        
    //=======================================================================
    // Si3000
    //=======================================================================
            
        input   wire                                Si3000_SDO,
        output  wire                                Si3000_SDI,
        input   wire                                Si3000_SCLK,
        output  wire                                Si3000_MCLK,
        input   wire                                Si3000_FSYNC_N,  
        output  wire                                Si3000_RESET_N,
        
    //=======================================================================
    // SD Card
    //=======================================================================
        
        output wire                                 sd_cs_n,
        output wire                                 sd_spi_clk,
        input  wire                                 sd_data_out,
        output wire                                 sd_data_in,         
        
    //=======================================================================
    // MAX10 ADC
    //=======================================================================
        input   wire                                adc_pll_clock_clk,                   
        input   wire                                adc_pll_locked_export,
        
    //=======================================================================
    // I2C
    //=======================================================================
        
        input wire                                  sda_in, 
        input wire                                  scl_in,
        
        output wire                                 sda_out,
        output wire                                 scl_out,
        
    //=======================================================================
    // PWM
    //=======================================================================
        output wire unsigned [NUM_OF_PWM - 1 : 0]   pwm_out     
             
            
);

    //=======================================================================
    // signals
    //=======================================================================
        wire                                        interrupt_return;
        wire                                        int_gen;
        wire logic unsigned [DATA_WIDTH - 1 : 0]    int_addr;
        
        wire                                        WB_RD_STB;
        wire  unsigned [DATA_WIDTH - 1 : 0]         WB_RD_ADR;
        wire  unsigned [DATA_WIDTH - 1 : 0]         WB_RD_DAT;
        wire                                        WB_RD_ACK;          
        
        wire                                        WB_WR_CYC;
        wire                                        WB_WR_STB;
        wire                                        WB_WR_WE;
        wire  unsigned [DATA_WIDTH - 1 : 0]         WB_WR_ADR;
        wire  unsigned [DATA_WIDTH - 1 : 0]         WB_WR_DAT;
        wire                                        WB_WR_ACK;
        
        wire                                        timer_event_pulse;
        
    //=======================================================================
    // processor core
    //=======================================================================
                
        generate 
            if (FAST0_SMALL1) begin : processor_core_gen
                
                // TO DO : Put small core here
                
            end else begin : processor_core_gen
                FP51_fast_core #(.FOR_SIM (FOR_SIM)) fast_core (.*,
                        .inst_mem_we (inst_mem_we),
                        .inst_mem_wr_addr (inst_mem_wr_addr),
                        .inst_mem_data_in (inst_mem_data_in),
                        
                        .inst_mem_re (inst_mem_re),
                        .inst_mem_re_addr (inst_mem_re_addr),
                        
                        .inst_mem_re_enable_out (inst_mem_re_enable_out),
                        .inst_mem_data_out (inst_mem_data_out),
                                                                        
                        .interrupt_return (interrupt_return),
                        .int_gen (int_gen),
                        .int_addr (int_addr),
                        
                        .WB_RD_CYC_O (),
                        .WB_RD_STB_O (WB_RD_STB),
                        .WB_RD_ADR_O (WB_RD_ADR),
                        .WB_RD_DAT_I (WB_RD_DAT),
                        .WB_RD_ACK_I (WB_RD_ACK),
                        
                        .WB_WR_CYC_O (),
                        .WB_WR_STB_O (WB_WR_STB),
                        .WB_WR_WE_O  (WB_WR_WE),
                        .WB_WR_ADR_O (WB_WR_ADR),
                        .WB_WR_DAT_O (WB_WR_DAT),
                        .WB_WR_ACK_I (WB_WR_ACK),
                        
                        .P0 (P0),
                        .P1 (P1),
                        .P2 (P2),
                        .P3 (P3),
                        
                        .pause (pause),
                        .break_on (break_on),
                        .break_addr_A (break_addr_A),
                        .break_addr_B (break_addr_B),
                        .run_pulse (run_pulse),
                                                
                        .debug_stall (debug_stall),
                        .debug_PC (debug_PC),
                        
                        .debug_data_read (debug_data_read),
                        .debug_rd_indirect1_direct0 (debug_rd_indirect1_direct0),
                        .debug_data_read_addr (debug_data_read_addr),
                        .debug_data_read_restore (debug_data_read_restore),
                        
                        .debug_data_write (debug_data_write),
                        .debug_data_write_addr (debug_data_write_addr),
                        .debug_wr_indirect1_direct0 (debug_wr_indirect1_direct0),
                        .debug_data_write_data (debug_data_write_data),
                                
                        .debug_read_data_enable_out (debug_read_data_enable_out),
                        .debug_read_data_out (debug_read_data_out)      
                        
                    );
            end : processor_core_gen
        endgenerate 
        
    //=======================================================================
    // peripherals
    //=======================================================================
        
        assign timer_event_pulse = 0; 
        
        peripherals #(.FOR_SIM (FOR_SIM)) peripherals_i (.*,
            .INTx (INTx), 
            
            .interrupt_return (interrupt_return),
            
            .int_addr (int_addr),
            .int_gen (int_gen),
            
            .WB_RD_STB_I (WB_RD_STB),
            .WB_RD_ADR_I (WB_RD_ADR),
            .WB_RD_DAT_O (WB_RD_DAT),
            .WB_RD_ACK_O (WB_RD_ACK),
            
            .WB_WR_STB_I (WB_WR_STB),
            .WB_WR_WE_I (WB_WR_WE),
            .WB_WR_ADR_I (WB_WR_ADR),
            .WB_WR_DAT_I (WB_WR_DAT),
            .WB_WR_ACK_O (WB_WR_ACK),
        
            .timer_event_pulse (timer_event_pulse),
            .timer_pulse_out (timer_pulse_out),
        
            .UART_RXD (UART_RXD),
            .UART_TXD (UART_TXD),
            
            .UART_AUX_RXD (UART_AUX_RXD),
            .UART_AUX_TXD (UART_AUX_TXD),
			 
            .debug_led (debug_led),
            .debug_counter_pulse (debug_counter_pulse),
            
            .mem_so (mem_so),
            .mem_si (mem_si),
            .mem_hold_n (mem_hold_n),
            .mem_cs_n (mem_cs_n),
            .mem_sck (mem_sck),
            
            .Si3000_SDO (Si3000_SDO),
            .Si3000_SDI (Si3000_SDI),
            .Si3000_SCLK (Si3000_SCLK),
            .Si3000_MCLK (Si3000_MCLK),
            .Si3000_FSYNC_N (Si3000_FSYNC_N),
            .Si3000_RESET_N (Si3000_RESET_N),
            
            .sd_cs_n     (sd_cs_n),
            .sd_spi_clk  (sd_spi_clk),
            .sd_data_out (sd_data_out),
            .sd_data_in  (sd_data_in),          
            
            .adc_pll_clock_clk (adc_pll_clock_clk),
            .adc_pll_locked_export (adc_pll_locked_export),
            
            .sda_in (sda_in),
            .scl_in (scl_in),
            
            .sda_out (sda_out),
            .scl_out (scl_out),
            .pwm_out (pwm_out)
            
           );
    
        
endmodule : PulseRain_FP51_MCU

`default_nettype wire
