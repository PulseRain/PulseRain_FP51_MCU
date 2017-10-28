`ifndef PERIPHERALS_SVH
`define PERIPHERALS_SVH

`include "common.svh"
`include "chip_ID.svh"
`include "debug_counter_led.svh"
`include "FASM_register.svh"
`include "interrupt.svh"
`include "timer.svh"
`include "Serial_8051.svh"
`include "flash_loader.svh"

parameter int UART_FIFO_SIZE = 1024;    


extern module peripherals #(parameter FOR_SIM = 0) (
    //=======================================================================
    // clock / reset
    //=======================================================================
        
        input wire                  clk,                             // clock input
        input wire                  reset_n,                         // reset, active low
        
    //=======================================================================
    // Interrupt
    //=======================================================================
                
        input wire  unsigned [NUM_OF_INTx - 1 : 0]  INTx, // external interrupt 
        
        input wire                                  interrupt_return,
        output  logic unsigned [DATA_WIDTH - 1 : 0] int_addr,
        output  logic                               int_gen,
                
    //=======================================================================
    // Wishbone Interface (FASM synchronous RAM dual port model)
    //=======================================================================
        input  wire                                 WB_RD_STB_I,
        input  wire  unsigned [DATA_WIDTH - 1 : 0]  WB_RD_ADR_I,
        output logic unsigned [DATA_WIDTH - 1 : 0]  WB_RD_DAT_O,
        output wire                                 WB_RD_ACK_O,
                
        input  wire                                 WB_WR_STB_I,
        input  wire                                 WB_WR_WE_I,
        input  wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_ADR_I,
        input  wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_DAT_I,
        output wire                                 WB_WR_ACK_O,
        
        
    //=======================================================================
    // Timer 
    //=======================================================================
        input  wire                                 timer_event_pulse,
        output logic                                timer_pulse_out,
    //=======================================================================
    // UART
    //=======================================================================
        input wire                                  UART_RXD,
        output wire                                 UART_TXD,
        
    //=======================================================================
    // Debug LED
    //=======================================================================
        output wire                                 debug_led,
        output wire                                 debug_counter_pulse,
        
    //=======================================================================
    // Flash Loader
    //=======================================================================
        input wire                                      flash_buffer_write_enable,
        input wire [DATA_WIDTH * 4 - 1 : 0]             flash_buffer_data_in,
        input wire [FLASH_LOADER_BUFFER_BITS - 1 : 0]   flash_buffer_write_address,
        
        output  wire                                    flash_loader_active_flag,
        output  wire                                    flash_loader_done_flag,
        
        output  wire                                    flash_loader_ping_busy,
        output  wire                                    flash_loader_pong_busy,
        
        output  wire unsigned [1 : 0]                   flash_buffer_ping_state_out,
        output  wire unsigned [1 : 0]                   flash_buffer_pong_state_out
);

`endif