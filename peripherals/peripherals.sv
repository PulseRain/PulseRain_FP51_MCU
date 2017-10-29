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
//      peripherals for FP51
//=============================================================================


`include "common.svh"
`include "SFR.svh"
`include "peripherals.svh"

`default_nettype none

module peripherals #(parameter FOR_SIM = 0) (
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
        
        input wire                                  UART_AUX_RXD,
        output wire                                 UART_AUX_TXD,
        
    //=======================================================================
    // Debug LED
    //=======================================================================
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
        
        input wire                                   sda_in, 
        input wire                                   scl_in,
        
        output wire                                  sda_out,
        output wire                                  scl_out,
        
    //=======================================================================
    // PWM
    //=======================================================================
        output wire unsigned [NUM_OF_PWM - 1 : 0]    pwm_out     
        
);
    
    //=======================================================================
    // signal 
    //=======================================================================
        wire unsigned [DATA_WIDTH - 1 : 0]          IE_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          IE;
        wire unsigned [DATA_WIDTH - 1 : 0]          IP_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          TMOD_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          TH0_TL0_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          TH1_TL1_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          uart_reg_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          uart_aux_reg_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          debug_counter_led_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          M23XX1024_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          Si3000_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          ADC_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          sd_data; 
        wire unsigned [DATA_WIDTH - 1 : 0]          chip_ID_data_out;
        wire unsigned [DATA_WIDTH - 1 : 0]          I2C_data_out;
        
        
        wire                                        TMOD_GATE1, TMOD_C_T1, TMOD_T1M1;
        wire                                        TMOD_T1M0, TMOD_GATE0, TMOD_C_T0;
        wire                                        TMOD_T0M1, TMOD_T0M0;
        
        wire                                        class_8051_unit_pulse;
        logic unsigned [DATA_WIDTH - 1 : 0]         TCON_reg;
        
        wire                                        TCON_TR0, TCON_TR1;
        
        wire                                        timer0_trigger, timer1_trigger;
        
        logic  unsigned [NUM_OF_INTx - 1 : 0]       INTx_meta, INTx_sync;
        
        wire                                        SCON_TI, SCON_RI;       
        wire                                        SCON_AUX_TI, SCON_AUX_RI;       
        
        wire                                        jtag_uart_read_n;
        wire                                        jtag_uart_write_n;
        wire  unsigned [31 : 0]                     jtag_uart_read_data;
        
        logic                                       jtag_uart_write_n_d1 = 1'b1;
        logic                                       jtag_uart_write_n_d2 = 1'b1;
          
        logic unsigned [DATA_WIDTH - 1 : 0]         jtag_data_write_reg, WB_WR_DAT_I_d1;
        
        wire                                        dog_bite;
        wire                                        adc_data_ready;
        
        wire                                        codec_fsync_pulse;
        
        wire                                        i2c_irq;
        
    //=======================================================================
    // Output Mux
    //=======================================================================
        assign WB_RD_ACK_O = WB_RD_STB_I;
        assign WB_WR_ACK_O = WB_WR_STB_I;
        
        always_comb begin : output_data_proc
            case (WB_RD_ADR_I) 
                IE_ADDR : begin
                    WB_RD_DAT_O = IE_data_out;
                end
                
                IP_ADDR : begin
                    WB_RD_DAT_O = IP_data_out;
                end
                
                TMOD_ADDR : begin
                    WB_RD_DAT_O = TMOD_data_out;
                end
                
                TCON_ADDR : begin
                    WB_RD_DAT_O = TCON_reg;
                end
                
                TH0_ADDR : begin
                    WB_RD_DAT_O = TH0_TL0_data_out;
                end
                
                TL0_ADDR : begin
                    WB_RD_DAT_O = TH0_TL0_data_out;
                end
                
                TH1_ADDR : begin
                    WB_RD_DAT_O = TH1_TL1_data_out;
                end
                
                TL1_ADDR : begin
                    WB_RD_DAT_O = TH1_TL1_data_out;
                end
                
                SCON_ADDR : begin
                    WB_RD_DAT_O = uart_reg_data_out;
                end
                
                SBUF_ADDR : begin
                    WB_RD_DAT_O = uart_reg_data_out;
                end
                
                SCON_AUX_ADDR : begin
                    WB_RD_DAT_O = uart_aux_reg_data_out;
                end
                
                SBUF_AUX_ADDR : begin
                    WB_RD_DAT_O = uart_aux_reg_data_out;
                end
                
                DEBUG_LED_ADDR : begin
                    WB_RD_DAT_O = debug_counter_led_out;
                end
                
                JTAG_UART_ADDR : begin
                    WB_RD_DAT_O = jtag_uart_read_data [7 : 0];
                end
                
                SRAM_DATA_ADDR : begin
                    WB_RD_DAT_O = M23XX1024_data_out;
                end
                
                SRAM_CSR_ADDR  : begin
                    WB_RD_DAT_O = M23XX1024_data_out;
                end
                
                ADC_DATA_HIGH_ADDR : begin
                    WB_RD_DAT_O = ADC_data_out;
                end
                
                ADC_DATA_LOW_ADDR : begin
                    WB_RD_DAT_O = ADC_data_out;
                end
                
                ADC_CSR_ADDR      : begin
                    WB_RD_DAT_O = ADC_data_out;
                end
                
                CODEC_READ_DATA_LOW_ADDR : begin
                    WB_RD_DAT_O = Si3000_data_out;
                end
                
                CODEC_READ_DATA_HIGH_ADDR : begin
                    WB_RD_DAT_O = Si3000_data_out;
                end
                
                CODEC_CSR_ADDR : begin
                    WB_RD_DAT_O = Si3000_data_out;
                end
                
                CHIP_ID_DATA_CSR_ADDR : begin
                    WB_RD_DAT_O = chip_ID_data_out;
                end
                
                SD_CMD_ADDR : begin
                    WB_RD_DAT_O = sd_data;
                end
                
                SD_DATA_IN_ADDR : begin
                    WB_RD_DAT_O = sd_data;
                end
                
                MCU_REVISION_ADDR : begin
                    WB_RD_DAT_O = MCU_REVISION;
                end
                
                I2C_CSR_ADDR : begin
                    WB_RD_DAT_O = I2C_data_out;
                end
                
                I2C_ADDR_DATA_ADDR : begin
                    WB_RD_DAT_O = I2C_data_out;
                end
                
                
                default : begin
                    WB_RD_DAT_O = 0;
                end
                
            endcase
            
        end : output_data_proc
    
    //=======================================================================
    // IE (Interrupt Enable Register)
    //=======================================================================
        
        wb_register #(.REG_ADDR (IE_ADDR)) reg_IE (.*,
                .stb_i (WB_WR_STB_I),
                .we_i  (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (IE_data_out),
                .ack_o ());
        
    //=======================================================================
    // IP (Interrupt Priorities Register)
    //=======================================================================
        
        wb_register #(.REG_ADDR (IP_ADDR)) reg_IP (.*,
                .stb_i (WB_WR_STB_I),
                .we_i  (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (IP_data_out),
                .ack_o ());
        
        
    //=======================================================================
    // TMOD Register
    //=======================================================================
        
        wb_register #(.REG_ADDR (TMOD_ADDR)) reg_TMOD (.*,
                .stb_i (WB_WR_STB_I),
                .we_i  (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (TMOD_data_out),
                .ack_o ());
        
        assign {TMOD_GATE1, TMOD_C_T1, TMOD_T1M1, TMOD_T1M0, TMOD_GATE0, TMOD_C_T0, TMOD_T0M1, TMOD_T0M0} = TMOD_data_out;
        
    //=======================================================================
    // TCON Register
    //=======================================================================
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                TCON_reg <= 0;
            end else if (WB_WR_STB_I && WB_WR_WE_I && (WB_WR_ADR_I == TCON_ADDR)) begin
                TCON_reg <= WB_WR_DAT_I;
            end else if (timer0_trigger) begin
                TCON_reg <= TCON_reg | ($size(TCON_reg))'(1 << TCON_TF0_INDEX);
            end else if (timer1_trigger) begin
                TCON_reg <= TCON_reg | ($size(TCON_reg))'(1 << TCON_TF1_INDEX);
            end
        end
                  
    //=======================================================================
    // Timer Unit Pulse
    //=======================================================================
       /* timer_unit_pulse #(.TIMER_UNIT_PULSE_PERIOD (TIMER_UNIT_CLASSIC_PULSE_PERIOD))
            timer_unit_pulse_i (.*,
            .unit_period_update (1'b0),
            .unit_period (8'd0),
            .enable (1'b1),
            .unit_pulse (class_8051_unit_pulse));
        */
    //=======================================================================
    // Timer 0
    //=======================================================================
          assign class_8051_unit_pulse = 0;
		  
		  wb_timer_8051 
          #(.REG_ADDR_TH (TH0_ADDR), .REG_ADDR_TL (TL0_ADDR)) timer0 (.*,
            .stb_i (WB_WR_STB_I),
            .we_i (WB_WR_WE_I),
            .adr_wr_i (WB_WR_ADR_I),
            .adr_rd_i (WB_RD_ADR_I),
            .dat_i (WB_WR_DAT_I),
            
            .dat_o (TH0_TL0_data_out),
            .ack_o (),
    
            .class_8051_unit_pulse (class_8051_unit_pulse),
            
            .TMOD_C_T (TMOD_C_T0),
            .TMOD_M1 (TMOD_T0M1),
            .TMOD_M0 (TMOD_T0M0),
            
            .TCON_TR (TCON_reg [TCON_TR0_INDEX]),
            
            .INTx (INTx),
            .TMOD_GATE ({TMOD_GATE1, TMOD_GATE0}),
            
            .event_pulse (timer_event_pulse),
            .timer_trigger (timer0_trigger));
        
          always_ff @(posedge clk, negedge reset_n) begin
                if (!reset_n) begin
                    timer_pulse_out <= 0;
                end else if (timer0_trigger) begin
                    timer_pulse_out <= ~timer_pulse_out;
                end
          end

   
    //=======================================================================
    // Timer 1
    //=======================================================================
        wb_timer_8051 
          #(.REG_ADDR_TH (TH1_ADDR), .REG_ADDR_TL (TL1_ADDR)) timer1 (.*,
            .stb_i (WB_WR_STB_I),
            .we_i (WB_WR_WE_I),
            .adr_wr_i (WB_WR_ADR_I),
            .adr_rd_i (WB_RD_ADR_I),
            .dat_i (WB_WR_DAT_I),
            
            .dat_o (TH1_TL1_data_out),
            .ack_o (),
    
            .class_8051_unit_pulse (class_8051_unit_pulse),
            
            .TMOD_C_T (TMOD_C_T1),
            .TMOD_M1 (TMOD_T1M1),
            .TMOD_M0 (TMOD_T1M0),
            
            .TCON_TR (TCON_reg [TCON_TR1_INDEX]),
            
            .INTx (INTx),
            .TMOD_GATE ({TMOD_GATE1, TMOD_GATE0}),
            
            .event_pulse (timer_event_pulse),
            .timer_trigger (timer1_trigger));
        
          
    //=======================================================================
    // AUX Serial Port
    //=======================================================================
        wb_Serial_8051
            #(.STABLE_TIME (UART_STABLE_COUNT), 
              .MAX_BAUD_PERIOD (MAX_UART_BAUD_RATE), 
              .REG_ADDR_SCON (SCON_AUX_ADDR), 
              .REG_ADDR_SBUF (SBUF_AUX_ADDR),
              .FIFO_SIZE (8)) UART_AUX (.*,
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (uart_aux_reg_data_out),
                .ack_o (),
                
                .class_8051_unit_pulse (class_8051_unit_pulse),
                .timer_trigger (timer0_trigger),
                .UART_RXD (UART_AUX_RXD),
                .UART_TXD (UART_AUX_TXD),
                .SCON_TI (SCON_AUX_TI),
                .SCON_RI (SCON_AUX_RI));
                  
    //=======================================================================
    // Serial Port
    //=======================================================================
        wb_Serial_8051
            #(.STABLE_TIME (UART_STABLE_COUNT), 
              .MAX_BAUD_PERIOD (MAX_UART_BAUD_RATE), 
              .REG_ADDR_SCON (SCON_ADDR), 
              .REG_ADDR_SBUF (SBUF_ADDR),
              .FIFO_SIZE (4)) UART (.*,
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (uart_reg_data_out),
                .ack_o (),
                
                .class_8051_unit_pulse (class_8051_unit_pulse),
                .timer_trigger (timer1_trigger),
                .UART_RXD (UART_RXD),
                .UART_TXD (UART_TXD),
                .SCON_TI (SCON_TI),
                .SCON_RI (SCON_RI));
    
    
    //=======================================================================
    // M23XX1024
    //=======================================================================
         wb_M23XX1024 #(.CLK_SCK_RATIO        (SRAM_CLK_SCK_RATIO),
                        .REG_ADDR_INSTRUCTION (SRAM_INSTRUCTION_ADDR),
                        .REG_ADDR_DATA        (SRAM_DATA_ADDR),
                        .REG_ADDR_ADDRESS2    (SRAM_ADDRESS2_ADDR),
                        .REG_ADDR_ADDRESS1    (SRAM_ADDRESS1_ADDR),
                        .REG_ADDR_ADDRESS0    (SRAM_ADDRESS0_ADDR),
                        .REG_ADDR_CSR         (SRAM_CSR_ADDR))  SRAM_i (.*,
                
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (M23XX1024_data_out),
                .ack_o (),
                    
                .mem_so     (mem_so),
                .mem_si     (mem_si),
                .mem_hold_n (mem_hold_n),
                .mem_cs_n   (mem_cs_n),
                .mem_sck    (mem_sck)
            );
            
    //=======================================================================
    // Si3000
    //=======================================================================
        wb_Si3000 #(.MCLK_DENOM(1), .MCLK_NUMER(24), .WORD_SIZE (16),
                .REG_ADDR_WRITE_DATA_LOW  (CODEC_WRITE_DATA_LOW_ADDR),
                .REG_ADDR_WRITE_DATA_HIGH (CODEC_WRITE_DATA_HIGH_ADDR),
                .REG_ADDR_READ_DATA_LOW  (CODEC_READ_DATA_LOW_ADDR),
                .REG_ADDR_READ_DATA_HIGH (CODEC_READ_DATA_HIGH_ADDR),
                .REG_ADDR_CSR (CODEC_CSR_ADDR))  wb_Si3000_i (.*,
                
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (Si3000_data_out),
                .ack_o (),
                
                .fsync_pulse (codec_fsync_pulse),
                
                .Si3000_SDO     (Si3000_SDO),
                .Si3000_SDI     (Si3000_SDI),
                .Si3000_SCLK    (Si3000_SCLK),
                .Si3000_MCLK    (Si3000_MCLK),
                .Si3000_FSYNC_N (Si3000_FSYNC_N),
                .Si3000_RESET_N (Si3000_RESET_N)
        );
        
         
    //=======================================================================
    // SD Card
    //=======================================================================
        
        wb_SD #(.SD_CLK_SLOW_SCK_RATIO (SD_CLK_SLOW_SCK_RATIO), 
                .SD_CLK_FAST_SCK_RATIO (SD_CLK_FAST_SCK_RATIO), 
                .BUFFER_SIZE_IN_BYTES (SD_BUFFER_SIZE_IN_BYTES),
                .REG_ADDR_CSR      (SD_CSR_ADDR),
                .REG_ADDR_CMD      (SD_CMD_ADDR),
                .REG_ADDR_ARG0     (SD_ARG0_ADDR),
                .REG_ADDR_ARG1     (SD_ARG1_ADDR),
                .REG_ADDR_ARG2     (SD_ARG2_ADDR),
                .REG_ADDR_ARG3     (SD_ARG3_ADDR),
                .REG_ADDR_BUF_ADDR (SD_BUF_ADDR),
                .REG_ADDR_DATA_IN  (SD_DATA_IN_ADDR),
                .REG_ADDR_DATA_OUT (SD_DATA_OUT_ADDR)) wb_SD_i (.*,
                
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (sd_data),
                .ack_o (),
                
                .cs_n (sd_cs_n),
                .spi_clk (sd_spi_clk),
                .sd_data_out (sd_data_out),
                .sd_data_in (sd_data_in)    
        );
            
    //=======================================================================
    // MAX 10 ADC
    //=======================================================================
             
         wb_MAX10_ADC #(.REG_ADDR_DATA_HIGH (ADC_DATA_HIGH_ADDR),
                        .REG_ADDR_DATA_LOW  (ADC_DATA_LOW_ADDR),
                        .REG_ADDR_CSR (ADC_CSR_ADDR) ) ADC_i (.*,
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (ADC_data_out),
                .ack_o (),
                
                .adc_pll_clock_clk (adc_pll_clock_clk),
                .adc_pll_locked_export (adc_pll_locked_export),               
                .adc_data_ready (adc_data_ready)
        );
         
         
    //=======================================================================
    // unique chip id
    //=======================================================================
        wb_Altera_chip_ID #(.REG_ADDR_DATA_CSR(CHIP_ID_DATA_CSR_ADDR)) chip_ID_i (.*,
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (chip_ID_data_out),
                .ack_o ()
        );     
         
        
    //=======================================================================
    // I2C
    //=======================================================================
        wb_I2C #(.REG_ADDR_CSR (I2C_CSR_ADDR), 
                 .REG_ADDR_DATA (I2C_ADDR_DATA_ADDR)) wb_I2C_i (.*,
                
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (I2C_data_out),
                .ack_o (),
                
                .sda_in (sda_in),
                .scl_in (scl_in),
                
                .sda_out (sda_out),
                .scl_out (scl_out),
                
                .irq (i2c_irq)
        );
    
        
    //=======================================================================
    // PWM
    //=======================================================================
        wb_PWM #(.NUM_OF_PWM (NUM_OF_PWM), 
                 .REG_ADDR_CSR (PWM_CSR_ADDR),
                 .REG_ADDR_DATA (PWM_DATA_ADDR)) wb_PWM_i (.*,
                
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (),
                .ack_o (),
                
                .pwm_out (pwm_out) 
        
        );
        
        
    //=======================================================================
    // Interrupt Controller
    //=======================================================================
        always_ff @(posedge clk, negedge reset_n) begin : INTx_proc
            if (!reset_n) begin
                INTx_meta <= 0;
                INTx_sync <= 0;
            end else begin
                INTx_meta <= INTx;
                INTx_sync <= INTx_meta;
            end
        end : INTx_proc
        
        assign IE = IE_data_out;
        
        interrupt #(.NUM_OF_INT (NUM_OF_INT)) interrupt_i (.*,
            .ret_int (interrupt_return),
            .global_int_enable (IE [IE_GLOBAL_INT_ENABLE_INDEX]),
            
            .int_enable_mask ({IE [IE_ENABLE_CODEC_INDEX],
                               IE [IE_ENABLE_ADC_INDEX], 
                               IE [IE_ENABLE_SERIAL_INT_INDEX],
                               IE [IE_ENABLE_TIMER1_INT_INDEX],
                               IE [IE_ENABLE_EXT_INT1_INDEX],
                               IE [IE_ENABLE_TIMER0_INT_INDEX],
                               IE [IE_ENABLE_EXT_INT0_INDEX]}),
            
            .int_priority_mask (IP_data_out [NUM_OF_INT - 1 : 0]),
            .int_level1_pulse0 (7'b011_0101),
            
            .int_pins ({codec_fsync_pulse,
                        adc_data_ready,
                        SCON_TI | SCON_RI,
                        timer1_trigger,
                        i2c_irq, //==INTx_sync [1],
                        timer0_trigger,
                        INTx_sync [0]}),
                        
            .int_addr (int_addr),
            .int_gen (int_gen)
    
        );
    
        
    
    //=======================================================================
    // debug counter and LED
    //=======================================================================
        
        
        debug_counter_led #(.REG_ADDR (DEBUG_LED_ADDR))  debug_counter_led_I (.*,
                .stb_i (WB_WR_STB_I),
                .we_i (WB_WR_WE_I),
                .adr_wr_i (WB_WR_ADR_I),
                .adr_rd_i (WB_RD_ADR_I),
                .dat_i (WB_WR_DAT_I),
                .dat_o (debug_counter_led_out),
                .ack_o (),
                
                .led (debug_led),
                .non_zero_pulse (debug_counter_pulse),
                .dog_bite (dog_bite)
        );
        
    //=======================================================================
    // JTAG UART
    //=======================================================================
    
        assign jtag_uart_read_n  = (WB_RD_ADR_I == JTAG_UART_ADDR) ? 1'b0 : 1'b1;
        assign jtag_uart_write_n = ((WB_WR_ADR_I == JTAG_UART_ADDR) && WB_WR_WE_I && WB_WR_STB_I) ? 1'b0 : 1'b1;
        
          
          always_ff @(posedge clk, negedge reset_n) begin
                if (!reset_n) begin
                      jtag_data_write_reg <= 0;
                end else if (!jtag_uart_write_n_d1) begin  
                      jtag_data_write_reg <= WB_WR_DAT_I_d1;
                end
          end
          
          always_ff @(posedge clk, negedge reset_n) begin
                if (!reset_n) begin
                    jtag_uart_write_n_d1 <= 1'b1;
                    jtag_uart_write_n_d2 <= 1'b1;
                    WB_WR_DAT_I_d1       <= 0;
                end else begin
                    jtag_uart_write_n_d1 <= jtag_uart_write_n;
                    jtag_uart_write_n_d2 <= jtag_uart_write_n_d1;
                   WB_WR_DAT_I_d1       <= WB_WR_DAT_I;

                end
          
          end
    
        generate
            if (FOR_SIM) begin : jtag_uart_gen
                JTAG_UART jtag_uart_i (
                    .clk_clk (clk), 
                    .jtag_uart_avalon_jtag_slave_chipselect (1'b1), 
                    .jtag_uart_avalon_jtag_slave_address (1'b0), 
                    .jtag_uart_avalon_jtag_slave_read_n (jtag_uart_read_n),
                    .jtag_uart_avalon_jtag_slave_readdata (jtag_uart_read_data),
                    .jtag_uart_avalon_jtag_slave_write_n (jtag_uart_write_n_d2),
                    .jtag_uart_avalon_jtag_slave_writedata ({24'd0, jtag_data_write_reg}),
                    .jtag_uart_avalon_jtag_slave_waitrequest (),
                    .jtag_uart_irq_irq (),
                    .reset_reset_n (reset_n));
            end else begin : jtag_uart_gen
                JTAG_UART jtag_uart_i (
                    .clk_clk (clk), 
                    .jtag_uart_avalon_jtag_slave_chipselect (1'b1), 
                    .jtag_uart_avalon_jtag_slave_address (1'b0), 
                    .jtag_uart_avalon_jtag_slave_read_n (jtag_uart_read_n),
                    .jtag_uart_avalon_jtag_slave_readdata (jtag_uart_read_data),
                    .jtag_uart_avalon_jtag_slave_write_n (jtag_uart_write_n_d2),
                    .jtag_uart_avalon_jtag_slave_writedata ({24'd0, jtag_data_write_reg}),
                    .jtag_uart_avalon_jtag_slave_waitrequest (),
                    .jtag_uart_irq_irq (),
                    .reset_reset_n (reset_n));
            end
        endgenerate
        
endmodule : peripherals

`default_nettype wire
