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
//      Special Function Register Definition
//=============================================================================


`ifndef SFR_SVH
`define SFR_SVH

`include "common.svh"

parameter unsigned [DATA_WIDTH - 1 : 0] P0_ADDR     = 8'h80;
parameter unsigned [DATA_WIDTH - 1 : 0] SPL_ADDR    = 8'h81;
parameter unsigned [DATA_WIDTH - 1 : 0] DPL_ADDR    = 8'h82;
parameter unsigned [DATA_WIDTH - 1 : 0] DPH_ADDR    = 8'h83;

parameter unsigned [DATA_WIDTH - 1 : 0] PCON_ADDR   = 8'h87;
parameter unsigned [DATA_WIDTH - 1 : 0] TCON_ADDR   = 8'h88;
parameter unsigned [DATA_WIDTH - 1 : 0] TMOD_ADDR   = 8'h89;

parameter unsigned [DATA_WIDTH - 1 : 0] TL0_ADDR    = 8'h8A;
parameter unsigned [DATA_WIDTH - 1 : 0] TL1_ADDR    = 8'h8B;
parameter unsigned [DATA_WIDTH - 1 : 0] TH0_ADDR    = 8'h8C;
parameter unsigned [DATA_WIDTH - 1 : 0] TH1_ADDR    = 8'h8D;

parameter unsigned [DATA_WIDTH - 1 : 0] P1_ADDR     = 8'h90;

parameter unsigned [DATA_WIDTH - 1 : 0] SCON_ADDR   = 8'h98;
parameter unsigned [DATA_WIDTH - 1 : 0] SBUF_ADDR   = 8'h99;

parameter unsigned [DATA_WIDTH - 1 : 0] SCON_AUX_ADDR  = 8'h9A;
parameter unsigned [DATA_WIDTH - 1 : 0] SBUF_AUX_ADDR  = 8'h9B;

parameter unsigned [DATA_WIDTH - 1 : 0] P2_ADDR     = 8'hA0;
parameter unsigned [DATA_WIDTH - 1 : 0] IE_ADDR     = 8'hA8;

parameter unsigned [DATA_WIDTH - 1 : 0] P3_ADDR     = 8'hB0;

parameter unsigned [DATA_WIDTH - 1 : 0] IP_ADDR     = 8'hB8;
parameter unsigned [DATA_WIDTH - 1 : 0] DEBUG_LED_ADDR  = 8'hC0;

parameter unsigned [DATA_WIDTH - 1 : 0] LCD_CSR_ADDR  = 8'hC1;
parameter unsigned [DATA_WIDTH - 1 : 0] LCD_DATA_ADDR = 8'hC2;

parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_CSR_ADDR  = 8'hC3;
parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_DATA_ADDR = 8'hC4;

parameter unsigned [DATA_WIDTH - 1 : 0] PSW_ADDR    = 8'hD0;
parameter integer PSW_CY_INDEX                      = 7;
parameter integer PSW_AC_INDEX                      = 6;
parameter integer PSW_F0_INDEX                      = 5;
parameter integer PSW_RS1_INDEX                     = 4;
parameter integer PSW_RS0_INDEX                     = 3;
parameter integer PSW_OV_INDEX                      = 2;
parameter integer PSW_UD_INDEX                      = 1;
parameter integer PSW_P_INDEX                       = 0;

parameter unsigned [DATA_WIDTH - 1 : 0] I2C_CSR_ADDR        = 8'hD1;
parameter unsigned [DATA_WIDTH - 1 : 0] I2C_ADDR_DATA_ADDR  = 8'hD2;

parameter unsigned [DATA_WIDTH - 1 : 0] PWM_CSR_ADDR        = 8'hD3;
parameter unsigned [DATA_WIDTH - 1 : 0] PWM_DATA_ADDR       = 8'hD4;

parameter unsigned [DATA_WIDTH - 1 : 0] PS2_CSR_ADDR    = 8'hD5;
parameter unsigned [DATA_WIDTH - 1 : 0] PS2_DATA_ADDR   = 8'hD6;



parameter unsigned [DATA_WIDTH - 1 : 0] SD_CSR_ADDR      = 8'hD7;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_CMD_ADDR      = 8'hD8;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_ARG0_ADDR     = 8'hD9;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_ARG1_ADDR     = 8'hDA;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_ARG2_ADDR     = 8'hDB;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_ARG3_ADDR     = 8'hDC;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_BUF_ADDR      = 8'hDD;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_DATA_IN_ADDR  = 8'hDE;
parameter unsigned [DATA_WIDTH - 1 : 0] SD_DATA_OUT_ADDR = 8'hDF;

parameter unsigned [DATA_WIDTH - 1 : 0] ACC_ADDR    = 8'hE0;

parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_LOADER_DATA0_ADDR     = 8'hE1;
parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_LOADER_DATA1_ADDR     = 8'hE2;
parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_LOADER_DATA2_ADDR     = 8'hE3;
parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_LOADER_DATA3_ADDR     = 8'hE4;
parameter unsigned [DATA_WIDTH - 1 : 0] FLASH_LOADER_CSR_ADDR       = 8'hE5;

parameter unsigned [DATA_WIDTH - 1 : 0] MCU_REVISION_ADDR           = 8'hE6;

parameter unsigned [DATA_WIDTH - 1 : 0] XPAGE_ADDR                  = 8'hE7;

parameter unsigned [DATA_WIDTH - 1 : 0] CODEC_WRITE_DATA_LOW_ADDR   = 8'hE8;
parameter unsigned [DATA_WIDTH - 1 : 0] CODEC_WRITE_DATA_HIGH_ADDR  = 8'hE9;
parameter unsigned [DATA_WIDTH - 1 : 0] CODEC_READ_DATA_LOW_ADDR    = 8'hEA;
parameter unsigned [DATA_WIDTH - 1 : 0] CODEC_READ_DATA_HIGH_ADDR   = 8'hEB;
parameter unsigned [DATA_WIDTH - 1 : 0] CODEC_CSR_ADDR              = 8'hEC;

parameter unsigned [DATA_WIDTH - 1 : 0] CHIP_ID_DATA_CSR_ADDR       = 8'hED;

parameter unsigned [DATA_WIDTH - 1 : 0] SPH_ADDR    = 8'hEF;

parameter unsigned [DATA_WIDTH - 1 : 0] B_ADDR      = 8'hF0;

parameter unsigned [DATA_WIDTH - 1 : 0] P0_DIRECTION_ADDR = 8'hF1;
parameter unsigned [DATA_WIDTH - 1 : 0] P1_DIRECTION_ADDR = 8'hF2;
parameter unsigned [DATA_WIDTH - 1 : 0] P2_DIRECTION_ADDR = 8'hF3;
parameter unsigned [DATA_WIDTH - 1 : 0] P3_DIRECTION_ADDR = 8'hF4;

parameter unsigned [DATA_WIDTH - 1 : 0] ADC_DATA_HIGH_ADDR = 8'hF5;
parameter unsigned [DATA_WIDTH - 1 : 0] ADC_DATA_LOW_ADDR  = 8'hF6;
parameter unsigned [DATA_WIDTH - 1 : 0] ADC_CSR_ADDR       = 8'hF7;

parameter unsigned [DATA_WIDTH - 1 : 0] JTAG_UART_ADDR = 8'hF8;

parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_INSTRUCTION_ADDR = 8'hF9;
parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_DATA_ADDR        = 8'hFA;
parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_ADDRESS2_ADDR    = 8'hFB;
parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_ADDRESS1_ADDR    = 8'hFC;
parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_ADDRESS0_ADDR    = 8'hFD;
parameter unsigned [DATA_WIDTH - 1 : 0] SRAM_CSR_ADDR         = 8'hFE;

parameter unsigned [DATA_WIDTH - 1 : 0] ROTARY_ENCODER_ADDR   = 8'hFF;



parameter integer IE_GLOBAL_INT_ENABLE_INDEX         = 7;
parameter integer IE_ENABLE_CODEC_INDEX              = 6;
parameter integer IE_ENABLE_PS2_INDEX                = 5;
parameter integer IE_ENABLE_SERIAL_INT_INDEX         = 4;
parameter integer IE_ENABLE_TIMER1_INT_INDEX         = 3;
parameter integer IE_ENABLE_EXT_INT1_INDEX           = 2;
parameter integer IE_ENABLE_TIMER0_INT_INDEX         = 1;
parameter integer IE_ENABLE_EXT_INT0_INDEX           = 0;


parameter integer TCON_TR0_INDEX                     = 4;
parameter integer TCON_TF0_INDEX                     = 5;
parameter integer TCON_TR1_INDEX                     = 6;
parameter integer TCON_TF1_INDEX                     = 7;

`endif
