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
//  8051 Instruction Set Definition
//=============================================================================

`ifndef MCS_51_INSTRUCTIONS_SVH
`define MCS_51_INSTRUCTIONS_SVH

// Branch Instructions
parameter unsigned [4 : 0] INST_ACALL_LSB              = 5'b1_0001;
parameter unsigned [4 : 0] INST_AJMP_LSB               = 5'b0_0001;
parameter unsigned [7 : 0] INST_CJNE_A_DIR_REL         = 8'b1011_0101;
parameter unsigned [7 : 0] INST_CJNE_A_DATA_REL        = 8'b1011_0100;
parameter unsigned [4 : 0] INST_CJNE_R_DATA_REL_MSB    = 5'b1011_1;
parameter unsigned [6 : 0] INST_CJNE_AT_R_DATA_REL_MSB = 7'b1011_011;

parameter unsigned [4 : 0] INST_DJNZ_R_REL_MSB      = 5'b1101_1;
parameter unsigned [7 : 0] INST_DJNZ_DIR_REL        = 8'hD5;

parameter unsigned [7 : 0] INST_JB                  = 8'h20;
parameter unsigned [7 : 0] INST_JBC                 = 8'h10;
parameter unsigned [7 : 0] INST_JC                  = 8'h40;
parameter unsigned [7 : 0] INST_JMP_A_DPTR          = 8'h73;
parameter unsigned [7 : 0] INST_JNB                 = 8'h30;
parameter unsigned [7 : 0] INST_JNC                 = 8'h50;
parameter unsigned [7 : 0] INST_JNZ                 = 8'h70;
parameter unsigned [7 : 0] INST_JZ                  = 8'h60;
parameter unsigned [7 : 0] INST_LCALL               = 8'h12;
parameter unsigned [7 : 0] INST_LJMP                = 8'h02;
parameter unsigned [7 : 0] INST_SJMP                = 8'h80;
parameter unsigned [7 : 0] INST_NOP                 = 8'h00;
parameter unsigned [7 : 0] INST_RET                 = 8'h22;
parameter unsigned [7 : 0] INST_RETI                = 8'h32;

// Arithmetic Operations
parameter unsigned [3 : 0] INST_ADD_MSB             = 4'b0010;
parameter unsigned [4 : 0] INST_ADD_A_R_MSB         = 5'b0010_1;
parameter unsigned [7 : 0] INST_ADD_A_DIR           = 8'h25;
parameter unsigned [6 : 0] INST_ADD_A_AT_R_MSB      = 7'b0010_011;
parameter unsigned [7 : 0] INST_ADD_A_DATA          = 8'h24;


parameter unsigned [3 : 0] INST_ADDC_MSB            = 4'b0011;
parameter unsigned [4 : 0] INST_ADDC_A_R_MSB        = 5'b0011_1;
parameter unsigned [7 : 0] INST_ADDC_A_DIR          = 8'h35;
parameter unsigned [6 : 0] INST_ADDC_A_AT_R_MSB     = 7'b0011_011;
parameter unsigned [7 : 0] INST_ADDC_A_DATA         = 8'h34;

parameter unsigned [7 : 0] INST_DA_A                = 8'hD4;
parameter unsigned [3 : 0] INST_DEC_MSB             = 4'b0001;
parameter unsigned [7 : 0] INST_DEC_A               = 8'h14;
parameter unsigned [7 : 0] INST_DEC_DIR             = 8'h15;
parameter unsigned [4 : 0] INST_DEC_R_MSB           = 5'b0001_1;
parameter unsigned [6 : 0] INST_DEC_AT_R_MSB        = 7'b0001_011;

parameter unsigned [7 : 0] INST_MUL_AB              = 8'hA4;
parameter unsigned [7 : 0] INST_DIV_AB              = 8'h84;
parameter unsigned [3 : 0] INST_INC_MSB             = 4'b0000;
parameter unsigned [7 : 0] INST_INC_DPTR            = 8'hA3;
parameter unsigned [7 : 0] INST_INC_A               = 8'h04;
parameter unsigned [7 : 0] INST_INC_DIR             = 8'h05;
parameter unsigned [4 : 0] INST_INC_R_MSB           = 5'b0000_1;
parameter unsigned [6 : 0] INST_INC_AT_R_MSB        = 7'b0000_011;

parameter unsigned [4 : 0] INST_SUBB_A_R_MSB        = 5'b1001_1;
parameter unsigned [7 : 0] INST_SUBB_A_DIR          = 8'h95;
parameter unsigned [6 : 0] INST_SUBB_A_AT_R_MSB     = 7'b1001_011;
parameter unsigned [7 : 0] INST_SUBB_A_DATA         = 8'h94;

parameter unsigned [7 : 0] INST_SWAP_A              = 8'hC4;
parameter unsigned [4 : 0] INST_XCH_A_R_MSB         = 5'b1100_1;
parameter unsigned [7 : 0] INST_XCH_A_DIR           = 8'hC5;
parameter unsigned [6 : 0] INST_XCH_A_AT_R_MSB      = 7'b1100_011;
parameter unsigned [6 : 0] INST_XCHD_A_AT_R_MSB     = 7'b1101_011;

parameter unsigned [4 : 0] INST_XRL_A_R_MSB         = 5'b0110_1;
parameter unsigned [7 : 0] INST_XRL_A_DIR           = 8'h65;
parameter unsigned [6 : 0] INST_XRL_A_AT_R_MSB      = 7'b0110_011;
parameter unsigned [7 : 0] INST_XRL_A_DATA          = 8'h64;
parameter unsigned [7 : 0] INST_XRL_DIR_A           = 8'h62;
parameter unsigned [7 : 0] INST_XRL_DIR_DATA        = 8'h63;


// unsignedal Operations and Boolean Variable Manipulation
parameter unsigned [3 : 0] INST_ANL_MSB             = 4'b0101;
parameter unsigned [4 : 0] INST_ANL_A_R_MSB         = 5'b0101_1;
parameter unsigned [7 : 0] INST_ANL_A_DIR           = 8'h55;
parameter unsigned [6 : 0] INST_ANL_A_AT_R_MSB      = 7'b0101_011;
parameter unsigned [7 : 0] INST_ANL_A_DATA          = 8'h54;
parameter unsigned [7 : 0] INST_ANL_DIR_A           = 8'h52;
parameter unsigned [7 : 0] INST_ANL_DIR_DATA        = 8'h53;


parameter unsigned [7 : 0] INST_ANL_C_BIT           = 8'h82;
parameter unsigned [7 : 0] INST_ANL_C_NBIT          = 8'hB0;

parameter unsigned [7 : 0] INST_CLR_A               = 8'hE4;
parameter unsigned [7 : 0] INST_CLR_C               = 8'hC3;
parameter unsigned [7 : 0] INST_CLR_BIT             = 8'hC2;

parameter unsigned [7 : 0] INST_CPL_A               = 8'hF4;
parameter unsigned [7 : 0] INST_CPL_C               = 8'hB3;
parameter unsigned [7 : 0] INST_CPL_BIT             = 8'hB2;

parameter unsigned [4 : 0] INST_ORL_A_R_MSB         = 5'b0100_1;
parameter unsigned [7 : 0] INST_ORL_A_DIR           = 8'h45;
parameter unsigned [6 : 0] INST_ORL_A_AT_R_MSB      = 7'b0100_011;
parameter unsigned [7 : 0] INST_ORL_A_DATA          = 8'h44;
parameter unsigned [7 : 0] INST_ORL_DIR_A           = 8'h42;
parameter unsigned [7 : 0] INST_ORL_DIR_DATA        = 8'h43;

parameter unsigned [7 : 0] INST_ORL_C_BIT           = 8'h72;
parameter unsigned [7 : 0] INST_ORL_C_NBIT          = 8'hA0;

parameter unsigned [7 : 0] INST_RL_A                = 8'h23;
parameter unsigned [7 : 0] INST_RLC_A               = 8'h33;
parameter unsigned [7 : 0] INST_RR_A                = 8'h03;
parameter unsigned [7 : 0] INST_RRC_A               = 8'h13;

parameter unsigned [7 : 0] INST_SETB_C              = 8'hD3;
parameter unsigned [7 : 0] INST_SETB_BIT            = 8'hD2;

// Data Transfer
parameter unsigned [3 : 0] INST_MOV_A_MSB           = 4'b1110;
parameter unsigned [4 : 0] INST_MOV_A_R_MSB         = 5'b1110_1;
parameter unsigned [7 : 0] INST_MOV_A_DIR           = 8'hE5;
parameter unsigned [6 : 0] INST_MOV_A_AT_R_MSB      = 7'b1110_011;

parameter unsigned [7 : 0] INST_MOV_A_DATA          = 8'h74;

parameter unsigned [4 : 0] INST_MOV_R_A_MSB         = 5'b1111_1;
parameter unsigned [4 : 0] INST_MOV_R_DIR_MSB       = 5'b1010_1;
parameter unsigned [4 : 0] INST_MOV_R_DATA_MSB      = 5'b0111_1;

parameter unsigned [7 : 0] INST_MOV_DIR_A           = 8'hF5;
parameter unsigned [4 : 0] INST_MOV_DIR_R_MSB       = 5'b10001;
parameter unsigned [7 : 0] INST_MOV_DIR_DIR         = 8'h85;
parameter unsigned [6 : 0] INST_MOV_DIR_AT_R_MSB    = 7'b1000_011;
parameter unsigned [7 : 0] INST_MOV_DIR_DATA        = 8'h75;
parameter unsigned [6 : 0] INST_MOV_AT_R_A_MSB      = 7'b1111_011;
parameter unsigned [6 : 0] INST_MOV_AT_R_DIR_MSB    = 7'b1010_011;
parameter unsigned [6 : 0] INST_MOV_AT_R_DATA_MSB   = 7'b0111_011;

parameter unsigned [7 : 0] INST_MOV_C_BIT           = 8'hA2;
parameter unsigned [7 : 0] INST_MOV_BIT_C           = 8'h92;
parameter unsigned [7 : 0] INST_MOV_DPTR_DATA       = 8'h90;

parameter unsigned [7 : 0] INST_MOVC_A_DPTR         = 8'h93;
parameter unsigned [7 : 0] INST_MOVC_A_PC           = 8'h83;

parameter unsigned [6 : 0] INST_MOVX_A_AT_R_MSB     = 7'b1110_001;
parameter unsigned [7 : 0] INST_MOVX_A_AT_DPTR      = 8'hE0;
parameter unsigned [6 : 0] INST_MOVX_AT_R_A_MSB     = 7'b1111_001;
parameter unsigned [7 : 0] INST_MOVX_AT_DPTR_A      = 8'hF0;

parameter unsigned [7 : 0] INST_POP                 = 8'hD0;
parameter unsigned [7 : 0] INST_PUSH                = 8'hC0;

`endif
