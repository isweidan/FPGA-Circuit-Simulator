`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/06/2026 12:11:21 AM
// Design Name: 
// Module Name: complex_pkg
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

package complex_pkg;

parameter INPUT_INTEGER_WIDTH = 'd16;
parameter INPUT_FRACTIONAL_WIDTH = 'd16;
parameter INPUT_DATA_WIDTH  = INPUT_INTEGER_WIDTH + INPUT_FRACTIONAL_WIDTH;
parameter SUM_DATA_WIDTH    = 'd48;
parameter MULT_DATA_WIDTH   = 'd64;
parameter DIV_DATA_WIDTH    = 'd64;

typedef struct packed {
    logic signed [31:0] r;
    logic signed [31:0] i;
} complex_t;

typedef struct packed {
    logic signed [SUM_DATA_WIDTH-1:0] r;
    logic signed [SUM_DATA_WIDTH-1:0] i;
} complex_add_t;

endpackage
