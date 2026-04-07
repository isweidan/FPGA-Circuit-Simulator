`timescale 1ns/1ps
import complex_pkg::*;

module LU_system_top #(parameter int N = 5) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic reuse,

    input  logic        a_we,
    input  logic [$clog2(N*N)-1:0] a_addr,
    input  logic signed [31:0] a_wdata_real,
    input  logic signed [31:0] a_wdata_imag,

    input  logic signed [32*N-1:0] b_flat_real_in,
    input  logic signed [32*N-1:0] b_flat_imag_in,

    output logic signed [32*N-1:0] x_flat_real_out,
    output logic signed [32*N-1:0] x_flat_imag_out,

    output logic done,
    output logic fail
);

    logic decomp_done, decomp_fail;
    logic [3*N-1:0] P_flat;
    logic signed [32*N*N-1:0] L_flat_real, L_flat_imag;
    logic signed [32*N*N-1:0] U_flat_real, U_flat_imag;

    logic signed [32*N-1:0] pb_flat_real, pb_flat_imag;

    // decomp divider
    logic        decomp_divisor_tvalid, decomp_dividend_tvalid, decomp_div_out_valid, decomp_div_by_zero;
    logic signed [31:0] decomp_divisor_tdata, decomp_divider_result;
    logic signed [47:0] decomp_dividend_tdata;
    logic [79:0] decomp_div_dout;

    // solver divider
    logic        solver_divisor_tvalid, solver_dividend_tvalid, solver_div_out_valid, solver_div_by_zero;
    logic signed [31:0] solver_divisor_tdata, solver_divider_result;
    logic signed [47:0] solver_dividend_tdata;
    logic [79:0] solver_div_dout;

    // decomp cmult
    logic        decomp_mult_a_tvalid, decomp_mult_b_tvalid, decomp_mult_out_valid;
    logic signed [63:0] decomp_mult_a_tdata, decomp_mult_b_tdata;
    logic [136:0] decomp_cmult_tdata;
    logic signed [129:0] decomp_mult_result_tdata;

    // solver cmult
    logic        solver_mult_a_tvalid, solver_mult_b_tvalid, solver_mult_out_valid;
    logic signed [63:0] solver_mult_a_tdata, solver_mult_b_tdata;
    logic [136:0] solver_cmult_tdata;
    logic signed [129:0] solver_mult_result_tdata;

    (* max_fanout = 500 *) logic solver_start;
    logic decomp_done_d;
    (* max_fanout = 500 *) logic decomp_rstn;
    (* max_fanout = 500 *) logic solver_rstn;

    always_comb begin
        pb_flat_real = '0;
        pb_flat_imag = '0;
        for (int r = 0; r < N; r++) begin
            pb_flat_real[32*r +: 32] = b_flat_real_in[32*P_flat[3*r +: 3] +: 32];
            pb_flat_imag[32*r +: 32] = b_flat_imag_in[32*P_flat[3*r +: 3] +: 32];
        end
    end
    
    always_ff @(posedge clk) begin
        decomp_rstn <= rst_n;
        solver_rstn <= rst_n;
    end

    assign decomp_divider_result = decomp_div_dout[63:32];
    assign solver_divider_result = solver_div_dout[63:32];

    assign decomp_mult_result_tdata = {$signed(decomp_cmult_tdata[136:72]), $signed(decomp_cmult_tdata[64:0])};

    assign solver_mult_result_tdata = {$signed(solver_cmult_tdata[136:72]), $signed(solver_cmult_tdata[64:0])};

    lu_decomp #(.N(N)) u_decomp (
        .clk(clk),
        .rst_n(decomp_rstn),
        .start(start),
        .done(decomp_done),
        .fail(decomp_fail),

        .a_we(a_we),
        .a_addr(a_addr),
        .a_wdata_real(a_wdata_real),
        .a_wdata_imag(a_wdata_imag),

        .divider_tvalid(decomp_div_out_valid),
        .divider_tdata(decomp_divider_result),
        .divisor_tvalid(decomp_divisor_tvalid),
        .divisor_tdata(decomp_divisor_tdata),
        .dividend_tvalid(decomp_dividend_tvalid),
        .dividend_tdata(decomp_dividend_tdata),

        .mult_result_tvalid(decomp_mult_out_valid),
        .mult_result_tdata(decomp_mult_result_tdata),
        .mult_a_tvalid(decomp_mult_a_tvalid),
        .mult_b_tvalid(decomp_mult_b_tvalid),
        .mult_a_tdata(decomp_mult_a_tdata),
        .mult_b_tdata(decomp_mult_b_tdata),

        .P_flat(P_flat),
        .L_flat_real(L_flat_real),
        .L_flat_imag(L_flat_imag),
        .U_flat_real(U_flat_real),
        .U_flat_imag(U_flat_imag)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            decomp_done_d <= 1'b0;
            solver_start  <= 1'b0;
        end else begin
            decomp_done_d <= decomp_done;
            solver_start  <= (decomp_done && !decomp_done_d && !decomp_fail) | reuse;
        end
    end

    lu_solver #(.N(N)) u_solver (
        .clk(clk),
        .resetn(solver_rstn),
        .start(solver_start),

        .L_flat_real(L_flat_real),
        .U_flat_real(U_flat_real),
        .b_flat_real(pb_flat_real),

        .L_flat_imag(L_flat_imag),
        .U_flat_imag(U_flat_imag),
        .b_flat_imag(pb_flat_imag),

        .divider_tvalid(solver_div_out_valid),
        .divider_tdata(solver_divider_result),
        .divisor_tvalid(solver_divisor_tvalid),
        .divisor_tdata(solver_divisor_tdata),
        .dividend_tvalid(solver_dividend_tvalid),
        .dividend_tdata(solver_dividend_tdata),

        .mult_result_tvalid(solver_mult_out_valid),
        .mult_result_tdata(solver_mult_result_tdata),
        .mult_a_tvalid(solver_mult_a_tvalid),
        .mult_b_tvalid(solver_mult_b_tvalid),
        .mult_a_tdata(solver_mult_a_tdata),
        .mult_b_tdata(solver_mult_b_tdata),

        .x_flat_real(x_flat_real_out),
        .x_flat_imag(x_flat_imag_out),
        .done(done)
    );

    assign fail = decomp_fail;

    div_gen_q1616 divider_core_decomp (
        .aclk(clk),
        .s_axis_divisor_tvalid(decomp_divisor_tvalid),
        .s_axis_divisor_tdata(decomp_divisor_tdata),
        .s_axis_dividend_tvalid(decomp_dividend_tvalid),
        .s_axis_dividend_tdata(decomp_dividend_tdata),
        .m_axis_dout_tvalid(decomp_div_out_valid),
        .m_axis_dout_tuser(decomp_div_by_zero),
        .m_axis_dout_tdata(decomp_div_dout)
    );

    div_gen_q1616 divider_core_solver (
        .aclk(clk),
        .s_axis_divisor_tvalid(solver_divisor_tvalid),
        .s_axis_divisor_tdata(solver_divisor_tdata),
        .s_axis_dividend_tvalid(solver_dividend_tvalid),
        .s_axis_dividend_tdata(solver_dividend_tdata),
        .m_axis_dout_tvalid(solver_div_out_valid),
        .m_axis_dout_tuser(solver_div_by_zero),
        .m_axis_dout_tdata(solver_div_dout)
    );

    cmpy_0 cmult_core_decomp (
        .aclk(clk),
        .s_axis_a_tvalid(decomp_mult_a_tvalid),
        .s_axis_a_tdata(decomp_mult_a_tdata),
        .s_axis_b_tvalid(decomp_mult_b_tvalid),
        .s_axis_b_tdata(decomp_mult_b_tdata),
        .m_axis_dout_tvalid(decomp_mult_out_valid),
        .m_axis_dout_tdata(decomp_cmult_tdata)
    );

    cmpy_0 cmult_core_solver (
        .aclk(clk),
        .s_axis_a_tvalid(solver_mult_a_tvalid),
        .s_axis_a_tdata(solver_mult_a_tdata),
        .s_axis_b_tvalid(solver_mult_b_tvalid),
        .s_axis_b_tdata(solver_mult_b_tdata),
        .m_axis_dout_tvalid(solver_mult_out_valid),
        .m_axis_dout_tdata(solver_cmult_tdata)
    );

endmodule