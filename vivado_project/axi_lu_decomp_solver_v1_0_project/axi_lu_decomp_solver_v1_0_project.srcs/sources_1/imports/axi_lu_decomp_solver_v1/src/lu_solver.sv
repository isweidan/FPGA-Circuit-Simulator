`timescale 1ns / 1ps
import complex_pkg::*;

module lu_solver #(parameter int N = 3) (
    input  logic clk,
    input  logic resetn,
    input  logic start,

    input  logic signed [32*N*N-1:0] L_flat_real,
    input  logic signed [32*N*N-1:0] U_flat_real,
    input  logic signed [32*N-1:0]   b_flat_real,

    input  logic signed [32*N*N-1:0] L_flat_imag,
    input  logic signed [32*N*N-1:0] U_flat_imag,
    input  logic signed [32*N-1:0]   b_flat_imag,

    input  logic        divider_tvalid,
    input  logic signed [31:0] divider_tdata,
    output logic        divisor_tvalid,
    output logic signed [31:0] divisor_tdata,
    output logic        dividend_tvalid,
    output logic signed [47:0] dividend_tdata,

    input  logic        mult_result_tvalid,
    input  logic signed [129:0] mult_result_tdata,
    output logic        mult_a_tvalid,
    output logic        mult_b_tvalid,
    output logic signed [63:0] mult_a_tdata,
    output logic signed [63:0] mult_b_tdata,

    output logic signed [32*N-1:0] x_flat_real,
    output logic signed [32*N-1:0] x_flat_imag,
    output logic done
);

    localparam int FB = INPUT_FRACTIONAL_WIDTH;
    localparam int ACC_W = SUM_DATA_WIDTH;

    typedef enum logic [4:0] {
        IDLE,
        FORWARD,
        BACKWARD,
        MULT_PREP_START,
        MULT_START,
        MULT_WAIT,
        DIV_PREP_START,
        DIV_REAL_PREP1,
        DIV_REAL_PREP2,
        DIV_REAL_START,
        DIV_REAL_WAIT,
        DIV_IMAG_PREP1,
        DIV_IMAG_PREP2,
        DIV_IMAG_START,
        DIV_IMAG_WAIT,
        DONE_ST
    } state_t;

    state_t state;
    logic phase_forward;

    (* max_fanout = 100 *) integer i, j, k;

    (* ram_style = "distributed" *) complex_t L [N-1:0][N-1:0];
    (* ram_style = "distributed" *) complex_t U [N-1:0][N-1:0];
    complex_t b [N-1:0];
    complex_t y [N-1:0];
    complex_t x [N-1:0];

    logic signed [ACC_W-1:0] sum_r, sum_i;

    logic signed [63:0] num_r, num_i;
    logic signed [63:0] den_r, den_i;
    logic signed [63:0] c2, d2, mag;
    logic signed [63:0] real_num, imag_num;
    logic signed [63:0] real_num_comp1, real_num_comp2, imag_num_comp1, imag_num_comp2, mag_comp1, mag_comp2;
    complex_t mult_a_temp, mult_b_temp;
    
    logic signed [31:0] div_q_real;

    function automatic logic signed [63:0] sx32(input logic signed [31:0] v);
        sx32 = {{32{v[31]}}, v};
    endfunction

    function automatic logic signed [31:0] q32_32_to_q16_16_trunc(input logic signed [64:0] v);
        q32_32_to_q16_16_trunc = v[47:16];
    endfunction

    integer r, c;  
    always_ff @(posedge clk) begin
        if(start) begin
            for(int r = 0; r < N; r++) begin
                for(int c = 0; c < N; c++) begin
                    L[r][c].r <= L_flat_real[32*(r*N+c) +: 32];
                    L[r][c].i <= L_flat_imag[32*(r*N+c) +: 32];
                    U[r][c].r <= U_flat_real[32*(r*N+c) +: 32];
                    U[r][c].i <= U_flat_imag[32*(r*N+c) +: 32];
                end
                b[r].r <= b_flat_real[32*r +: 32];
                b[r].i <= b_flat_imag[32*r +: 32];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            state <= IDLE;
            phase_forward <= 1'b1;
            done <= 1'b0;

            i <= 0;
            j <= 0;
            sum_r <= '0;
            sum_i <= '0;

            num_r <= '0; num_i <= '0;
            den_r <= '0; den_i <= '0;
            c2 <= '0; d2 <= '0; mag <= '0;
            real_num <= '0; imag_num <= '0;
            div_q_real <= '0;

            x_flat_real <= '0;
            x_flat_imag <= '0;

            for (k = 0; k < N; k++) begin
                y[k].r <= '0; y[k].i <= '0;
                x[k].r <= '0; x[k].i <= '0;
            end

            divisor_tvalid  <= 1'b0;
            dividend_tvalid <= 1'b0;
            divisor_tdata   <= '0;
            dividend_tdata  <= '0;

            mult_a_tvalid <= 1'b0;
            mult_b_tvalid <= 1'b0;
            mult_a_tdata  <= '0;
            mult_b_tdata  <= '0;
        end else begin
            done <= 1'b0;

            divisor_tvalid  <= 1'b0;
            dividend_tvalid <= 1'b0;
            divisor_tdata   <= '0;
            dividend_tdata  <= '0;

            mult_a_tvalid <= 1'b0;
            mult_b_tvalid <= 1'b0;
            mult_a_tdata  <= '0;
            mult_b_tdata  <= '0;

            case (state)
                IDLE: begin
                    phase_forward <= 1'b1;
                    i <= 0;
                    j <= 0;
                    sum_r <= '0;
                    sum_i <= '0;

                    if (start) begin
                        for (k = 0; k < N; k++) begin
                            y[k].r <= '0; y[k].i <= '0;
                            x[k].r <= '0; x[k].i <= '0;
                        end
                        state <= FORWARD;
                    end
                end

                FORWARD: begin
                    if (j < i) begin
                        phase_forward <= 1'b1;
                        state <= MULT_PREP_START;
                    end else begin
                        phase_forward <= 1'b1;
                        state <= DIV_PREP_START;
                    end
                end

                BACKWARD: begin
                    if (j > i) begin
                        phase_forward <= 1'b0;
                        state <= MULT_PREP_START;
                    end else begin
                        phase_forward <= 1'b0;
                        state <= DIV_PREP_START;
                    end
                end
                
                MULT_PREP_START: begin
                    if(phase_forward) begin
                        mult_a_temp <= L[i][j];
                        mult_b_temp <= y[j];
                    end else begin
                        mult_a_temp <= U[i][j];
                        mult_b_temp <= x[j];
                    end
                    state <= MULT_START;
                end

                MULT_START: begin
                    mult_a_tvalid <= 1'b1;
                    mult_b_tvalid <= 1'b1;
                    mult_a_tdata <= {$signed(mult_a_temp.i), $signed(mult_a_temp.r)};
                    mult_b_tdata <= {$signed(mult_b_temp.i), $signed(mult_b_temp.r)};
                    state <= MULT_WAIT;
                end

                MULT_WAIT: begin
                    if (mult_result_tvalid) begin
                        sum_r <= sum_r + $signed(q32_32_to_q16_16_trunc($signed(mult_result_tdata[64:0])));
                        sum_i <= sum_i + $signed(q32_32_to_q16_16_trunc($signed(mult_result_tdata[129:65])));
                        if (phase_forward) begin
                            j <= j + 1;
                            state <= FORWARD;
                        end else begin
                            j <= j - 1;
                            state <= BACKWARD;
                        end
                    end
                end

                DIV_PREP_START: begin
                    if (phase_forward) begin
                        num_r <= sx32($signed(b[i].r) - $signed(sum_r));
                        num_i <= sx32($signed(b[i].i) - $signed(sum_i));
                        den_r <= sx32(L[i][i].r);
                        den_i <= sx32(L[i][i].i);
                    end else begin
                        num_r <= sx32($signed(y[i].r) - $signed(sum_r));
                        num_i <= sx32($signed(y[i].i) - $signed(sum_i));
                        den_r <= sx32(U[i][i].r);
                        den_i <= sx32(U[i][i].i);
                    end
                    state <= DIV_REAL_PREP1;
                end

                DIV_REAL_PREP1: begin
                    //real_num <= ((num_r * den_r) >>> FB) + ((num_i * den_i) >>> FB);
                    //imag_num <= ((num_i * den_r) >>> FB) - ((num_r * den_i) >>> FB);
                    //state <= DIV_REAL_START;
                    real_num_comp1 <= (num_r * den_r) >>> FB;
                    real_num_comp2 <= (num_i * den_i) >>> FB;
                    mag_comp1 <= (den_r * den_r) >>> FB;
                    mag_comp2 <= (den_i * den_i) >>> FB;
                    state <= DIV_REAL_PREP2;
                end
                
                DIV_REAL_PREP2: begin
                    real_num <= real_num_comp1 + real_num_comp2;
                    mag <= mag_comp1 + mag_comp2;
                    state <= DIV_REAL_START;
                end

                DIV_REAL_START: begin
                    divisor_tvalid  <= 1'b1;
                    dividend_tvalid <= 1'b1;
                    divisor_tdata   <= mag[31:0];
                    dividend_tdata  <= $signed(real_num <<< FB);
                    state <= DIV_IMAG_PREP1;
                end
                
                DIV_IMAG_PREP1: begin
                    imag_num_comp1 <= (num_i * den_r) >>> FB;
                    imag_num_comp2 <= (num_r * den_i) >>> FB;
                    state <= DIV_IMAG_PREP2;
                end
                
                DIV_IMAG_PREP2: begin
                    imag_num <= imag_num_comp1 - imag_num_comp2;
                    state <= DIV_IMAG_START;
                end

                DIV_REAL_WAIT: begin
                    if (divider_tvalid) begin
                        div_q_real <= divider_tdata;
                        state <= DIV_IMAG_WAIT;
                    end
                end

                DIV_IMAG_START: begin
                    divisor_tvalid  <= 1'b1;
                    dividend_tvalid <= 1'b1;
                    divisor_tdata   <= mag[31:0];
                    dividend_tdata  <= $signed(imag_num <<< FB);
                    state <= DIV_REAL_WAIT;
                end

                DIV_IMAG_WAIT: begin
                    if (divider_tvalid) begin
                        if (phase_forward) begin
                            y[i].r <= div_q_real;
                            y[i].i <= divider_tdata;
                            if (i == N-1) begin
                                i <= N-1;
                                j <= N-1;
                                sum_r <= '0;
                                sum_i <= '0;
                                phase_forward <= 1'b0;
                                state <= BACKWARD;
                            end else begin
                                i <= i + 1;
                                j <= 0;
                                sum_r <= '0;
                                sum_i <= '0;
                                state <= FORWARD;
                            end
                        end else begin
                            x[i].r <= div_q_real;
                            x[i].i <= divider_tdata;
                            if (i == 0) begin
                                state <= DONE_ST;
                            end else begin
                                i <= i - 1;
                                j <= N-1;
                                sum_r <= '0;
                                sum_i <= '0;
                                state <= BACKWARD;
                            end
                        end
                    end
                end

                DONE_ST: begin
                    done <= 1'b1;
                    for (k = 0; k < N; k++) begin
                        x_flat_real[32*k +: 32] <= x[k].r;
                        x_flat_imag[32*k +: 32] <= x[k].i;
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule