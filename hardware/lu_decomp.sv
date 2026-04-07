`timescale 1ns/1ps
import complex_pkg::*;


module lu_decomp #(parameter int N = 3) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,
    output logic fail,


    input  logic        a_we,
    input  logic [$clog2(N*N)-1:0] a_addr,
    input  logic signed [31:0] a_wdata_real,
    input  logic signed [31:0] a_wdata_imag,


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


    output logic [3*N-1:0] P_flat,
    output logic signed [32*N*N-1:0] L_flat_real,
    output logic signed [32*N*N-1:0] L_flat_imag,
    output logic signed [32*N*N-1:0] U_flat_real,
    output logic signed [32*N*N-1:0] U_flat_imag
);


    localparam int FB = INPUT_FRACTIONAL_WIDTH;


    typedef enum logic [4:0] {
        S_IDLE,
        S_INIT,
        S_K_START,
        S_PIVOT_FETCH,
        S_PIVOT_MAG1,
        S_PIVOT_MAG2,
        S_PIVOT_MAG3,
        S_PIVOT_SCAN,
        S_SWAP_U,
        S_SWAP_L,
        S_ROW_PREP,
        S_DIV_REAL_PREP1,
        S_DIV_REAL_PREP2,
        S_DIV_REAL_PREP3,
        S_DIV_REAL_START,
        S_DIV_REAL_WAIT,
        S_DIV_IMAG_PREP1,
        S_DIV_IMAG_PREP2,
        S_DIV_IMAG_START,
        S_DIV_IMAG_WAIT,
        S_COL_PREP,
        S_MULT_START,
        S_MULT_WAIT,
        S_NEXT_ROW,
        S_NEXT_K,
        S_DONE
    } state_t;


    state_t state;


    complex_t A [0:N*N-1];
    (* ram_style = "distributed" *) complex_t L [0:N*N-1];
    (* ram_style = "distributed" *) complex_t U [0:N*N-1];
    complex_t factor;


    logic [2:0] P [0:N-1];


    (* max_fanout = 100 *) integer k;
    (* max_fanout = 100 *) integer i, j, t;
    integer scan_row;
    integer pivot_row;
    integer swap_col;


    logic signed [63:0] num_r, num_i;
    logic signed [63:0] den_r, den_i;
    logic signed [63:0] c2, d2, mag;
    logic signed [63:0] real_num, imag_num;
    logic signed [63:0] real_num_comp1, real_num_comp2, imag_num_comp1, imag_num_comp2, mag_comp1, mag_comp2;
    logic signed [63:0] pivot_mag;
    complex_t u_temp_reg;
    logic signed [63:0] mag_real, mag_imag, curr_mag2;

    logic signed [31:0] tmp_r, tmp_i;
    logic [2:0] tmp_p;


    function automatic integer IDX(input integer r, input integer c);
        IDX = r*N + c;
    endfunction


    function automatic logic signed [63:0] sx32(input logic signed [31:0] v);
        sx32 = {{32{v[31]}}, v};
    endfunction

    function automatic logic signed [31:0] q32_32_to_q16_16_trunc(input logic signed [64:0] v);
        q32_32_to_q16_16_trunc = v[47:16];
    endfunction


    function automatic logic signed [63:0] mag2(input complex_t z);
        mag2 = (((sx32(z.r) * sx32(z.r)) >>> FB) +
                ((sx32(z.i) * sx32(z.i)) >>> FB));
    endfunction


    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (t = 0; t < N*N; t++) begin
                A[t].r <= '0; A[t].i <= '0;
            end
        end else if (a_we) begin
            A[a_addr].r <= a_wdata_real;
            A[a_addr].i <= a_wdata_imag;
        end
    end


    always_ff @(posedge clk) begin
        for (int r = 0; r < N; r++) begin
            P_flat[3*r +: 3] <= P[r];
        end

        for (int q = 0; q < N*N; q++) begin
            L_flat_real[32*q +: 32] <= L[q].r;
            L_flat_imag[32*q +: 32] <= L[q].i;
            U_flat_real[32*q +: 32] <= U[q].r;
            U_flat_imag[32*q +: 32] <= U[q].i;
        end
    end


    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (t = 0; t < N*N; t++) begin
                L[t].r <= '0; L[t].i <= '0;
                U[t].r <= '0; U[t].i <= '0;
            end
            state <= S_IDLE;
            done  <= 1'b0;
            fail  <= 1'b0;


            k <= 0; i <= 0; j <= 0;
            scan_row <= 0;
            pivot_row <= 0;
            swap_col <= 0;


            factor.r <= '0; factor.i <= '0;
            num_r <= '0; num_i <= '0;
            den_r <= '0; den_i <= '0;
            c2 <= '0; d2 <= '0; mag <= '0;
            real_num <= '0; imag_num <= '0;
            pivot_mag <= '0;
            u_temp_reg <= 0;

            tmp_r <= '0; tmp_i <= '0; tmp_p <= '0;

            for (t = 0; t < N; t++) begin
                P[t] <= t[2:0];
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
                S_IDLE: begin
                    fail <= 1'b0;
                    if (start)
                        state <= S_INIT;
                end


                S_INIT: begin
                    for (t = 0; t < N*N; t++) begin
                        U[t] <= A[t];
                        L[t].r <= '0;
                        L[t].i <= '0;
                    end
                    for (t = 0; t < N; t++) begin
                        L[IDX(t,t)].r <= (32'sd1 <<< FB);
                        L[IDX(t,t)].i <= '0;
                        P[t] <= t[2:0];
                    end
                    k <= 0;
                    state <= S_K_START;
                end


                S_K_START: begin
                    /*
                    pivot_row <= k;
                    pivot_mag <= mag2(U[IDX(k,k)]);
                    scan_row  <= k + 1;
                    state <= S_PIVOT_FETCH;
                    */
                    pivot_row <= k;
                    scan_row <= k;
                    pivot_mag <= 'd0;
                    state <= S_PIVOT_FETCH;
                end
                
                S_PIVOT_FETCH: begin
                    if (scan_row < N) begin
                        u_temp_reg <= U[IDX(scan_row, k)];
                        state <= S_PIVOT_MAG1;
                    end else begin
                        if (pivot_mag == 0) begin
                            fail <= 1'b1;
                            state <= S_DONE;
                        end else if (pivot_row != k) begin
                            swap_col <= 0;
                            state <= S_SWAP_U;
                        end else begin
                            i <= k + 1;
                            state <= S_ROW_PREP;
                        end 
                    end
                end
                
                S_PIVOT_MAG1: begin
                    mag_real <= sx32(u_temp_reg.r) * sx32(u_temp_reg.r);
                    mag_imag <= sx32(u_temp_reg.i) * sx32(u_temp_reg.i);
                    state <= S_PIVOT_MAG2;
                end
                
                S_PIVOT_MAG2: begin
                    curr_mag2 <= (mag_imag + mag_real) >>> FB;
                    state <= S_PIVOT_SCAN;
                end

                S_PIVOT_SCAN: begin
                    if (curr_mag2 > pivot_mag) begin
                        pivot_mag <= curr_mag2;
                        pivot_row <= scan_row;
                    end
                    scan_row <= scan_row + 1;
                    state <= S_PIVOT_FETCH;
                end


                S_SWAP_U: begin
                    tmp_r = U[IDX(k,swap_col)].r;
                    tmp_i = U[IDX(k,swap_col)].i;


                    U[IDX(k,swap_col)].r <= U[IDX(pivot_row,swap_col)].r;
                    U[IDX(k,swap_col)].i <= U[IDX(pivot_row,swap_col)].i;
                    U[IDX(pivot_row,swap_col)].r <= tmp_r;
                    U[IDX(pivot_row,swap_col)].i <= tmp_i;


                    if (swap_col == N-1) begin
                        tmp_p = P[k];
                        P[k] <= P[pivot_row];
                        P[pivot_row] <= tmp_p;


                        if (k == 0) begin
                            i <= k + 1;
                            state <= S_ROW_PREP;
                        end else begin
                            swap_col <= 0;
                            state <= S_SWAP_L;
                        end
                    end else begin
                        swap_col <= swap_col + 1;
                    end
                end


                S_SWAP_L: begin
                    tmp_r = L[IDX(k,swap_col)].r;
                    tmp_i = L[IDX(k,swap_col)].i;


                    L[IDX(k,swap_col)].r <= L[IDX(pivot_row,swap_col)].r;
                    L[IDX(k,swap_col)].i <= L[IDX(pivot_row,swap_col)].i;
                    L[IDX(pivot_row,swap_col)].r <= tmp_r;
                    L[IDX(pivot_row,swap_col)].i <= tmp_i;


                    if (swap_col == k-1) begin
                        i <= k + 1;
                        state <= S_ROW_PREP;
                    end else begin
                        swap_col <= swap_col + 1;
                    end
                end


                S_ROW_PREP: begin
                    if (i >= N)
                        state <= S_NEXT_K;
                    else
                        state <= S_DIV_REAL_PREP1;
                end


                S_DIV_REAL_PREP1: begin
                    num_r <= sx32(U[IDX(i,k)].r);
                    num_i <= sx32(U[IDX(i,k)].i);
                    den_r <= sx32(U[IDX(k,k)].r);
                    den_i <= sx32(U[IDX(k,k)].i);
                    state <= S_DIV_REAL_PREP2;
                end

                S_DIV_REAL_PREP2: begin
                    //real_num <= ((num_r * den_r) >>> FB) + ((num_i * den_i) >>> FB);
                    //imag_num <= ((num_i * den_r) >>> FB) - ((num_r * den_i) >>> FB);
                    //state <= S_DIV_REAL_START;
                    real_num_comp1 <= (num_r * den_r) >>> FB;
                    real_num_comp2 <= (num_i * den_i) >>> FB;
                    mag_comp1 <= (den_r * den_r) >>> FB;
                    mag_comp2 <= (den_i * den_i) >>> FB;
                    
                    state <= S_DIV_REAL_PREP3;
                end
                
                S_DIV_REAL_PREP3: begin
                    real_num <= real_num_comp1 + real_num_comp2;
                    mag <= mag_comp1 + mag_comp2;
                    state <= S_DIV_REAL_START;
                end

                S_DIV_REAL_START: begin
                    divisor_tvalid  <= 1'b1;
                    dividend_tvalid <= 1'b1;
                    divisor_tdata   <= mag[31:0];
                    dividend_tdata  <= $signed(real_num <<< FB);
                    state <= S_DIV_IMAG_PREP1;
                end
                
                S_DIV_IMAG_PREP1: begin
                    imag_num_comp1 <= (num_i * den_r) >>> FB;
                    imag_num_comp2 <= (num_r * den_i) >>> FB;
                    state <= S_DIV_IMAG_PREP2;
                end
                
                S_DIV_IMAG_PREP2: begin
                    imag_num <= imag_num_comp1 - imag_num_comp2;
                    state <= S_DIV_IMAG_START;
                end

                S_DIV_REAL_WAIT: begin
                    if (divider_tvalid) begin
                        factor.r <= divider_tdata;
                        state <= S_DIV_IMAG_WAIT;
                    end
                end

                S_DIV_IMAG_START: begin
                    divisor_tvalid  <= 1'b1;
                    dividend_tvalid <= 1'b1;
                    divisor_tdata   <= mag[31:0];
                    dividend_tdata  <= $signed(imag_num <<< FB);
                    state <= S_DIV_REAL_WAIT;
                end

                S_DIV_IMAG_WAIT: begin
                    if (divider_tvalid) begin
                        factor.i <= divider_tdata;
                        L[IDX(i,k)].r <= factor.r;
                        L[IDX(i,k)].i <= divider_tdata;
                        j <= k;
                        state <= S_COL_PREP;
                    end
                end


                S_COL_PREP: begin
                    if (j >= N)
                        state <= S_NEXT_ROW;
                    else
                        state <= S_MULT_START;
                end


                S_MULT_START: begin
                    mult_a_tvalid <= 1'b1;
                    mult_b_tvalid <= 1'b1;
                    mult_a_tdata  <= {$signed(factor.i),      $signed(factor.r)};
                    mult_b_tdata  <= {$signed(U[IDX(k,j)].i), $signed(U[IDX(k,j)].r)};
                    state <= S_MULT_WAIT;
                end


                S_MULT_WAIT: begin
                    if (mult_result_tvalid) begin
                        U[IDX(i,j)].r <= $signed(U[IDX(i,j)].r) - $signed(q32_32_to_q16_16_trunc($signed(mult_result_tdata[64:0])));
                        U[IDX(i,j)].i <= $signed(U[IDX(i,j)].i) - $signed(q32_32_to_q16_16_trunc($signed(mult_result_tdata[129:65])));
                        j <= j + 1;
                        state <= S_COL_PREP;
                    end
                end


                S_NEXT_ROW: begin
                    i <= i + 1;
                    state <= S_ROW_PREP;
                end


                S_NEXT_K: begin
                    if (k == N-1)
                        state <= S_DONE;
                    else begin
                        k <= k + 1;
                        state <= S_K_START;
                    end
                end


                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end


                default: state <= S_IDLE;
            endcase
        end
    end


endmodule
