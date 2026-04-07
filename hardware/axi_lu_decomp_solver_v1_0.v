`timescale 1 ns / 1 ps


module axi_lu_decomp_solver_v1_0 #
(
  parameter integer C_S00_AXI_DATA_WIDTH = 32,
  parameter integer C_S00_AXI_ADDR_WIDTH = 4
)
(
  // ---- BRAM Port B (native) ----
  output wire [31:0] bram_addr,
  output wire        bram_en,
  output wire [3:0]  bram_we,
  output wire [31:0] bram_wrdata,
  input  wire [31:0] bram_rddata,
  input  wire        bram_clk,
  output wire        bram_rst,


  // ---- AXI4-Lite Slave S00_AXI ----
  input  wire  s00_axi_aclk,
  input  wire  s00_axi_aresetn,
  input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
  input  wire [2:0] s00_axi_awprot,
  input  wire  s00_axi_awvalid,
  output wire  s00_axi_awready,
  input  wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_wdata,
  input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
  input  wire  s00_axi_wvalid,
  output wire  s00_axi_wready,
  output wire [1:0] s00_axi_bresp,
  output wire  s00_axi_bvalid,
  input  wire  s00_axi_bready,
  input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
  input  wire [2:0] s00_axi_arprot,
  input  wire  s00_axi_arvalid,
  output wire  s00_axi_arready,
  output wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_rdata,
  output wire [1:0] s00_axi_rresp,
  output wire  s00_axi_rvalid,
  input  wire  s00_axi_rready
);


  localparam [31:0] BRAM_AXI_BASE_ADDR = 32'hC0000000;


  // AXI regs exported from S00_AXI
  wire [31:0] reg0, reg1, reg2, reg3;
  wire [31:0] status_in;


  // Software passes AXI-visible BRAM addresses.
  // Native BRAM port needs local offsets, so subtract the AXI BRAM base.
  wire [31:0] A_BASE_W = reg1 - BRAM_AXI_BASE_ADDR;
  wire [31:0] B_BASE_W = reg2 - BRAM_AXI_BASE_ADDR;
  wire [31:0] X_BASE_W = reg3 - BRAM_AXI_BASE_ADDR;


  // Status flags
  reg done_flag, busy_flag, fail_flag, reuse_flag;
  assign status_in = {27'd0, reuse_flag, fail_flag, busy_flag, done_flag, 1'b0};


  // BRAM controls (registered)
  reg        bram_en_r;
  reg [3:0]  bram_we_r;
  reg [31:0] bram_addr_r;
  reg [31:0] bram_wrdata_r;


  assign bram_rst    = 1'b0;
  assign bram_en     = bram_en_r;
  assign bram_we     = bram_we_r;
  assign bram_addr   = bram_addr_r;
  assign bram_wrdata = bram_wrdata_r;


  // Current design/debug indicates this BRAM port uses local byte offsets.
  function [31:0] waddr_to_baddr(input [31:0] waddr);
    begin
      waddr_to_baddr = waddr;
    end
  endfunction


  // Core interface
  reg        core_start;
  reg        reuse_start;
  wire       core_done, core_fail;


  reg        a_we;
  reg [4:0]  a_addr;
  reg signed [31:0] a_wdata_real;
  reg signed [31:0] a_wdata_imag;


  reg  signed [32*5-1:0] b_flat_real_in;
  reg  signed [32*5-1:0] b_flat_imag_in;
  wire signed [32*5-1:0] x_flat_real_out;
  wire signed [32*5-1:0] x_flat_imag_out;


  // Temporary holding registers for A pair assembly
  reg signed [31:0] pair_real_buf;


  // FSM
  localparam IDLE    = 3'd0,
             READ_A  = 3'd1,
             PREP_B  = 3'd2,
             READ_B  = 3'd3,
             START_S = 3'd4,
             WAIT_S  = 3'd5,
             WRITE_X = 3'd6,
             DONE_S  = 3'd7;


  reg [2:0] state;
  reg       rd_pending;
  reg [5:0] idx;        // word index within each BRAM region
  reg [1:0] wait_count;
  (* max_fanout = 100 *) reg rstn_local;


  // start edge detect (start = reg0[0])
  reg start_d;
  wire start_pulse = reg0[0] & ~start_d;


  axi_lu_decomp_solver_v1_0_S00_AXI #(
    .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
    .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
  ) LU_v2_0_S00_AXI_inst (
    .S_AXI_ACLK   (s00_axi_aclk),
    .S_AXI_ARESETN(s00_axi_aresetn),
    .S_AXI_AWADDR (s00_axi_awaddr),
    .S_AXI_AWPROT (s00_axi_awprot),
    .S_AXI_AWVALID(s00_axi_awvalid),
    .S_AXI_AWREADY(s00_axi_awready),
    .S_AXI_WDATA  (s00_axi_wdata),
    .S_AXI_WSTRB  (s00_axi_wstrb),
    .S_AXI_WVALID (s00_axi_wvalid),
    .S_AXI_WREADY (s00_axi_wready),
    .S_AXI_BRESP  (s00_axi_bresp),
    .S_AXI_BVALID (s00_axi_bvalid),
    .S_AXI_BREADY (s00_axi_bready),
    .S_AXI_ARADDR (s00_axi_araddr),
    .S_AXI_ARPROT (s00_axi_arprot),
    .S_AXI_ARVALID(s00_axi_arvalid),
    .S_AXI_ARREADY(s00_axi_arready),
    .S_AXI_RDATA  (s00_axi_rdata),
    .S_AXI_RRESP  (s00_axi_rresp),
    .S_AXI_RVALID (s00_axi_rvalid),
    .S_AXI_RREADY (s00_axi_rready),
    .usr_reg0(reg0),
    .usr_reg1(reg1),
    .usr_reg2(reg2),
    .usr_reg3(reg3),
    .status_in(status_in)
  );


  LU_system_top u_core (
    .clk          (bram_clk),
    .rst_n        (s00_axi_aresetn),
    .start        (core_start),
    .reuse        (reuse_start),
    .a_we         (a_we),
    .a_addr       (a_addr),
    .a_wdata_real (a_wdata_real),
    .a_wdata_imag (a_wdata_imag),
    .b_flat_real_in(b_flat_real_in),
    .b_flat_imag_in(b_flat_imag_in),
    .x_flat_real_out(x_flat_real_out),
    .x_flat_imag_out(x_flat_imag_out),
    .done         (core_done),
    .fail         (core_fail)
  );
  
  always @(posedge bram_clk) begin
    rstn_local <= s00_axi_aresetn;
  end


  always @(posedge bram_clk) begin
    if (~rstn_local) begin
      state          <= IDLE;
      start_d        <= 1'b0;


      done_flag      <= 1'b0;
      busy_flag      <= 1'b0;
      fail_flag      <= 1'b0;
      reuse_flag     <= 1'b0;


      bram_en_r      <= 1'b0;
      bram_we_r      <= 4'b0000;
      bram_addr_r    <= 32'd0;
      bram_wrdata_r  <= 32'd0;


      core_start     <= 1'b0;
      reuse_start    <= 1'b0;
      a_we           <= 1'b0;
      a_addr         <= 5'd0;
      a_wdata_real   <= 32'sd0;
      a_wdata_imag   <= 32'sd0;
      pair_real_buf  <= 32'sd0;


      rd_pending     <= 1'b0;
      idx            <= 6'd0;
      wait_count     <= 2'd0;
    end else begin
      // defaults each cycle
      start_d     <= reg0[0];
      core_start  <= 1'b0;
      reuse_start <= 1'b0;
      a_we        <= 1'b0;
      bram_en_r   <= 1'b0;
      bram_we_r   <= 4'b0000;


      case (state)
        IDLE: begin
          done_flag  <= 1'b0;
          fail_flag  <= 1'b0;
          busy_flag  <= 1'b0;
          rd_pending <= 1'b0;
          idx        <= 6'd0;
          wait_count <= 2'd0;


          if (start_pulse) begin
            busy_flag      <= 1'b1;
            a_addr         <= 5'd0;
            a_wdata_real   <= 32'sd0;
            a_wdata_imag   <= 32'sd0;
            b_flat_real_in <= {32*5{1'b0}};
            b_flat_imag_in <= {32*5{1'b0}};
            pair_real_buf  <= 32'sd0;
            if(reg0[4] == 1'b0) begin
                state          <= READ_A;
            end else begin
                state          <= READ_B;
                reuse_flag     <= 1'b1;
            end
          end
        end


        // Read 50 words: real0 imag0 real1 imag1 ... real24 imag24
        READ_A: begin
          if (!rd_pending) begin
            bram_en_r   <= 1'b1;
            bram_addr_r <= waddr_to_baddr(A_BASE_W + (idx << 2));
            rd_pending  <= 1'b1;
            wait_count  <= 2'd2;
          end else if (wait_count != 2'd0) begin
            bram_en_r   <= 1'b1;
            wait_count  <= wait_count - 2'd1;
          end else begin
            rd_pending <= 1'b0;
            bram_en_r  <= 1'b1;


            if (!idx[0]) begin
              pair_real_buf <= bram_rddata;
            end else begin
              a_addr       <= idx[5:1];
              a_wdata_real <= pair_real_buf;
              a_wdata_imag <= bram_rddata;
              a_we         <= 1'b1;
            end


            if (idx == 6'd49) begin
              idx        <= 6'd0;
              rd_pending <= 1'b0;
              wait_count <= 2'd0;
              bram_en_r   <= 1'b0;
              state      <= PREP_B;
            end else begin
              idx <= idx + 6'd1;
            end
          end
        end






        // One clean cycle between A and B so stale BRAM data from the
        // final A read cannot be captured as b[0].real.
        PREP_B: begin
          rd_pending <= 1'b0;
          wait_count <= 2'd0;
          bram_en_r   <= 1'b0;
          state      <= READ_B;
        end


        // Read 10 words: real0 imag0 real1 imag1 ... real4 imag4
        READ_B: begin
          if (!rd_pending) begin
            bram_en_r   <= 1'b1;
            bram_addr_r <= waddr_to_baddr(B_BASE_W + (idx << 2));
            rd_pending  <= 1'b1;
            wait_count  <= 2'd2;
          end else if (wait_count != 2'd0) begin
            bram_en_r   <= 1'b1;
            wait_count  <= wait_count - 2'd1;
          end else begin
            rd_pending <= 1'b0;
            bram_en_r  <= 1'b1;


            if (!idx[0]) begin
              b_flat_real_in[32*idx[5:1] +: 32] <= bram_rddata;
            end else begin
              b_flat_imag_in[32*idx[5:1] +: 32] <= bram_rddata;
            end


            if (idx == 6'd9) begin
              idx   <= 6'd0;
              state <= START_S;
            end else begin
              idx <= idx + 6'd1;
            end
          end
        end


        START_S: begin
          if(reuse_flag) begin
            core_start <= 1'b0;
            reuse_start <= 1'b1;
          end else begin
            core_start <= 1'b1;
            reuse_start <= 1'b0;
          end
          state      <= WAIT_S;
        end


        WAIT_S: begin
          if (core_fail) begin
            fail_flag <= 1'b1;
            done_flag <= 1'b1;
            busy_flag <= 1'b0;
            state     <= DONE_S;
          end else if (core_done) begin
            idx   <= 6'd0;
            state <= WRITE_X;
          end
        end


        // Write 10 words interleaved: real0 imag0 ... real4 imag4
        WRITE_X: begin
          bram_en_r   <= 1'b1;
          bram_we_r   <= 4'b1111;
          bram_addr_r <= waddr_to_baddr(X_BASE_W + (idx << 2));
          if (!idx[0])
            bram_wrdata_r <= x_flat_real_out[32*idx[5:1] +: 32];
          else
            bram_wrdata_r <= x_flat_imag_out[32*idx[5:1] +: 32];


          if (idx == 6'd9) begin
            done_flag <= 1'b1;
            busy_flag <= 1'b0;
            reuse_flag <= 1'b0;
            state     <= DONE_S;
          end else begin
            idx <= idx + 6'd1;
          end
        end


        DONE_S: begin
          if (start_pulse) begin
            done_flag  <= 1'b0;
            fail_flag  <= 1'b0;
            busy_flag  <= 1'b1;
            //state      <= READ_A;
            idx        <= 6'd0;
            rd_pending <= 1'b0;
            wait_count <= 2'd0;
            b_flat_real_in <= {32*5{1'b0}};
            b_flat_imag_in <= {32*5{1'b0}};
            if(reg0[4] == 1'b0) begin
                state          <= READ_A;
            end else begin
                state          <= READ_B;
                reuse_flag     <= 1'b1;
            end
          end
        end


        default: state <= IDLE;
      endcase
    end
  end


endmodule

