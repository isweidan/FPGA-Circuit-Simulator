`timescale 1ns/1ps
`default_nettype none

module tb_lu_axi_golden;

  // ============================================================
  // Parameters
  // ============================================================
  localparam int AXI_DW     = 32;
  localparam int AXI_AW     = 4;
  localparam int BRAM_WORDS = 2048;

  // AXI register map
  localparam logic [AXI_AW-1:0] REG0 = 4'h0; // start/status
  localparam logic [AXI_AW-1:0] REG1 = 4'h4; // A_BASE_W (word addr)
  localparam logic [AXI_AW-1:0] REG2 = 4'h8; // B_BASE_W (word addr)
  localparam logic [AXI_AW-1:0] REG3 = 4'hC; // X_BASE_W (word addr)

  // ============================================================
  // Clock / Reset
  // ============================================================
  logic clk;
  logic aresetn;

  // ============================================================
  // AXI4-Lite signals
  // ============================================================
  logic [AXI_AW-1:0] s_axi_awaddr;
  logic [2:0]        s_axi_awprot;
  logic              s_axi_awvalid;
  wire               s_axi_awready;

  logic [AXI_DW-1:0] s_axi_wdata;
  logic [(AXI_DW/8)-1:0] s_axi_wstrb;
  logic              s_axi_wvalid;
  wire               s_axi_wready;

  wire [1:0]         s_axi_bresp;
  wire               s_axi_bvalid;
  logic              s_axi_bready;

  logic [AXI_AW-1:0] s_axi_araddr;
  logic [2:0]        s_axi_arprot;
  logic              s_axi_arvalid;
  wire               s_axi_arready;

  wire [AXI_DW-1:0]  s_axi_rdata;
  wire [1:0]         s_axi_rresp;
  wire               s_axi_rvalid;
  logic              s_axi_rready;

  // ============================================================
  // BRAM native port
  // ============================================================
  wire [31:0] bram_addr;
  wire        bram_en;
  wire [3:0]  bram_we;
  wire [31:0] bram_wrdata;
  logic [31:0] bram_rddata;

  // These are INPUTS to DUT now, so TB must DRIVE them
  logic       bram_clk;
  logic       bram_rst;

  // ============================================================
  // DUT
  // ============================================================
  axi_lu_decomp_solver_v1_0 #(
    .C_S00_AXI_DATA_WIDTH(AXI_DW),
    .C_S00_AXI_ADDR_WIDTH(AXI_AW)
  ) dut (
    .bram_addr   (bram_addr),
    .bram_en     (bram_en),
    .bram_we     (bram_we),
    .bram_wrdata (bram_wrdata),
    .bram_rddata (bram_rddata),
    .bram_clk    (bram_clk),
    .bram_rst    (bram_rst),

    .s00_axi_aclk    (clk),
    .s00_axi_aresetn (aresetn),

    .s00_axi_awaddr  (s_axi_awaddr),
    .s00_axi_awprot  (s_axi_awprot),
    .s00_axi_awvalid (s_axi_awvalid),
    .s00_axi_awready (s_axi_awready),

    .s00_axi_wdata   (s_axi_wdata),
    .s00_axi_wstrb   (s_axi_wstrb),
    .s00_axi_wvalid  (s_axi_wvalid),
    .s00_axi_wready  (s_axi_wready),

    .s00_axi_bresp   (s_axi_bresp),
    .s00_axi_bvalid  (s_axi_bvalid),
    .s00_axi_bready  (s_axi_bready),

    .s00_axi_araddr  (s_axi_araddr),
    .s00_axi_arprot  (s_axi_arprot),
    .s00_axi_arvalid (s_axi_arvalid),
    .s00_axi_arready (s_axi_arready),

    .s00_axi_rdata   (s_axi_rdata),
    .s00_axi_rresp   (s_axi_rresp),
    .s00_axi_rvalid  (s_axi_rvalid),
    .s00_axi_rready  (s_axi_rready)
  );

  // ============================================================
  // Clock generation
  // ============================================================
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100MHz
  end

  // ============================================================
  // BRAM clock/reset drive (FIX #1)
  // In Vivado BD you will tie BRAM clock domain to system clock.
  // ============================================================
  always_comb begin
    bram_clk = clk;        // same domain as AXI
    bram_rst = ~aresetn;    // active-high reset
  end

  // ============================================================
  // BRAM model
  // IMPORTANT: for your wrapper (registered bram_addr/bram_en),
  //            the most robust model is:
  //   - READ: combinational from bram_addr (no bram_en gating)
  //   - WRITE: synchronous when bram_en && bram_we!=0
  // ============================================================
  logic signed [31:0] mem [0:BRAM_WORDS-1];

  // temps at module scope (XSim-friendly)
  int unsigned wa;
  int unsigned ra;
  logic [31:0] cur;

  function automatic int unsigned bramaddr_to_word(input logic [31:0] addr);
    begin
      // DUT drives WORD addresses directly
      return addr;
    end
  endfunction

  // READ (combinational)
  always_comb begin
    ra = bramaddr_to_word(bram_addr);
    bram_rddata = mem[ra];
  end

  // WRITE (sync)
  always_ff @(posedge bram_clk) begin
    if (!bram_rst) begin
      if (bram_en && (bram_we != 4'b0000)) begin
        wa  = bramaddr_to_word(bram_addr);
        cur = mem[wa];

        if (bram_we[0]) cur[7:0]   = bram_wrdata[7:0];
        if (bram_we[1]) cur[15:8]  = bram_wrdata[15:8];
        if (bram_we[2]) cur[23:16] = bram_wrdata[23:16];
        if (bram_we[3]) cur[31:24] = bram_wrdata[31:24];

        mem[wa] <= cur;
      end
    end
  end

  // ============================================================
  // Optional: BRAM transaction printing (set to 1 to enable)
  // ============================================================
  localparam bit TRACE_BRAM = 0;
  int rd_count;
  int wr_count;

  // FIX #2: remove (bram_addr>>2) since bram_addr is WORD address now
  always_ff @(posedge clk) begin
    if (!aresetn) begin
      rd_count <= 0;
      wr_count <= 0;
    end else if (TRACE_BRAM) begin
      if (bram_en && (bram_we == 4'b0000)) begin
        rd_count <= rd_count + 1;
        $display("[%0t] BRAM RD #%0d  waddr=%0d data=0x%08h",
                 $time, rd_count, bram_addr, bram_rddata);
      end
      if (bram_en && (bram_we != 4'b0000)) begin
        wr_count <= wr_count + 1;
        $display("[%0t] BRAM WR #%0d  waddr=%0d we=%b wdata=0x%08h",
                 $time, wr_count, bram_addr, bram_we, bram_wrdata);
      end
    end
  end

  // ============================================================
  // Fixed-point helper (Q16.16) - only convert Q->real for golden
  // ============================================================
  function automatic real q16_to_real(input logic signed [31:0] q);
    real rr;
    begin
      rr = real'(q) / 65536.0;
      return rr;
    end
  endfunction

  // ============================================================
  // AXI Master tasks
  // ============================================================
  task automatic axi_init;
    begin
      s_axi_awaddr  = '0;
      s_axi_awprot  = 3'b000;
      s_axi_awvalid = 1'b0;

      s_axi_wdata   = '0;
      s_axi_wstrb   = 4'hF;
      s_axi_wvalid  = 1'b0;

      s_axi_bready  = 1'b0;

      s_axi_araddr  = '0;
      s_axi_arprot  = 3'b000;
      s_axi_arvalid = 1'b0;

      s_axi_rready  = 1'b0;
    end
  endtask

  task automatic axi_write(input logic [AXI_AW-1:0] addr, input logic [31:0] data);
    begin
      @(posedge clk);
      s_axi_awaddr  <= addr;
      s_axi_awvalid <= 1'b1;
      s_axi_wdata   <= data;
      s_axi_wvalid  <= 1'b1;

      while (!(s_axi_awready && s_axi_wready)) @(posedge clk);

      s_axi_awvalid <= 1'b0;
      s_axi_wvalid  <= 1'b0;

      s_axi_bready  <= 1'b1;
      while (!s_axi_bvalid) @(posedge clk);
      s_axi_bready  <= 1'b0;

      if (s_axi_bresp !== 2'b00) $fatal(1, "AXI write BRESP != OKAY at 0x%0h", addr);
    end
  endtask

  task automatic axi_read(input logic [AXI_AW-1:0] addr, output logic [31:0] data);
    begin
      @(posedge clk);
      s_axi_araddr  <= addr;
      s_axi_arvalid <= 1'b1;

      while (!s_axi_arready) @(posedge clk);
      s_axi_arvalid <= 1'b0;

      s_axi_rready <= 1'b1;
      while (!s_axi_rvalid) @(posedge clk);
      data = s_axi_rdata;
      s_axi_rready <= 1'b0;

      if (s_axi_rresp !== 2'b00) $fatal(1, "AXI read RRESP != OKAY at 0x%0h", addr);
    end
  endtask

  // ============================================================
  // Golden solver (real Gaussian elimination w/ partial pivoting)
  // ============================================================
  task automatic solve5_real(
    input  real A_in [0:4][0:4],
    input  real b_in [0:4],
    output real x_out[0:4],
    output bit  singular
  );
    real A [0:4][0:4];
    real b [0:4];
    int i, j, k, piv;
    real maxv, tmp, f;
    real sum;

    begin
      singular = 0;

      for (i = 0; i < 5; i = i + 1) begin
        b[i] = b_in[i];
        for (j = 0; j < 5; j = j + 1) begin
          A[i][j] = A_in[i][j];
        end
      end

      for (k = 0; k < 5; k = k + 1) begin
        piv = k;
        maxv = (A[k][k] < 0.0) ? -A[k][k] : A[k][k];

        for (i = k + 1; i < 5; i = i + 1) begin
          tmp = (A[i][k] < 0.0) ? -A[i][k] : A[i][k];
          if (tmp > maxv) begin
            maxv = tmp;
            piv  = i;
          end
        end

        if (maxv < 1e-12) begin
          singular = 1;
          for (i = 0; i < 5; i = i + 1) x_out[i] = 0.0;
          return;
        end

        if (piv != k) begin
          for (j = k; j < 5; j = j + 1) begin
            tmp       = A[k][j];
            A[k][j]   = A[piv][j];
            A[piv][j] = tmp;
          end
          tmp    = b[k];
          b[k]   = b[piv];
          b[piv] = tmp;
        end

        for (i = k + 1; i < 5; i = i + 1) begin
          f       = A[i][k] / A[k][k];
          A[i][k] = 0.0;
          for (j = k + 1; j < 5; j = j + 1) begin
            A[i][j] = A[i][j] - f * A[k][j];
          end
          b[i] = b[i] - f * b[k];
        end
      end

      for (i = 4; i >= 0; i = i - 1) begin
        sum = b[i];
        for (j = i + 1; j < 5; j = j + 1) begin
          sum = sum - A[i][j] * x_out[j];
        end
        x_out[i] = sum / A[i][i];
      end
    end
  endtask

  // ============================================================
  // Load matrix/vector from Q16 arrays into BRAM
  // A_q16 is 25 entries row-major: A[r][c] at index r*5+c
  // b_q16 is 5 entries
  // ============================================================
  task automatic bram_load_from_q16(
    input int unsigned A_base_w,
    input int unsigned B_base_w,
    input logic signed [31:0] A_q16 [0:24],
    input logic signed [31:0] b_q16 [0:4]
  );
    int i;
    begin
      for (i = 0; i < 25; i = i + 1) mem[A_base_w + i] = A_q16[i];
      for (i = 0; i < 5;  i = i + 1) mem[B_base_w + i] = b_q16[i];
    end
  endtask

  // ============================================================
  // Build real A,b from Q16 arrays (for golden)
  // ============================================================
  task automatic q16_to_real_AB(
    input  logic signed [31:0] A_q16 [0:24],
    input  logic signed [31:0] b_q16 [0:4],
    output real A_r [0:4][0:4],
    output real b_r [0:4]
  );
    int r, c, idx;
    begin
      idx = 0;
      for (r = 0; r < 5; r = r + 1) begin
        for (c = 0; c < 5; c = c + 1) begin
          A_r[r][c] = q16_to_real(A_q16[idx]);
          idx = idx + 1;
        end
      end
      for (r = 0; r < 5; r = r + 1) begin
        b_r[r] = q16_to_real(b_q16[r]);
      end
    end
  endtask

  // ============================================================
  // Run one test: load, start, wait done, compare vs golden
  // ============================================================
  task automatic run_test_golden(
    input string name,
    input int unsigned A_base_w,
    input int unsigned B_base_w,
    input int unsigned X_base_w,
    input logic signed [31:0] A_q16 [0:24],
    input logic signed [31:0] b_q16 [0:4]
  );
    // decls at top
    logic [31:0] r0, r1, r2, r3;
    real A_r [0:4][0:4];
    real b_r [0:4];
    real x_g [0:4];
    bit singular;
    int i;
    logic signed [31:0] x_q;
    real x_dut;
    real diff;
    real tol;

    begin
      $display("\n=== TEST: %s ===", name);

      // tolerance: 3 LSBs in Q16.16 by default
      tol = 3.0 / 65536.0;

      // load BRAM
      bram_load_from_q16(A_base_w, B_base_w, A_q16, b_q16);

      // golden from same quantized inputs (avoid any format mismatch)
      q16_to_real_AB(A_q16, b_q16, A_r, b_r);
      solve5_real(A_r, b_r, x_g, singular);

      // program bases and readback
      axi_write(REG1, A_base_w);
      axi_write(REG2, B_base_w);
      axi_write(REG3, X_base_w);

      axi_read(REG1, r1);
      axi_read(REG2, r2);
      axi_read(REG3, r3);
      $display("Readback bases: A=%0d B=%0d X=%0d", r1, r2, r3);

      // start pulse
      axi_write(REG0, 32'h0);
      axi_write(REG0, 32'h1);

      // wait done/fail
      do begin
        axi_read(REG0, r0);
      end while ((r0[1] == 1'b0) && (r0[3] == 1'b0));

      $display("REG0=0x%08h done=%0d busy=%0d fail=%0d", r0, r0[1], r0[2], r0[3]);

      if (r0[3]) $fatal(1, "DUT FAIL in test '%s' (REG0=0x%08h)", name, r0);

      // Compare X
      if (singular) begin
        $display("Golden: matrix considered singular (skipping strict compare).");
      end else begin
        for (i = 0; i < 5; i = i + 1) begin
          x_q   = mem[X_base_w + i];
          x_dut = q16_to_real(x_q);
          diff  = x_dut - x_g[i];
          if (diff < 0.0) diff = -diff;

          $display("x[%0d] DUT=%f (0x%08h)  GOLD=%f  |diff|=%e",
                   i, x_dut, x_q, x_g[i], diff);

          if (diff > tol) begin
            $fatal(1, "Mismatch x[%0d] in '%s' diff=%e > tol=%e",
                   i, name, diff, tol);
          end
        end
      end

      $display("=== PASS: %s ===", name);
    end
  endtask

  // ============================================================
  // Test vectors in Q16.16 (avoid real->fixed conversion in XSim)
  // ============================================================
  logic signed [31:0] A1_q16 [0:24];
  logic signed [31:0] b1_q16 [0:4];

  logic signed [31:0] A2_q16 [0:24];
  logic signed [31:0] b2_q16 [0:4];

  task automatic init_vectors;
    int i;
    begin
      // clear
      for (i = 0; i < 25; i = i + 1) begin
        A1_q16[i] = 32'sd0;
        A2_q16[i] = 32'sd0;
      end
      for (i = 0; i < 5; i = i + 1) begin
        b1_q16[i] = 32'sd0;
        b2_q16[i] = 32'sd0;
      end

      // ----------------------------------------------------------
      // Test 1
      // ----------------------------------------------------------
      A1_q16[ 0]=32'sh00040000; A1_q16[ 1]=32'sh00010000; A1_q16[ 2]=32'sh00020000; A1_q16[ 3]=32'sh00000000; A1_q16[ 4]=32'sh00000000;
      A1_q16[ 5]=32'sh00010000; A1_q16[ 6]=32'sh00050000; A1_q16[ 7]=32'sh00000000; A1_q16[ 8]=32'sh00010000; A1_q16[ 9]=32'sh00000000;
      A1_q16[10]=32'sh00020000; A1_q16[11]=32'sh00000000; A1_q16[12]=32'sh00060000; A1_q16[13]=32'sh00000000; A1_q16[14]=32'sh00010000;
      A1_q16[15]=32'sh00000000; A1_q16[16]=32'sh00010000; A1_q16[17]=32'sh00000000; A1_q16[18]=32'sh00070000; A1_q16[19]=32'sh00020000;
      A1_q16[20]=32'sh00000000; A1_q16[21]=32'sh00000000; A1_q16[22]=32'sh00010000; A1_q16[23]=32'sh00020000; A1_q16[24]=32'sh00080000;

      b1_q16[0]=32'sh00010000; b1_q16[1]=32'sh00020000; b1_q16[2]=32'sh00030000; b1_q16[3]=32'sh00040000; b1_q16[4]=32'sh00050000;

      // ----------------------------------------------------------
      // Test 2
      // ----------------------------------------------------------
      A2_q16[ 0]=32'sh00000000; A2_q16[ 1]=32'sh00020000; A2_q16[ 2]=32'sh00030000; A2_q16[ 3]=32'sh00000000; A2_q16[ 4]=32'sh00000000;
      A2_q16[ 5]=32'sh00010000; A2_q16[ 6]=32'sh00010000; A2_q16[ 7]=32'sh00040000; A2_q16[ 8]=32'sh00000000; A2_q16[ 9]=32'sh00000000;
      A2_q16[10]=32'sh00020000; A2_q16[11]=32'sh00000000; A2_q16[12]=32'sh00010000; A2_q16[13]=32'sh00010000; A2_q16[14]=32'sh00000000;
      A2_q16[15]=32'sh00000000; A2_q16[16]=32'sh00000000; A2_q16[17]=32'sh00010000; A2_q16[18]=32'sh00030000; A2_q16[19]=32'sh00010000;
      A2_q16[20]=32'sh00000000; A2_q16[21]=32'sh00000000; A2_q16[22]=32'sh00000000; A2_q16[23]=32'sh00020000; A2_q16[24]=32'sh00020000;

      b2_q16[0]=32'sh00030000; b2_q16[1]=32'sh00020000; b2_q16[2]=32'sh00010000; b2_q16[3]=32'sh00040000; b2_q16[4]=32'sh00020000;
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    int i;

    $display(">>> RUNNING tb_lu_axi_golden <<<");

    axi_init();
    init_vectors();

    // clear BRAM
    for (i = 0; i < BRAM_WORDS; i = i + 1) mem[i] = '0;

    // reset
    aresetn = 1'b0;
    repeat (10) @(posedge clk);
    aresetn = 1'b1;
    repeat (5) @(posedge clk);

    // Run tests (use non-overlapping word regions)
    run_test_golden("Test1 basic",   32,  128, 256, A1_q16, b1_q16);
    run_test_golden("Test2 pivoting",400, 500, 600, A2_q16, b2_q16);

    $display("\nALL GOLDEN TESTS PASSED.\n");
    $finish;
  end

endmodule

`default_nettype wire