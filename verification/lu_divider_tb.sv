`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/27/2026 12:40:37 AM
// Design Name: 
// Module Name: lu_divider_tb
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
`timescale 1ns / 1ps

//import complex_pkg::*;

import axi_vip_pkg::*;
import design_1_axi_vip_0_0_pkg::*;

reg aclk;
reg aresetn;

typedef struct packed {
    logic signed [31:0] r;
    logic signed [31:0] i;
} complex_t;

//Testing level registers
reg [31:0] start;
reg [31:0] a_addr;
reg [31:0] b_addr;
reg [31:0] x_addr;
logic [31:0] status;

real A_matrix[5][5];
real b_vector[5];
real x_to_check[5];

complex_t A[5][5];
complex_t b[5];
complex_t x[5];

integer idx;

logic signed [31:0] tmp32;
localparam N = 5;

integer c_i;
integer c_j;
integer c_k;
integer val;

//Constants
//AXI GPIO base address and register offsets
localparam BRAM_BASE_ADDRESS = 'hC0000000;
localparam LU_BASE_ADDRESS = 'h44A00000;
localparam LU_READY_OFFSET = 'h0;
localparam LU_A_MATRIX_OFFSET = 'h4;
localparam LU_B_VECTOR_OFFSET = 'h8;
localparam LU_X_VECTOR_OFFSET = 'hC;
localparam NUM_ITERATIONS = 12;

// test module to drive the AXI VIP
module axi_lite_stimulus();

	//Your Code Here
    design_1_axi_vip_0_0_mst_t     agent;
	
     /*************************************************************************************************
     * Declare variables which will be used in API and parital randomization for transaction generation
     * and data read back from driver.
     *************************************************************************************************/
     axi_transaction                                          wr_trans;            // Write transaction
     axi_transaction                                          rd_trans;            // Read transaction
     xil_axi_uint                                             mtestWID;            // Write ID  
     xil_axi_ulong                                            mtestWADDR;          // Write ADDR  
     xil_axi_len_t                                            mtestWBurstLength;   // Write Burst Length   
     xil_axi_size_t                                           mtestWDataSize;      // Write SIZE  
     xil_axi_burst_t                                          mtestWBurstType;     // Write Burst Type  
     xil_axi_lock_t                                           mtestWLock;          // Write Lock Type  
     xil_axi_cache_t                                          mtestWCache;          // Write Cache Type  
     xil_axi_prot_t                                           mtestWProt;          // Write Prot Type  
     xil_axi_region_t                                         mtestWRegion;        // Write Region Type  
     xil_axi_qos_t                                            mtestWQos;           // Write Qos Type  
     xil_axi_uint                                             mtestRID;            // Read ID  
     xil_axi_ulong                                            mtestRADDR;          // Read ADDR  
     xil_axi_len_t                                            mtestRBurstLength;   // Read Burst Length   
     xil_axi_size_t                                           mtestRDataSize;      // Read SIZE  
     xil_axi_burst_t                                          mtestRBurstType;     // Read Burst Type  
     xil_axi_lock_t                                           mtestRLock;          // Read Lock Type  
     xil_axi_cache_t                                          mtestRCache;         // Read Cache Type  
     xil_axi_prot_t                                           mtestRProt;          // Read Prot Type  
     xil_axi_region_t                                         mtestRRegion;        // Read Region Type  
     xil_axi_qos_t                                            mtestRQos;           // Read Qos Type  

     xil_axi_data_beat                                        Rdatabeat[];       // Read data beats

    
    initial begin
        /***********************************************************************************************
        * Before agent is newed, user has to run simulation with an empty testbench to find the hierarchy
        * path of the AXI VIP's instance.Message like
        * "Xilinx AXI VIP Found at Path: my_ip_exdes_tb.DUT.ex_design.axi_vip_mst.inst" will be printed 
        * out. Pass this path to the new function. 
        ***********************************************************************************************/
        agent = new("master vip agent",DUT.design_1_i.axi_vip_0.inst.IF); //TODO: UPDATE
        agent.start_master();               // agent start to run
    
        wait(aresetn == 1'b1);
        repeat (5) @(posedge aclk);
        
        start = 'd1;
        a_addr = BRAM_BASE_ADDRESS;
        b_addr = BRAM_BASE_ADDRESS + 'hC8;
        x_addr = BRAM_BASE_ADDRESS + 'hF0;
        
        
        /*
        for(c_k = 0; c_k < NUM_ITERATIONS; c_k++) begin
            $display("Iteration %0d Starting:", c_k);
            
            if(c_k == 0) begin
                assert_inputs();
                drive_lu_ip_start(start, a_addr, b_addr, x_addr);
            end else if(c_k <= 8) begin
                assert_only_b();
                drive_lu_ip_start('d17, a_addr, b_addr, x_addr);
            end else begin
                assert_inputs();
                drive_lu_ip_start(start, a_addr, b_addr, x_addr);
            end
            @(posedge aclk);
            writeRegister(LU_BASE_ADDRESS + LU_READY_OFFSET, 'd0);   
            agent.wait_drivers_idle();   
            
            while (1'b1) begin
                readRegister(LU_BASE_ADDRESS + LU_READY_OFFSET, Rdatabeat);
                agent.wait_drivers_idle();
                
                status = Rdatabeat[0][31:0];
                //$display("status seen: %d", status);
                if(status == 'b10) begin
                    //can read X now
                    for(c_i=0; c_i<10; c_i++) begin
                        readRegister(x_addr+(c_i*'d4), Rdatabeat);
                        agent.wait_drivers_idle();
                        tmp32 = Rdatabeat[0][31:0];
                        if (c_i % 2 == 0) begin
                            x[c_i/2].r = $signed(32'(Rdatabeat[0]));
                        end else begin
                            x[c_i/2].i = $signed(32'(Rdatabeat[0]));
                        end
                        //x_to_check[i] = $signed(32'(Rdatabeat[0])) / 65536.0;
                    end                
                    break;
                end else begin
                    repeat (60) @(posedge aclk);
                end
            end
            */
            
            /*
            validate_LU_decomp(
                .L(lu_divider_tb.DUT.design_1_i.axi_lu_decomp_solver_0.inst.u_core.u_decomp.L),
                .U(lu_divider_tb.DUT.design_1_i.axi_lu_decomp_solver_0.inst.u_core.u_decomp.U)
            );
            */
            
            @(posedge aclk);
            validate_output_complex(A, b, x);
        end
        $finish;

    end
    
task set_A_idx(
    input int x,
    input int y,
    input real value_real,
    input real value_imag
);
    begin
        A[x][y].r = value_real;
        A[x][y].i = value_imag;
    end
endtask;

task randomize_A_matrix(); 
    int i, j;
    begin
        for(i = 0; i < 5; i++) begin
            for (j = 0; j < 5; j++) begin
                A[i][j].r = $urandom_range(3276800, -3276800);
                A[i][j].i = $urandom_range(3276800, -3276800);
                //A[i][j].i = 0;

            end
        end
    end
endtask

task randomize_b_vector();
    int i, j;
    begin
        for(i = 0; i < 5; i++) begin
            b[i].r = $urandom_range(1310720, -1310720);
            b[i].i = $urandom_range(1310720, -1310720); //normally will be 0...

        end
    end
endtask

task assert_inputs(); 
    begin
        int i, j;
        real ar, ai, br, bi;

        randomize_A_matrix();
        randomize_b_vector();
        idx = 0;
        // Assuming a_addr is the base address of your matrix in memory
        for (i = 0; i < 5; i = i + 1) begin
            for (j = 0; j < 5; j = j + 1) begin
                writeRegister(a_addr + (idx * 'd4), A[i][j].r);
                idx += 1;
                writeRegister(a_addr + (idx * 'd4), A[i][j].i);
                idx += 1;
                ar = real'(A[i][j].r) / 65536.0;
                ai = real'(A[i][j].i) / 65536.0;
                $display("A[%0d][%0d].r = %f, A[%0d][%0d].i = %f", i, j, ar, i, j, ai);
            end
        end  
        
        idx = 0;
        for (i = 0; i < 5; i += 1) begin
            writeRegister(b_addr+(idx*'d4), b[i].r);
            idx += 1;
            writeRegister(b_addr+(idx*'d4), b[i].i);
            idx += 1;
            br = real'(b[i].r) / 65536.0;
            bi = real'(b[i].i) / 65536.0;
            //$display("b[%0d].r = %0d, b[%0d].i = %0d", i, br, i, bi);
        end
        agent.wait_drivers_idle();
    end
endtask

task assert_only_b();
    begin
        int i, j;
        real ar, ai, br, bi;

        randomize_b_vector();
                
        idx = 0;
        for (i = 0; i < 5; i += 1) begin
            writeRegister(b_addr+(idx*'d4), b[i].r);
            idx += 1;
            writeRegister(b_addr+(idx*'d4), b[i].i);
            idx += 1;
            br = real'(b[i].r) / 65536.0;
            bi = real'(b[i].i) / 65536.0;
            $display("b[%0d].r = %0d, b[%0d].i = %0d", i, br, i, bi);
        end
        agent.wait_drivers_idle();
    end 
endtask
    
task drive_lu_ip_start(
        input [31:0] start,
        input [31:0] a_addr,
        input [31:0] b_addr,
        input [31:0] x_addr
    );
    begin
        //Write to all slave registers with desired values
        writeRegister(LU_BASE_ADDRESS + LU_A_MATRIX_OFFSET, a_addr);
        writeRegister(LU_BASE_ADDRESS + LU_B_VECTOR_OFFSET, b_addr);
        writeRegister(LU_BASE_ADDRESS + LU_X_VECTOR_OFFSET, x_addr);
      
        //Wait for all writes to complete
        agent.wait_drivers_idle();   
        writeRegister(LU_BASE_ADDRESS + LU_READY_OFFSET, start);   
        agent.wait_drivers_idle();   
    end
endtask

task validate_LU_decomp(
    input complex_t L[25],   // Flattened internal Q16.16 L
    input complex_t U[25]    // Flattened internal Q16.16 U
);
    integer i, j, k;
    real sum_r, sum_i, diff_r, diff_i;
    real lr, li, ur, ui, ar, ai;
    real threshold;
    real Q_CONV;
    logic error_found;
    
    threshold = 0.05;
    error_found = 1'b0;

    // Q16.16 conversion factor (2^16)
    Q_CONV = 65536.0;

    for (i = 0; i < 5; i++) begin
        for (j = 0; j < 5; j++) begin
            sum_r = 0;
            sum_i = 0;
            
            lr = real'(L[i*5 + j].r) / Q_CONV;
            ur = real'(U[i*5 + j].r) / Q_CONV;
 
            //$display("L[%0d][%0d].r = %f, U[%0d][%0d].r = %f", i, j, lr, i, j, ur);

            
            for (k = 0; k < 5; k++) begin
                // 1. Convert L and U elements from Q16.16 to Real
                // We access the 1D array using (row * width + col)
                lr = real'(L[i*5 + k].r) / Q_CONV;
                li = real'(L[i*5 + k].i) / Q_CONV;
                ur = real'(U[k*5 + j].r) / Q_CONV;
                ui = real'(U[k*5 + j].i) / Q_CONV;
                

                // 2. Perform Complex Multiplication (Real Math)
                sum_r += (lr * ur) - (li * ui);
                sum_i += (lr * ui) + (li * ur); 
            end
            
            ar = real'(A[i][j].r) / Q_CONV;
            ai = real'(A[i][j].i) / Q_CONV;
            // 3. Compare against original A (Assuming A is already real/float)
            diff_r = sum_r - ar;
            diff_i = sum_i - ai;
            
            if (abs_real(diff_r) > threshold || abs_real(diff_i) > threshold) begin
                $display("[LU ERROR] At [%0d][%0d]: Expected A=(%f+j%f), Got LU=(%f+j%f)", 
                          i, j, ar, ai, sum_r, sum_i);
                error_found = 1'b1;
            end
        end
    end

    if (!error_found) begin
        $display("SUCCESS - LU decomposition (A=LU) check passed!");
    end
    else begin
        $display("FAILURE - LU decomposition mismatch detected.");
    end
endtask

// Simple absolute value function for 'real' types
function real abs_real(input real val);
    return (val < 0) ? -val : val;
endfunction
    
task validate_output_complex(
    input complex_t A[5][5],
    input complex_t b[5],
    input complex_t x[5]
);

    integer i,j;
    logic valid = 1'b1;

    real sum_r;
    real sum_i;

    real ar, ai;
    real xr, xi;

    real br, bi;

    real diff_r;
    real diff_i;

    real threshold = 0.01;

begin

    for (i = 0; i < 5; i++) begin

        sum_r = 0;
        sum_i = 0;
        
        //$display("x[%0d].r = %0d, x[%0d].i = %0d", i, x[i].r, i,  x[i].i);

        for (j = 0; j < 5; j++) begin

            ar = real'(A[i][j].r) / 65536.0;
            ai = real'(A[i][j].i) / 65536.0;

            xr = real'(x[j].r) / 65536.0;
            xi = real'(x[j].i) / 65536.0;

            // complex multiply accumulate
            sum_r += (ar * xr) - (ai * xi);
            sum_i += (ar * xi) + (ai * xr);

        end

        br = real'(b[i].r) / 65536.0;
        bi = real'(b[i].i) / 65536.0;

        diff_r = sum_r - br;
        diff_i = sum_i - bi;

        if ( (diff_r > threshold) || (diff_r < -threshold) ||
             (diff_i > threshold) || (diff_i < -threshold) ) begin

            valid = 1'b0;

            $display("[ERROR] Index %0d:", i);
            $display("Expected b = (%f + j%f)", br, bi);
            $display("Got Ax    = (%f + j%f)", sum_r, sum_i);
            $display("Error     = (%f + j%f)", diff_r, diff_i);

        end
        else begin
            //$display("[OK] Index %0d:", i);
            //$display("Expected b = (%f + j%f)", br, bi);
            //$display("Got Ax    = (%f + j%f)", sum_r, sum_i);
        end
    end

    if (!valid)
        $display("Output did not match");
    else
        $display("All checks passed - complex output is correct!");

end
endtask

task writeRegister( input xil_axi_ulong              addr =0,
                    input bit [31:0]              data =0
                );

    single_write_transaction_api("single write with api",
                                 .addr(addr),
                                 .size(xil_axi_size_t'(2)),
                                 .data(data)
                                 );
endtask : writeRegister

  /************************************************************************************************
  *  task single_write_transaction_api is to create a single write transaction, fill in transaction 
  *  by using APIs and send it to write driver.
  *   1. declare write transction
  *   2. Create the write transaction
  *   3. set addr, burst,ID,length,size by calling set_write_cmd(addr, burst,ID,length,size), 
  *   4. set prot.lock, cache,region and qos
  *   5. set beats
  *   6. set AWUSER if AWUSER_WIDH is bigger than 0
  *   7. set WUSER if WUSR_WIDTH is bigger than 0
  *************************************************************************************************/

  task automatic single_write_transaction_api ( 
                                input string                     name ="single_write",
                                input xil_axi_uint               id =0, 
                                input xil_axi_ulong              addr =0,
                                input xil_axi_len_t              len =0, 
                                input xil_axi_size_t             size =xil_axi_size_t'(xil_clog2((32)/8)),
                                input xil_axi_burst_t            burst =XIL_AXI_BURST_TYPE_INCR,
                                input xil_axi_lock_t             lock = XIL_AXI_ALOCK_NOLOCK,
                                input xil_axi_cache_t            cache =3,
                                input xil_axi_prot_t             prot =0,
                                input xil_axi_region_t           region =0,
                                input xil_axi_qos_t              qos =0,
                                input xil_axi_data_beat [255:0]  wuser =0, 
                                input xil_axi_data_beat          awuser =0,
                                input bit [32767:0]              data =0
                                                );
    axi_transaction                               wr_trans;
    wr_trans = agent.wr_driver.create_transaction(name);
    wr_trans.set_write_cmd(addr,burst,id,len,size);
    wr_trans.set_prot(prot);
    wr_trans.set_lock(lock);
    wr_trans.set_cache(cache);
    wr_trans.set_region(region);
    wr_trans.set_qos(qos);
    wr_trans.set_awuser(awuser);
    wr_trans.set_data_block(data);
    agent.wr_driver.send(wr_trans);   
  endtask  : single_write_transaction_api
  
  //task automatic readRegister (  input xil_axi_ulong addr =0 );
                                                
    //single_read_transaction_api(.addr(addr));

  //endtask  : readRegister
  task automatic readRegister ( 
                                    input xil_axi_ulong              addr =0,
                                    output xil_axi_data_beat Rdatabeat[]
                                                );
    axi_transaction                               rd_trans;
    xil_axi_uint               id =0; 
    xil_axi_len_t              len =0; 
    xil_axi_size_t             size =xil_axi_size_t'(xil_clog2((32)/8));
    xil_axi_burst_t            burst =XIL_AXI_BURST_TYPE_INCR;
    xil_axi_lock_t             lock =XIL_AXI_ALOCK_NOLOCK ;
    xil_axi_cache_t            cache =3;
    xil_axi_prot_t             prot =0;
    xil_axi_region_t           region =0;
    xil_axi_qos_t              qos =0;
    xil_axi_data_beat          aruser =0;
    rd_trans = agent.rd_driver.create_transaction("single-read");
    rd_trans.set_read_cmd(addr,burst,id,len,size);
    rd_trans.set_prot(prot);
    rd_trans.set_lock(lock);
    rd_trans.set_cache(cache);
    rd_trans.set_region(region);
    rd_trans.set_qos(qos);
    rd_trans.set_aruser(aruser);
    get_rd_data_beat_back(rd_trans,Rdatabeat);
  endtask  : readRegister
  
  /************************************************************************************************
  * Task send_wait_rd is a task which set_driver_return_item_policy of the read transaction, 
  * send the transaction to the driver and wait till it is done
  *************************************************************************************************/
  task send_wait_rd(inout axi_transaction rd_trans);
    rd_trans.set_driver_return_item_policy(XIL_AXI_PAYLOAD_RETURN);
    agent.rd_driver.send(rd_trans);
    agent.rd_driver.wait_rsp(rd_trans);
  endtask

  /************************************************************************************************
  * Task get_rd_data_beat_back is to get read data back from read driver with
  *  data beat format.
  *************************************************************************************************/
  task get_rd_data_beat_back(inout axi_transaction rd_trans, 
                                 output xil_axi_data_beat Rdatabeat[]
                            );  
    send_wait_rd(rd_trans);
    Rdatabeat = new[rd_trans.get_len()+1];
    for( xil_axi_uint beat=0; beat<rd_trans.get_len()+1; beat++) begin
      Rdatabeat[beat] = rd_trans.get_data_beat(beat);
   //   $display("Read data from Driver: beat index %d, Data beat %h ", beat, Rdatabeat[beat]);
    end  
  endtask


  /************************************************************************************************
  *  task single_read_transaction_api is to create a single read transaction, fill in command with user
  *  inputs and send it to read driver.
  *   1. declare read transction
  *   2. Create the read transaction
  *   3. set addr, burst,ID,length,size by calling set_read_cmd(addr, burst,ID,length,size), 
  *   4. set prot.lock, cache,region and qos
  *   5. set ARUSER if ARUSER_WIDH is bigger than 0
  *************************************************************************************************/
  task automatic single_read_transaction_api ( 
                                    input string                     name ="single_read",
                                    input xil_axi_uint               id =0, 
                                    input xil_axi_ulong              addr =0,
                                    input xil_axi_len_t              len =0, 
                                    input xil_axi_size_t             size =xil_axi_size_t'(xil_clog2((32)/8)),
                                    input xil_axi_burst_t            burst =XIL_AXI_BURST_TYPE_INCR,
                                    input xil_axi_lock_t             lock =XIL_AXI_ALOCK_NOLOCK ,
                                    input xil_axi_cache_t            cache =3,
                                    input xil_axi_prot_t             prot =0,
                                    input xil_axi_region_t           region =0,
                                    input xil_axi_qos_t              qos =0,
                                    input xil_axi_data_beat          aruser =0
                                                );
    axi_transaction                               rd_trans;
    rd_trans = agent.rd_driver.create_transaction(name);
    rd_trans.set_read_cmd(addr,burst,id,len,size);
    rd_trans.set_prot(prot);
    rd_trans.set_lock(lock);
    rd_trans.set_cache(cache);
    rd_trans.set_region(region);
    rd_trans.set_qos(qos);
    rd_trans.set_aruser(aruser);
    agent.rd_driver.send(rd_trans);   
  endtask  : single_read_transaction_api
  
endmodule

// testbench entry point
module lu_divider_tb();

	// instantiate the "design under test" module
	design_1_wrapper DUT(
		.clk(aclk),
		.reset(aresetn)
		);

	// clock generator (100MHz)
	initial
	begin
		aclk = 0;
		forever
			#5ns aclk = ~aclk;
	end

	// start the testbench by resetting the system for 5 cycles
	initial
	begin
		apply_reset(5);
	end

	// instantiate instance of axi_lite_stimulus into the tb
	axi_lite_stimulus mst();
	
	task apply_reset(input integer cycles = 5);
	   aresetn = 0;
	   repeat (cycles) @(posedge aclk);
	   aresetn = 1;
	endtask : apply_reset

endmodule

