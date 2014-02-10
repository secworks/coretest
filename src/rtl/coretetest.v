p//======================================================================
//
// ctttest.v
// ---------
// Top level of the Cryptech test core. Simply a 32-bit interface
// with some intrernal functionality to see that we can read and write
// registers in the FPGA from the host. This core will be the basis
// for all core top levels.
//
// This should be a WISBONE interface.
//
// (c) 2014 JoachimS
//
//======================================================================

module cttest(
              input wire           clk,
              input wire           reset_n,

              input wire           cs,
              input wire           we,
              input wire [2 : 0]   address,
              input wire [31 : 0]  write_data,
              output wire [31 : 0] read_data,
              output wire [31 : 0] api_error
             );

  
  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter ADDR0 = 3'h0; // Version
  parameter ADDR1 = 3'h1; // Op A
  parameter ADDR2 = 3'h2; // Op B
  parameter ADDR3 = 3'h3; // Result

  // String: ctt1
  parameter VERSION = 32'h63747431;

  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [31 : 0] testop_a_reg;
  reg [31 : 0] testop_a_new;
  reg          testop_a_we;
  
  reg [31 : 0] testop_b_reg;
  reg [31 : 0] testop_b_new;
  reg          testop_b_we;

  reg [31 : 0] testresult_reg;
  reg [31 : 0] testresult_new;
  reg          testresult_we;

  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_read_data;
  reg          tmp_api_error;
  

  
  //----------------------------------------------------------------
  // Module instantiantions.
  //----------------------------------------------------------------


  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------

  
  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
      if (!reset_n)
        begin
          testop_a_reg   <= 32'h00000000;
          testop_b_reg   <= 32'h00000000;
          testresult_reg <= 32'h00000000;
        end
      else
        begin
          if  (testop_a_we)
            begin
              testop_a_reg <= testop_a_new;
            end
          
          if  (testop_b_we)
            begin
              testop_b_reg <= testop_b_new;
            end
          
          if  (testresult_we)
            begin
              testresult_reg <= testresult_new;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // api_logic
  //
  // Main update logic.
  //----------------------------------------------------------------
  always @*
    begin : api_logic
      tmp_read_data = 32'h00000000;
      testop_a_new  = 32'h00000000;
      testop_a_we   = 0;
      testop_b_new  = 32'h00000000;
      testop_b_we   = 0;
      
      if (cs)
        begin
          if (we)
            begin
              // Read operations.
            end
          else
            begin
              // Read operations.

              case (address)
                ADDR0:
                  begin
                    tmp_read_data = VERSION;
                  end
            end
        end
    end // api_logic
  
endmodule // cttest



//======================================================================
// EOF cttest
//======================================================================
