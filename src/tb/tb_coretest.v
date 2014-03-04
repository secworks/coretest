//======================================================================
//
// tb_coretest.v
// -------------
// Testbench for coretest. Generates commands and observes responses.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014, Secworks Sweden AB
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or 
// without modification, are permitted provided that the following 
// conditions are met: 
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
// 
// 2. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the 
//    distribution. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

//------------------------------------------------------------------
// Simulator directives.
//------------------------------------------------------------------
`timescale 1ns/10ps


module tb_coretest();
  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG           = 0;
  parameter VERBOSE         = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD      = CLK_HALF_PERIOD * 2;
  
  
  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0]  cycle_ctr;
  reg [31 : 0]  error_ctr;
  reg [31 : 0]  tc_ctr;

  reg           tb_clk;
  reg           tb_reset_n;

  reg           tb_rx_syn;
  reg [7 : 0]   tb_rx_data;
  wire          tb_rx_ack;
  wire          tb_tx_syn;
  wire [7 : 0]  tb_tx_data;
  reg           tb_tx_ack;

  wire          tb_core_reset;
  wire          tb_core_cs;
  wire          tb_core_we;
  wire [15 : 0] tb_core_address;
  wire [31 : 0] tb_core_write_data;
  reg [31 : 0]  tb_core_read_data;
  reg           tb_core_error;
  

  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  coretest dut(
               .clk(tb_clk),
               .reset_n(tb_reset_n),
                
               // Interface to communication core
               .rx_syn(tb_rx_syn),
               .rx_data(tb_rx_data),
               .rx_ack(tb_rx_ack),
               
               .tx_syn(tb_tx_syn),
               .tx_data(tb_tx_data),
               .tx_ack(tb_tx_ack),
                
               // Interface to the core being tested.
               .core_reset(tb_core_reset),
               .core_cs(tb_core_cs),
               .core_we(tb_core_we),
               .core_address(tb_core_address),
               .core_write_data(tb_core_write_data),
               .core_read_data(tb_core_read_data),
               .core_error(tb_core_error)
              );

  
  //----------------------------------------------------------------
  // Concurrent assignments.
  //----------------------------------------------------------------
  

  //----------------------------------------------------------------
  // clk_gen
  //
  // Clock generator process. 
  //----------------------------------------------------------------
  always 
    begin : clk_gen
      #CLK_HALF_PERIOD tb_clk = !tb_clk;
    end // clk_gen
    

  //----------------------------------------------------------------
  // sys_monitor
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      #(CLK_PERIOD);      
      if (DEBUG)
        begin
          $display("");
        end

      if (VERBOSE)
        begin
          $display("cycle: 0x%016x", cycle_ctr);
        end
      cycle_ctr = cycle_ctr + 1;
    end

  //----------------------------------------------------------------
  // bus_monitor_response
  // Observes operation on the bus interface and responds
  // on read operations.
  //----------------------------------------------------------------
  always @*
    begin : bus_response
      if (tb_core_cs)
        begin
          if (tb_core_we)
            begin
              $display("*** Write operation on bus: address 0x%04x = 0x%08x", 
                       tb_core_address, tb_core_write_data);
            end
          else
            begin
              $display("*** REad operation on bus: address 0x%04x",
                       tb_core_address);
              tb_core_read_data = 32'hdeadbeef;
            end
        end
    end // bus_response
  
  
  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dut when needed.
  //----------------------------------------------------------------
  task dump_dut_state();
    begin
      $display("State of DUT");
      $display("------------");
      $display("Inputs and outputs:");
      $display("rx_syn = 0x%01x, rx_data = 0x%02x, rx_ack = 0x%01x",
               dut.rx_syn, dut.rx_data, dut.rx_ack);
      $display("tx_syn = 0x%01x, tx_data = 0x%02x, tx_ack = 0x%01x",
               dut.tx_syn, dut.tx_data, dut.tx_ack);
      $display("cs = 0x%01x, we = 0x%01x, address = 0x%04x, write_data = 0x%08x, read_data = 0x%08x, error = 0x%01x",
               dut.core_cs, dut.core_we, dut.core_address, dut.core_write_data, dut.core_read_data, dut.core_error);
      $display("");

      $display("Control signals and FSM state:");
      $display("test_engine_reg = 0x%02x, cmd_reg = 0x%02x, rx_buffer_ptr = 0x%02x, tx_buffer_ptr = 0x%02x",
               dut.test_engine_reg, dut.cmd_reg, dut.rx_buffer_ptr_reg, dut.tx_buffer_ptr_reg);
      $display("");
    end
  endtask // dump_dut_state

  
  //----------------------------------------------------------------
  // reset_dut()
  //----------------------------------------------------------------
  task reset_dut();
    begin
      $display("*** Toggle reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut

  
  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim();
    begin
      cycle_ctr         = 0;
      error_ctr         = 0;
      tc_ctr            = 0;
      
      tb_clk            = 0;
      tb_reset_n        = 1;
      tb_rx_syn         = 0;
      tb_rx_data        = 8'h00;
      tb_tx_ack         = 0;
      tb_core_read_data = 32'h00000000;
      tb_core_error     = 0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // send_byte
  //
  // Send a byte of data to the DUT.
  //----------------------------------------------------------------
  task send_byte(input [7 : 0] data);
    integer i;
    begin
      $display("*** Sending byte 0x%02x to the dut.", data);

      tb_rx_data = data;
      tb_rx_syn  = 1;
      while (!tb_rx_ack)
        begin
          #CLK_PERIOD;
        end
      tb_rx_syn  = 0;
    end
  endtask // transmit_byte

  
  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result();
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d test cases did not complete successfully.", error_ctr);
        end
    end
  endtask // display_test_result
                         
    
  //----------------------------------------------------------------
  // coretest_test
  // The main test functionality. 
  //----------------------------------------------------------------
  initial
    begin : coretest_test
      $display("   -- Testbench for coretest started --");

      init_sim();
      dump_dut_state();
      reset_dut();
      dump_dut_state();
      
      display_test_result();
      $display("*** Simulation done.");
      $finish;
    end // coretest_test
endmodule // tb_coretest

//======================================================================
// EOF tb_coretest.v
//======================================================================
