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
               
               .tx_syn(),
               .tx_data(),
               .tx_ack(),
                
               // Interface to the core being tested.
               .core_reset(),
               .core_cs(),
               .core_we(),
               .core_address(),
               .core_write_data(),
               .core_read_data(),
               .core_error()
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
  // dump_dut_state()
  //
  // Dump the state of the dut when needed.
  //----------------------------------------------------------------
  task dump_dut_state();
    begin
      $display("State of DUT");
      $display("------------");
      $display("Inputs and outputs:");
      $display("");

      $display("Control signals and FSM state:");
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
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      
      tb_clk     = 0;
      tb_reset_n = 1;

    end
  endtask // init_sim


  //----------------------------------------------------------------
  // transmit_byte
  //
  // Transmit a byte of data to the DUT receive port.
  //----------------------------------------------------------------
  task transmit_byte(input [7 : 0] data);
    integer i;
    begin
      $display("*** Transmitting byte 0x%02x to the dut.", data);

      #10;
      
      // Start bit
      $display("*** Transmitting start bit.");
      tb_rxd = 0;
      #(CLK_PERIOD * dut.core.DEFAULT_CLK_RATE);

      // Send the bits LSB first.
      for (i = 0 ; i < 8 ; i = i + 1)
        begin
          $display("*** Transmitting data[%1d] = 0x%01x.", i, data[i]);
          tb_rxd = data[i];
          #(CLK_PERIOD * dut.core.DEFAULT_CLK_RATE);
        end

      // Send two stop bits. I.e. two bit times high (mark) value.
      $display("*** Transmitting two stop bits.");
      tb_rxd = 1;
      #(2 * CLK_PERIOD * dut.core.DEFAULT_CLK_RATE * dut.core.DEFAULT_STOP_BITS);
      $display("*** End of transmission.");
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
