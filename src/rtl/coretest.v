//======================================================================
//
// coretest.v
// ----------
// Top level of the Cryptech test core. Simply a 32-bit interface
// with some intrernal functionality to see that we can read and write
// registers in the FPGA from the host. This core will be the basis
// for all core top levels.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2013  Secworks Sweden AB
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

module coretest(
                input wire           clk,
                input wire           reset_n,
                
                // Interface to communication core
                input wire           rx_syn,
                input wire [7 : 0]   rx_data,
                output wire          rx_ack,
                
                output wire          tx_syn,
                output wire [7 : 0]  tx_data,
                input wire           tx_ack,
                
                // Interface to the core being tested.
                output               core_reset_n,
                output wire          core_cs,
                output wire          core_we,
                output wire [15 : 0] core_address,
                output wire [31 : 0] core_write_data,
                input wire  [31 : 0] core_read_data,
                input wire           core_error
               );

  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // Command constants.
  parameter SOC       = 8'h55;
  parameter EOC       = 8'haa;

  parameter RESET_CMD = 8'h01; 
  parameter READ_CMD  = 8'h10; 
  parameter WRITE_CMD = 8'h11; 

  
  // Response constants.
  parameter SOR      = 8'haa;
  parameter EOR      = 8'h55;

  parameter UNKNOWN  = 8'hfe;
  parameter ERROR    = 8'hfd;
  parameter READ_OK  = 8'h7f;
  parameter WRITE_OK = 8'h7e;
  parameter RESET_OK = 8'h7d;


  // rx_engine states.
  parameter RX_IDLE = 3'h0;
  parameter RX_SYN  = 3'h1;
  parameter RX_ACK  = 3'h2;
  parameter RX_DONE = 3'h3;


  // rx_engine states.
  parameter TX_IDLE = 3'h0;
  parameter TX_SYN  = 3'h1;
  parameter TX_ACK  = 3'h2;
  parameter TX_DONE = 3'h3;
  
  
  // test_engine states.
  parameter TEST_IDLE      = 8'h00;

  parameter TEST_RX_START  = 8'h10;
  parameter TEST_RX_BYTES  = 8'h11;
  parameter TEST_RX_END    = 8'h12;

  parameter TEST_TX_START  = 8'h20;
  parameter TEST_TX_BYTES  = 8'h21;
  parameter TEST_TX_END    = 8'h22;
  
  parameter TEST_RST_START = 8'h30;
  parameter TEST_RST_END   = 8'h31;

  parameter TEST_RD_START  = 8'h50;
  parameter TEST_RD_END    = 8'h51;

  parameter TEST_WR_START  = 8'h60;
  parameter TEST_WR_END    = 8'h61;
                            
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          rx_syn_reg;
  reg          rx_syn_new;
  reg          rx_syn_we;

  reg          rx_ack_reg;
  reg          rx_ack_new;
  reg          rx_ack_we;

  reg [7 : 0]  rx_data_reg;
  reg [7 : 0]  rx_data_new;
  reg          rx_data_we;

  
  reg          tx_syn_reg;
  reg          tx_syn_new;
  reg          tx_syn_we;

  reg          tx_ack_reg;
  reg          tx_ack_new;
  reg          tx_ack_we;

  reg [7 : 0]  tx_data_reg;
  reg [7 : 0]  tx_data_new;
  reg          tx_data_we;
  
  reg          core_reset_n_reg;
  reg          core_cs_reg;
  reg          core_we_reg;
  reg [15 : 0] core_address_reg;
  reg [31 : 0] core_write_data_reg;
  reg [31 : 0] core_read_data_reg;
  reg          core_error_reg;

  reg [2 : 0]  rx_engine_reg;
  reg [2 : 0]  rx_engine_new;
  reg          rx_engine_we;
  
  reg [2 : 0]  tx_engine_reg;
  reg [2 : 0]  tx_engine_new;
  reg          tx_engine_we;
  
  reg [7 : 0]  test_engine_reg;
  reg [7 : 0]  test_engine_new;
  reg          test_engine_we;

  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  
  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign rx_ack = rx_ack_reg;

  assign tx_syn  = tx_syn_reg;
  assign tx_data = tx_data_reg;

  assign core_reset_n    = core_reset_n_reg;
  assign core_cs         = core_cs_reg;
  assign core_we         = core_we_reg;
  assign core_address    = core_address_reg;
  assign core_write_data = core_write_data_reg;
  
  
  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (!reset_n)
        begin
          rx_engine       <= RX_IDLE;
          tx_engine       <= TX_IDLE;
          test_engine_reg <= TEST_IDLE;
        end
      else
        begin
          if (rx_engine_we)
            begin
              rx_engine_reg <= rx_engine_new;
            end
          
          if (tx_engine_we)
            begin
              tx_engine_reg <= tx_engine_new;
            end
          
          if (test_engine_we)
            begin
              test_engine_reg <= test_engine_new;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // rx_engine
  //
  // FSM responsible for handling receiving message bytes from the
  // host interface and signalling the test engine that there is
  // a new command to be executed.
  //----------------------------------------------------------------
  always @*
    begin: rx_engine
      // Default assignments
      rx_engine_new = RX_IDLE;
      rx_engine_we  = 0;

      case (rx_engine_reg)
        RX_IDLE:
          begin
          end

        default:
          begin
          end
      endcase // case (rx_engine_reg)
    end // rx_engine
  


  //----------------------------------------------------------------
  // tx_engine
  //
  // FSM responsible for handling transmitting message bytes
  // to the host interface.
  //----------------------------------------------------------------
  always @*
    begin: tx_engine
      // Default assignments
      tx_engine_new = TX_IDLE;
      tx_engine_we  = 0;

      case (tx_engine_reg)
        TX_IDLE:
          begin
          end

        default:
          begin
          end
      endcase // case (tx_engine_reg)
    end // rx_engine
  
  
  //----------------------------------------------------------------
  // test_engine
  //
  // Test engine FSM logic. Parses received commands, tries to
  // execute the commands and assmbles the response to the
  // given commands.
  //----------------------------------------------------------------
  always @*
    begin: test_engine
      // Default assignments.
      test_engine_new = TEST_IDLE;
      test_engine_we  = 0;

      case (test_engine_reg)
        TEST_IDLE:
          begin
          end
        
        TEST_RX_START:
          begin
          end

        TEST_RX_BYTES:
          begin
          end

        TEST_RX_END:
          begin
          end

        TEST_TX_START:
          begin
          end

        TEST_TX_BYTES:
          begin
          end

        TEST_TX_END:
          begin
          end
  
        TEST_RST_START:
          begin
          end

        TEST_RST_END:
          begin
          end

        TEST_RD_START:
          begin
          end

        TEST_RD_END:
          begin
          end
        
        TEST_WR_START:
          begin
          end

        TEST_WR_END: 
          begin
          end

        default:
          begin
            // If we encounter an unknown state we move 
            // back to idle.
            coretest_ctrl_we = 1;
          end
      endcase // case (test_engine_reg)
    end // test_engine
  
endmodule // cttest

//======================================================================
// EOF coretest.v
//======================================================================
