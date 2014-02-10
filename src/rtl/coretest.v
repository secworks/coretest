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
  parameter SOC = 8'h55;
  parameter EOC = 8'haa;

  parameter RST_CMD = 8'h01; 
  parameter RD_CMD  = 8'h10; 
  parameter WR_CMD  = 8'h20; 

  
  // Response constants.
  parameter SOR = 8'haa;
  parameter EOR = 8'h55;

  
  // FSM states.
  parameter CTRL_IDLE      = 8'h00;

  parameter CTRL_RX_START  = 8'h10;
  parameter CTRL_RX_BYTES  = 8'h11;
  parameter CTRL_RX_END    = 8'h12;

  parameter CTRL_TX_START  = 8'h20;
  parameter CTRL_TX_BYTES  = 8'h21;
  parameter CTRL_TX_END    = 8'h22;
  
  parameter CTRL_RST_START = 8'h30;
  parameter CTRL_RST_END   = 8'h31;

  parameter CTRL_RD_START  = 8'h50;
  parameter CTRL_RD_END    = 8'h51;

  parameter CTRL_WR_START  = 8'h60;
  parameter CTRL_WR_END    = 8'h61;
                            
  
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
  
  reg [7 : 0] coretest_ctrl_reg;
  reg [7 : 0] coretest_ctrl_new;
  reg         coretest_ctrl_we;

  
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
    begin : reg_update
      if (!reset_n)
        begin
          coretest_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (coretest_ctrl_we)
            begin
              coretest_ctrl_reg <= coretest_ctrl_new;
            end
        end
    end // reg_update

  
  //----------------------------------------------------------------
  // coretest_ctrl
  //
  // Control FSM logic.
  //----------------------------------------------------------------
  always @*
    begin : coretest_ctrl
      // Default assignments.
      coretest_ctrl_new = CTRL_IDLE;
      coretest_ctrl_we  = 0;

      case (coretest_ctrl_reg)
        CTRL_RX_START:
          begin
          end

        CTRL_RX_BYTES:
          begin
          end

        CTRL_RX_END:
          begin
          end

        CTRL_TX_START:
          begin
          end

        CTRL_TX_BYTES:
          begin
          end

        CTRL_TX_END:
          begin
          end
  
        CTRL_RST_START:
          begin
          end

        CTRL_RST_END:
          begin
          end

        CTRL_RD_START:
          begin
          end

        CTRL_RD_END:
          begin
          end
        
        CTRL_WR_START:
          begin
          end

        CTRL_WR_END: 
          begin
          end

        default:
          begin
            // If we encounter an unknown state we move 
            // back to idle.
            coretest_ctrl_we = 1;
          end
      endcase // case (coretest_ctrl_reg)
    end // coretest_ctrl
  
endmodule // cttest

//======================================================================
// EOF coretest
//======================================================================
