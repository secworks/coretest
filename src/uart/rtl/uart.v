//======================================================================
//
// uart.v
// ------
// A simple universal asynchronous receiver/transmitter (UART)
// interface. The interface contains a buffers for a few bytes
// and can handle start and stop bits. But in general is.
// rather simple. The primary purpose is as host interface
// for the coretest design.
//
// Note that the UART has a separate API interface to allow
// a core to change settings such as speed. But the core
// does not to be set up in order to start operating.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014  Secworks Sweden AB
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

module uart(
            input wire           clk,
            input wire           reset,

            // External interface
            input wire           rxd,
            output wire          txd,
            
            // Internal interface
            output wire          rx_syn,
            output wire [7 : 0]  rx_data,
            intput wire          rx_ack,

            input wire           tx_syn,
            input wire [7 : 0]   tx_data,
            output wire          tx_ack,
            
            // API interface
            input wire           cs,
            input wire           we,
            input wire [3 : 0]   address,
            input wire [31 : 0]  write_data,
            output wire [31 : 0] read_data,
            output wire          error
           );

  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // The default clock rate is based on target clock frequency
  // divided by the bit rate times in order to hit the
  // center of the bits. I.e.
  // Clock: 50 MHz
  // Bitrate: 115200 bps
  // Divisor = 50*10E6 / (19200 * 4) = 651
  parameter DEFAULT_CLK_DIV = 651;
  
  parameter ADDR_CORE_NAME0 = 4'h0;
  parameter ADDR_CORE_NAME1 = 4'h1;
  parameter ADDR_CORE_TYPE0 = 4'h2;
  parameter ADDR_CORE_TYPE1 = 4'h3;
  parameter ADDR_CLK_DIV    = 4'h8;

  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [15 : 0] clk_div_reg;
  reg [15 : 0] clk_div_new;
  reg          clk_div_we;
  
  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_read_data;
  reg          tmp_error;
  
  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;
  assign error     = tmp_error;
  
  
  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (reset)
        begin
          clk_div_reg <= DEFAULT_CLK_DIV;
        end
      else
        begin
          if (clk_div_we)
            begin
              clk_div_reg <= clk_div_new;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //
  // The core API that allows an internal host to control the
  // core functionality.
  //----------------------------------------------------------------
  always @*
    begin: api
      // Default assignments.
      tmp_read_data = 32'h00000000;
      tmp_error     = 0;
      
      clk_div_new   = 16'h0000;
      clk_div_we    = 0;

      if (cs)
        begin
          if (we)
            begin
              // Write operations.
              case (address)
                ADDR_CORE_NAME0:
                  begin
                  end

                ADDR_CORE_NAME1:
                  begin
                  end

                ADDR_CORE_TYPE0:
                  begin
                  end

                ADDR_CORE_TYPE1:
                  begin
                  end

                ADDR_CLK_DIV:
                  begin
                    clk_div_new = write_data[15 : 0];
                    clk_div_we  = 1;
                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
          else
            begin
              // Read operations.
              case (address)
                ADDR_CORE_NAME0:
                  begin
                  end

                ADDR_CORE_NAME1:
                  begin
                  end

                ADDR_CORE_TYPE0:
                  begin
                  end

                ADDR_CORE_TYPE1:
                  begin
                  end

                ADDR_CLK_DIV:
                  begin
                    tmp_read_data = {16'h0000, clk_div_reg};
                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
        end
    end
  
  
endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================

