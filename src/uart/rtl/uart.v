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
  // Divisor = 50*10E6 / (19200 * 4) = 651.041666
  parameter DEFAULT_CLK_DIV = 651;

  CORE_NAME0   = 32'h75617274;  // "uart"
  CORE_NAME1   = 32'h20202020;  // "    "
  CORE_TYPE    = 32'h20202031;  // "   1"
  CORE_VERSION = 32'h302e3031;  // "0.01"

  
  // API addresses.
  parameter ADDR_CORE_NAME0   = 4'h0;
  parameter ADDR_CORE_NAME1   = 4'h1;
  parameter ADDR_CORE_TYPE    = 4'h2;
  parameter ADDR_CORE_VERSION = 4'h3;
  
  parameter ADDR_CRTL         = 4'h8; // Enable/disable. Loopback on/off.
  parameter ADDR_STATUS       = 4'h9; // Buffer status.
  parameter ADDR_CONFIG       = 4'ha; // Num start, data, stop, parity bits.
  parameter ADDR_CLK_DIV      = 4'hb; // Clock divisor to set bitrate.

  parameter ADDR_STAT_PARITY  = 4'hc; // Stats: Num parity errors detected.
  parameter ADDR_STAT_RX_FULL = 4'hd; // Stats: Num Rx buffer full events.
  parameter ADDR_STAT_TC_FULL = 4'he; // Stats: Num Tx buffer full events.
  
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [15 : 0] clk_div_reg;
  reg [15 : 0] clk_div_new;
  reg          clk_div_we;

  reg          enable_bit_reg;
  reg          enable_bit_new;
  reg          enable_bit_we;
  
  reg          loopback_bit_reg;
  reg          loopback_bit_new;
  reg          loopback_bit_we;
  
  reg [1 : 0]  start_bits_reg;
  reg [1 : 0]  start_bits_new;
  reg          start_bits_we;

  reg [1 : 0]  stop_bits_reg;
  reg [1 : 0]  stop_bits_new;
  reg          stop_bits_we;

  reg [1 : 0]  data_bits_reg;
  reg [1 : 0]  data_bits_new;
  reg          data_bits_we;

  reg          parity_bit_reg;
  reg          parity_bit_new;
  reg          parity_bit_we;
  
  
  // Rx data buffer with associated
  // read and write pointers as well
  // as counter for number of elements
  // in the buffer.
  reg [7 : 0] rx_buffer [0 : 15];

  reg [3 : 0] rx_rd_ptr_reg;
  reg [3 : 0] rx_rd_ptr_new;
  reg         rx_rd_ptr_we;
  reg         rx_rd_ptr_rst;
  reg         rx_rd_ptr_inc;

  reg [3 : 0] rx_wr_ptr_reg;
  reg [3 : 0] rx_wr_ptr_new;
  reg         rx_wr_ptr_we;
  reg         rx_wr_ptr_rst;
  reg         rx_wr_ptr_inc;

  reg [3 : 0] rx_ctr_reg;
  reg [3 : 0] rx_ctr_new;
  reg         rx_ctr_we;
  reg         rx_ctr_inc;
  reg         rx_ctr_dec;

  // Tx data buffer with associated
  // read and write pointers as well
  // as counter for number of elements
  // in the buffer.
  reg [7 : 0] tx_buffer [0 : 15];

  reg [3 : 0] tx_rd_ptr_reg;
  reg [3 : 0] tx_rd_ptr_new;
  reg         tx_rd_ptr_we;
  reg         tx_rd_ptr_rst;
  reg         tx_rd_ptr_inc;

  reg [3 : 0] tx_wr_ptr_reg;
  reg [3 : 0] tx_wr_ptr_new;
  reg         tx_wr_ptr_we;
  reg         tx_wr_ptr_rst;
  reg         tx_wr_ptr_inc;

  reg [3 : 0] tx_ctr_reg;
  reg [3 : 0] tx_ctr_new;
  reg         tx_ctr_we;
  reg         tx_ctr_inc;
  reg         tx_ctr_dec;
  
  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_read_data;
  reg          tmp_error;

  reg          rx_buffer_empty;
  reg          rx_buffer_full;

  reg          tx_buffer_empty;
  reg          tx_buffer_full;

  
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
          rx_rd_ptr_reg <= 4'h0;
          rx_wr_ptr_reg <= 4'h0;
          rx_ctr_reg    <= 4'h0;
          tx_rd_ptr_reg <= 4'h0;
          tx_wr_ptr_reg <= 4'h0;
          tx_ctr_reg    <= 4'h0;
          
          clk_div_reg   <= DEFAULT_CLK_DIV;
        end
      else
        begin
          if (clk_div_we)
            begin
              clk_div_reg <= clk_div_new;
            end
          
          if (rx_rd_ptr_we)
            begin
              rx_rd_ptr_reg <= rx_rd_ptr_new;
            end
          
          if (rx_wr_ptr_we)
            begin
              rx_wr_ptr_reg <= rx_wr_ptr_new;
            end
          
          if (rx_ctr_we)
            begin
              rx_ctr_reg <= rx_ctr_new;
            end
          
          if (tx_rd_ptr_we)
            begin
              tx_rd_ptr_reg <= tx_rd_ptr_new;
            end
          
          if (tx_wr_ptr_we)
            begin
              tx_wr_ptr_reg <= tx_wr_ptr_new;
            end
          
          if (tx_ctr_we)
            begin
              tx_ctr_reg <= tx_ctr_new;
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

                ADDR_CORE_TYPE:
                  begin
                  end

                ADDR_CORE_VERSION:
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

