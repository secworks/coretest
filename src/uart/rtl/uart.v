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

  // Core ID constants.
  parameter CORE_NAME0   = 32'h75617274;  // "uart"
  parameter CORE_NAME1   = 32'h20202020;  // "    "
  parameter CORE_TYPE    = 32'h20202031;  // "   1"
  parameter CORE_VERSION = 32'h302e3031;  // "0.01"
  
  // The default clock rate is based on target clock frequency
  // divided by the bit rate times in order to hit the
  // center of the bits. I.e.
  // Clock: 50 MHz
  // Bitrate: 115200 bps
  // Divisor = 50*10E6 / (19200 * 4) = 651.041666
  parameter DEFAULT_CLK_DIV = 651;

  parameter DEFAULT_START_BITS = 2'h1;
  parameter DEFAULT_STOP_BITS  = 2'h1;
  parameter DEFAULT_DATA_BITS  = 4'h08;
  parameter DEFAULT_PARITY     = 1'h0;
  parameter DEFAULT_ENABLE     = 1'h1;
  parameter DEFAULT_LOOPBACK   = 1'h0;
  
  
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

  reg [3 : 0]  data_bits_reg;
  reg [3 : 0]  data_bits_new;
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
  reg         rx_rd_ptr_inc;

  reg [3 : 0] rx_wr_ptr_reg;
  reg [3 : 0] rx_wr_ptr_new;
  reg         rx_wr_ptr_we;
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
  reg         tx_rd_ptr_inc;

  reg [3 : 0] tx_wr_ptr_reg;
  reg [3 : 0] tx_wr_ptr_new;
  reg         tx_wr_ptr_we;
  reg         tx_wr_ptr_inc;

  reg [3 : 0] tx_ctr_reg;
  reg [3 : 0] tx_ctr_new;
  reg         tx_ctr_we;
  reg         tx_ctr_inc;
  reg         tx_ctr_dec;
  
  reg         rxd_reg;

  reg         txd_reg;
  reg         txd_new;
  reg         txd_we;

  reg [7 : 0] rxd_byte_reg;
  reg [7 : 0] rxd_byte_new;
  reg         rxd_byte_we;

  reg [2 : 0] bit_ctr_reg;
  reg [2 : 0] bit_ctr_new;
  reg         bit_ctr_we;
  reg         bit_ctr_rst;
  reg         bit_ctr_inc;
  
  reg [31 : 0] rx_buffer_full_ctr_reg;
  reg [31 : 0] rx_buffer_full_ctr_new;
  reg          rx_buffer_full_ctr_we;
  reg          rx_buffer_full_ctr_inc;
  reg          rx_buffer_full_ctr_rst;

  reg [31 : 0] rx_parity_error_ctr_reg;
  reg [31 : 0] rx_parity_error_ctr_new;
  reg          rx_parity_error_ctr_we;
  reg          rx_parity_error_ctr_inc;
  reg          rx_parity_error_ctr_rst;
               
  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_read_data;
  reg          tmp_error;

  reg          rx_empty;
  reg          rx_full;
  reg          tx_empty;
  reg          tx_full;

  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign txd = txd_reg;

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
          clk_div_reg      <= DEFAULT_CLK_DIV;
          start_bits_reg   <= DEFAULT_START_BITS;
          stop_bits_reg    <= DEFAULT_STOP_BITS;
          data_bits_reg    <= DEFAULT_DATA_BITS;
          parity_bit_reg   <= DEFAULT_PARITY;
          enable_bit_reg   <= DEFAULT_ENABLE;
          loopback_bit_reg <= DEFAULT_LOOPBACK;

          rxd_reg          <= 0;
          rxd_byte_reg     <= 8'h00;
          txd_reg          <= 0;
          
          rx_rd_ptr_reg    <= 4'h0;
          rx_wr_ptr_reg    <= 4'h0;
          rx_ctr_reg       <= 4'h0;
          tx_rd_ptr_reg    <= 4'h0;
          tx_wr_ptr_reg    <= 4'h0;
          tx_ctr_reg       <= 4'h0;

        end
      else
        begin
          // We sample the rx input port every cycle.
          rxd_reg <= rxd;

          if (rxd_byte_we)
            begin
              rxd_byte_reg <= {rxd_byte_reg[6 : 1], rxd_reg};
            end

          if (txd_we)
            begin
              txd_reg <= txd_new;
            end
                    
          if (clk_div_we)
            begin
              clk_div_reg <= clk_div_new;
            end

          if (start_bits_we)
            begin
              start_bits_reg <= start_bits_new;
            end

          if (stop_bits_we)
            begin
              stop_bits_reg <= stop_bits_new;
            end

          if (data_bits_we)
            begin
              data_bits_reg <= data_bits_new;
            end

          if (parity_bit_we)
            begin
              parity_bit_reg <= parity_bit_new;
            end

          if (enable_bit_we)
            begin
              enable_bit_reg <= enable_bit_new;
            end

          if (loopback_bit_we)
            begin
              loopback_bit_reg <= loopback_bit_new;
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

  
  //----------------------------------------------------------------
  // loopback_mux
  //
  // The mux controlled by the loopback_bit_reg. If set the
  // interfaces towards the internal system is tied together
  // making the UART echoing received back to the external host.
  //----------------------------------------------------------------
  always @*
    begin: loopback_mux
      if (loopback_bit_reg)
        begin
          
        end
      else
        begin

        end
    end // loopback_mux


  //----------------------------------------------------------------
  // rx_rd_ptr
  //
  // Read pointer for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_rd_ptr
      rx_rd_ptr_new = 4'h00;
      rx_rd_ptr_we  = 0;

      if (rx_rd_ptr_inc)
        begin
          rx_rd_ptr_new = rx_rd_ptr_reg + 1'b1;
          rx_rd_ptr_we  = 1;
        end
    end // rx_rd_ptr


  //----------------------------------------------------------------
  // rx_wr_ptr
  //
  // Write pointer for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_wr_ptr
      rx_wr_ptr_new = 4'h00;
      rx_wr_ptr_we  = 0;

      if (rx_wr_ptr_inc)
        begin
          rx_wr_ptr_new = rx_wr_ptr_reg + 1'b1;
          rx_wr_ptr_we  = 1;
        end
    end // rx_wr_ptr


  //----------------------------------------------------------------
  // rx_ctr
  //
  // Counter for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_ctr
      rx_ctr_new = 4'h00;
      rx_ctr_we  = 0;
      rx_empty   = 0;

      if (rx_ctr_reg == 4'h0)
        begin
          rx_buffer_empty = 1;
        end
      
      if ((rx_ctr_inc) && (!rx_ctr_dec))
        begin
          rx_ctr_new = rx_ctr_reg + 1'b1;
          rx_ctr_we  = 1;
        end
      else if ((!rx_ctr_inc) && (rx_ctr_dec))
        begin
          rx_ctr_new = rx_ctr_reg - 1'b1;
          rx_ctr_we  = 1;
        end
    end // rx_ctr

  
  //----------------------------------------------------------------
  // tx_rd_ptr
  //
  // Read pointer for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_rd_ptr
      tx_rd_ptr_new = 4'h00;
      tx_rd_ptr_we  = 0;

      if (tx_rd_ptr_inc)
        begin
          tx_rd_ptr_new = tx_rd_ptr_reg + 1'b1;
          tx_rd_ptr_we  = 1;
        end
    end // tx_rd_ptr


  //----------------------------------------------------------------
  // tx_wr_ptr
  //
  // Write pointer for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_wr_ptr
      tx_wr_ptr_new = 4'h00;
      tx_wr_ptr_we  = 0;

      if (tx_wr_ptr_inc)
        begin
          tx_wr_ptr_new = tx_wr_ptr_reg + 1'b1;
          tx_wr_ptr_we  = 1;
        end
    end // tx_wr_ptr


  //----------------------------------------------------------------
  // tx_ctr
  //
  // Counter for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_ctr
      tx_ctr_new = 4'h0;
      tx_ctr_we  = 0;
      tx_full    = 0;

      if (tx_ctr_reg == 4'f)
        begin
          tx_full = 1;
        end
      
      if ((tx_ctr_inc) && (!tx_ctr_dec))
        begin
          tx_ctr_new = tx_ctr_reg + 1'b1;
          tx_ctr_we  = 1;
        end
      else if ((!tx_ctr_inc) && (tx_ctr_dec))
        begin
          tx_ctr_new = tx_ctr_reg - 1'b1;
          tx_ctr_we  = 1;
        end
    end // tx_ctr


  //----------------------------------------------------------------
  // external_rx_engine
  //
  // Logic that implements the receive engine towards the externa
  // interface. Detects incoming data, collects it, if required 
  // checks parity and store correct data into the rx buffer.
  //----------------------------------------------------------------
  always @*
    begin: external_rx_engine
    end // external_rx_engine


  //----------------------------------------------------------------
  // external_tx_engine
  //
  // Logic that implements the transmit engine towards the external
  // interface. When there is data in the tx buffer, the engine 
  // transmits the data including start, stop and possible 
  // parity bits.
  //----------------------------------------------------------------
  always @*
    begin: external_tx_engine
    end // external_tx_engine

  
  //----------------------------------------------------------------
  // external_tx_engine
  //
  // Logic that implements the transmit engine towards the external
  // interface. When there is data in the tx buffer, the engine 
  // transmits the data including start, stop and possible 
  // parity bits.
  //----------------------------------------------------------------
  always @*
    begin: external_tx_engine
    end // external_tx_engine


  //----------------------------------------------------------------
  // internal_rx_engine
  //
  // Logic that implements the receive engine towards the internal
  // interface. When there is data in the rx buffer it asserts
  // the syn flag to signal that there is data available on 
  // rx_data. When the ack signal is asserted the syn flag is
  // dropped and the data is considered to have been consumed and
  // can be discarded.
  //----------------------------------------------------------------
  always @*
    begin: internal_rx_engine
    end // internal_rx_engine


  //----------------------------------------------------------------
  // internal_tx_engine
  //
  // Logic that implements the transmit engine towards the internal
  // interface. When the tx_syn flag is asserted the engine checks
  // if there are any room in the tx buffer. If it is, the data
  // available at tx_data is stored in the buffer. The tx_ack
  // is then asserted. The engine then waits for the syn flag
  // to be dropped.
  //----------------------------------------------------------------
  always @*
    begin: internal_tx_engine
    end // internal_tx_engine
  
endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================

