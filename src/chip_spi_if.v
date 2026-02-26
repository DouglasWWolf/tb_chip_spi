/*

===============================================================================
                   ------->  Revision History  <------
===============================================================================

   Date    Who  Ver  Changes
===============================================================================
18-Feb-26  DWW    1  Initial creation
===============================================================================

    This module provides a client-side interface to the chip_spi module.  It 
    features a simple synchronization feature in case and read and a write are
    requested at the same time.

*/


module chip_spi_if 
(
    input clk, resetn,

    // The client-side interface for write transactions
    input     [31:0] io_waddr,
    input     [31:0] io_wdata,
    input            io_write_stb,
    output           io_write_busy,

    // The client-side interface for read transactions
    input     [31:0] io_raddr,   
    input            io_read_stb, 
    output reg[31:0] io_rdata,
    output           io_read_busy,

    // These connect to the chip_spi module
    output reg[31:0] spi_addr,
    output reg[31:0] spi_wdata,
    output reg[ 1:0] spi_start_stb,
    input            spi_busy,
    input     [31:0] spi_rdata

);

// We strobe one of these to the chip_spi module to start a transaction
localparam OP_READ  = 0;
localparam OP_WRITE = 1;

// Used internally to track whether the current transaction is read or write
localparam READ_CMD  = 1;
localparam WRITE_CMD = 2;

//=============================================================================
// This state machine performs transactions with the chip_spi module
//=============================================================================
reg fsm_state;
reg pending_read;
reg pending_write;
reg op;
reg[31:0] pending_waddr, pending_raddr, pending_wdata;
//-----------------------------------------------------------------------------
always @(posedge clk) begin

    // This strobes high for one cycle at a time
    spi_start_stb <= 0;

    // Keep track of pending write transactions
    if (io_write_stb) begin
        pending_waddr <= io_waddr;
        pending_wdata <= io_wdata;
        pending_write <= 1;
    end

    // Keep track of pending read transactions
    if (io_read_stb) begin
        pending_raddr <= io_raddr;
        pending_read  <= 1;
    end

    // During reset, nothing is pending
    if (resetn == 0) begin
        fsm_state     <= 0;
        pending_read  <= 0;
        pending_write <= 0;
    end

    else case(fsm_state) 
    
        0:  if (pending_write) begin
                spi_addr      <= pending_waddr;
                spi_wdata     <= pending_wdata;
                spi_start_stb <= WRITE_CMD;
                op            <= OP_WRITE;
                fsm_state     <= 1;
            end

            else if (pending_read) begin
                spi_addr      <= pending_raddr;
                spi_start_stb <= READ_CMD;
                op            <= OP_READ;
                fsm_state     <= 1;
            end

            else if (io_write_stb) begin
                spi_addr      <= io_waddr;
                spi_wdata     <= io_wdata;
                spi_start_stb <= WRITE_CMD;
                op            <= OP_WRITE;
                fsm_state     <= 1;
            end

            else if (io_read_stb) begin
                spi_addr      <= io_raddr;
                spi_start_stb <= READ_CMD;
                op            <= OP_READ;
                fsm_state     <= 1;
            end

        1:  if (!spi_busy) begin
                if (op == OP_READ) begin
                    io_rdata      <= spi_rdata;
                    pending_read  <= 0;
                end else begin
                    pending_write <= 0;
                end
                fsm_state <= 0;
            end

    endcase
end

// Tell the user we're busy during any transaction
assign io_read_busy  = io_read_stb  | pending_read;
assign io_write_busy = io_write_stb | pending_write;
//=============================================================================

endmodule
