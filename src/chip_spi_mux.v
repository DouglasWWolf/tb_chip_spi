/*

===============================================================================
                   ------->  Revision History  <------
===============================================================================

   Date    Who  Ver  Changes
===============================================================================
18-Feb-26  DWW    1  Initial creation
===============================================================================

This module provides a 2-to-1 mux for the chip_spi module.   It is assumed 
that one of the mux inputs will be connected to the SMEM-writer, and the other
will be connected to the chip-register/SMEM read-write logic.

The client-side starts a transaction by strobing "start_stb_N" for one clock
cycle.   After the busy_N signal goes low, the transaction has been completed
and if it was a read transaction, the resulting data will be waiting in
"rdata_N"

*/



module chip_spi_mux
(
    input clk, resetn,

    //---------------------
    // The mux input ports
    //---------------------
    input  [ 1:0] start_stb_0,  start_stb_1,
    input  [31:0]      addr_0,       addr_1,
    input  [31:0]     wdata_0,      wdata_1,
    output [31:0]     rdata_0,      rdata_1,
    output             busy_0,       busy_1,

    //------------------------------------------
    // Mux output that connects to the chip_spi
    //------------------------------------------
    output reg[ 1:0] spi_start_stb,
    output reg[31:0] spi_addr,
    output reg[31:0] spi_wdata,
    input     [31:0] spi_rdata,
    input            spi_busy
);

//=============================================================================
// This state machine forwards SPI transaction requests from the mux input
// to the chip_spi, and forwards the chip_spi read-result back to the mux 
// input that requested the transaction.
//=============================================================================

// State of the state machine
reg fsm_state;          

// When fsm_state = 1, this is the mux channel #
reg active_channel;     

// 0 = Nothing is pending
// 1 = A read is pending
// 2 = A write is pending
// 3 = A very fast simulated write
reg[ 1:0] pending_start[0:1];
reg[31:0] pending_wdata[0:1];
reg[31:0]  pending_addr[0:1];

// This is the read data returned to each mux channel
reg[31:0] read_data[0:1];
assign rdata_0 = read_data[0];
assign rdata_1 = read_data[1];
//-----------------------------------------------------------------------------
always @(posedge clk) begin

    // This strobes high for one cycle at a time
    spi_start_stb <= 0;

    // Keep track of pending transactions on channel 0
    if (start_stb_0) begin
        pending_start[0] <= start_stb_0;
        pending_wdata[0] <=     wdata_0;
        pending_addr [0] <=      addr_0;
    end

    // Keep track of pending transactions on channel 1
    if (start_stb_1) begin
        pending_start[1] <= start_stb_1;
        pending_wdata[1] <=     wdata_1;
        pending_addr [1] <=      addr_1;
    end

    // If we're in reset, initialize all the outputs
    if (resetn == 0) begin
        spi_addr         <= 0;
        spi_wdata        <= 0;
        pending_start[0] <= 0;
        pending_start[1] <= 0;
        fsm_state        <= 0;
    end

    else case(fsm_state)

        0:  if (pending_start[0]) begin
                spi_addr       <=  pending_addr[0];
                spi_wdata      <= pending_wdata[0];
                spi_start_stb  <= pending_start[0];
                active_channel <= 0;
                fsm_state      <= 1;
            end

            else if (pending_start[1]) begin
                spi_addr       <=  pending_addr[1];
                spi_wdata      <= pending_wdata[1];
                spi_start_stb  <= pending_start[1];
                active_channel <= 1;
                fsm_state      <= 1;
            end

            else if (start_stb_0) begin
                spi_addr       <=      addr_0;
                spi_wdata      <=     wdata_0;
                spi_start_stb  <= start_stb_0;
                active_channel <= 0;
                fsm_state      <= 1;
            end

            else if (start_stb_1) begin
                spi_addr       <=      addr_1;
                spi_wdata      <=     wdata_1;
                spi_start_stb  <= start_stb_1;
                active_channel <= 1;
                fsm_state      <= 1;
            end

        // Here we wait for the chip_spi module to complete the transaction, 
        // then we send the result data back to the appropriate mux channel
        1:  if (!spi_busy) begin
                read_data    [active_channel] <= spi_rdata;
                pending_start[active_channel] <= 0;               
                fsm_state                     <= 0;
            end
    endcase

end

// A pending mux channel remains "busy" until it has been serviced
assign busy_0 = start_stb_0 || pending_start[0];
assign busy_1 = start_stb_1 || pending_start[1];
//=============================================================================

endmodule