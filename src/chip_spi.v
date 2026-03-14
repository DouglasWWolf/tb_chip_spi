/*

===============================================================================
                   ------->  Revision History  <------
===============================================================================

   Date    Who  Ver  Changes
===============================================================================
18-Feb-26  DWW    1  Initial creation
===============================================================================

This module reads and writes 32-bit words in the sensor-chip via the SPI
interface.

This assumes that all data sent during SPI transactions is sent MSB first and
further assumes that the chip clocks in data on rising edges.

The SPI clock is synchronous to a free-running clock that is obtained
by dividing "hsi_clk" by 8 then synchronizing it to "clk".   The neccessity
of the SPI clock running no faster than "hsi_clk / 8" is dictated by the
sensor-chip

A single register or SMEM read/write consists of two 40-bit transactions, each
of those consisting of an 8-bit command word and a 32-bit data-value.

The 32-bit data in the first transaction is the upper 27-bits of the register
or SMEM address.   These 32-bits must be in LITTLE-ENDIAN order.

The 32-bit data sent in the 2nd transaction of a write must be in LITTLE-ENDIAN 
order!

The 32-bit data received in the 2nd transaction of read will always be in 
little endian-order.

*/


module chip_spi 
(

    // This is the HSI clock, running no faster than 80 MHz.  We will divide it down
    // by a factor of 8, meaning the divided down clock will never be faster than
    // 10 MHz.
    input hsi_clk,

    input clk, resetn,

    // This is a free-running clock that never stops.  It is synchronous to clk
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 free_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 10000000" *)
    output free_clk,

    //----------------
    // User interface    
    //----------------
    input     [ 1:0] start,
    input     [31:0] addr,
    input     [31:0] wdata,
    output    [31:0] rdata,
    output           busy,


    //----------------
    //  SPI interface    
    //----------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 spi_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 10000000" *)
    output     spi_clk,
    output reg spi_mosi,
    input      spi_miso,
    output reg spi_cs_n,

    // An input from the chip-simulator's MISO pin, along
    // with a way to select is as the MISO input
    input sim_miso,
    input sim_select
);


// Chip select is active low
localparam ASSERT_CHIP_SELECT  = 0;
localparam RELEASE_CHIP_SELECT = 1;

// The two-bit "start" field will strobe high with one of these values
localparam START_READ  = 1;
localparam START_WRITE = 2;
localparam START_SIM   = 3;

// When sending SPI commands to the chip, these are the opcodes
localparam[0:0] OP_READ  = 0;
localparam[0:0] OP_WRITE = 1;

// This tells the chip that we will be sending 32-bits of data
localparam[0:0] CHIP_SPI_MODE32 = 1'b1;

// This is the SPI register that determines the upper 27 bits of the address
// in the chip's address space
localparam[0:0] MAP_SELECT_ON  = 1'b1;
localparam[0:0] MAP_SELECT_OFF = 1'b0;


// The number of spi_clk cycles from:
//
// (1) Chip-select assertion to the first rising edge of spi_clk
// (2) The last falling edge of spi_clk to chip-select release
localparam PORCH_CYCLES = 1;

// Bits are numbered in MSB-to-LSB in the usual manner - 79 down to 0
// After bit 40 has been sent, we will suspend the spi_clk to provide
// a gap between transactions.  
localparam TRANSACTION_BOUNDARY = 40;

// Between transactions, we will insert this many idle free_clk cycles
localparam TRANSACTION_SUSPEND = 2;

// During a read of SMEM, we will stretch the spi_clk cycle between the 
// command-byte and the 32 "data" cycles.
localparam READ_DATA_BOUNDARY = 32;

//=============================================================================
// This function swaps big-endian to little-endian or vice-versa
//=============================================================================
function [31:0] byte_swap (input [31:0] value);
    byte_swap = {value[7:0], value[15:8], value[23:16], value[31:24]};
endfunction
//=============================================================================

//=============================================================================
// We need to do edge detection on "free_clk"
//=============================================================================
reg prior_free_clk;
always @(posedge clk) prior_free_clk <= free_clk;
wire rising_edge  = (prior_free_clk == 0) & (free_clk == 1);
wire falling_edge = (prior_free_clk == 1) & (free_clk == 0);
//=============================================================================


// These are the address, write-data, and operation (read or write) for this
// transaction
reg[31:0] user_addr;
reg[31:0] user_wdata;
reg       user_op;

// A read/write into the chip's address space requres two 40-bit transactions.
// The 1st transaction tells the chip what the upper 27 bits of the address
// will be, while the 2nd transactions provides the lower 5 bits of the address
// and the 32 bits of data.
wire[79:0] transaction =
{
    // First 40-bit transaction
    OP_WRITE,                            //  1 bit
    CHIP_SPI_MODE32,                     //  1 bit
    MAP_SELECT_ON,                       //  1 bit
    5'b0,                                //  5 bits
    byte_swap({5'b0, user_addr[31:5]}),  // 32 bits

    // Second 40-bit transactions
    user_op,                //  1 bit
    CHIP_SPI_MODE32,        //  1 bit
    MAP_SELECT_OFF,         //  1 bit
    user_addr[4:0],         //  5 bits
    byte_swap(user_wdata)   // 32 bits
};

//=============================================================================
// This is the main state machine.  It drives the spi_clk from free_clk, and
// is clocks data out on falling edges and clocks data in on rising edges.
//
// It includes a mechanism that stretches the clock before reading the first
// bit of SMEM data.   This is to give the chip time to fetch the required
// data from SMEM before we start clocking it in.
//=============================================================================

// A shift register
reg[79:0] shift_reg;

// Keeps track of the bit number that is currently written to spi_mosi
reg[6:0] bit_number;

// Both of these count down the falling edges of free_clk
reg[2:0] sleep, suspend_counter;

// This counts down the rising edges of free_clk
reg[1:0] stretch_counter;

// Keep track of whether spi_clk is current stretched or suspended
wire spi_clk_suspended = (suspend_counter != 0);
wire spi_clk_stretched = (stretch_counter != 0);

// When this is asserted, free_clk is driven out to spi_clk
wire spi_clk_en;

// This is a count-down timer for simulating SPI transactions
reg[7:0] sim_delay;

// Are we reading from the sensor-chip's SMEM area?
reg is_smem_read;

// The state of the main finite-state-machine
reg[2:0] fsm_state;
localparam FSM_IDLE               = 0;
localparam FSM_ASSERT_CHIP_SELECT = 1;
localparam FSM_FRONT_PORCH        = 2;
localparam FSM_TRANSACT           = 3;
localparam FSM_BACK_PORCH         = 4;
localparam FSM_SIM_DELAY          = 5;
//--------------------------------------------------------------------------
always @(posedge clk) begin

    // This is a countdown timer that counts falling edges of free_clk
    if (sleep && falling_edge)
        sleep <= sleep - 1;

    // While this is non-zero, the spi_clk is suspended (i.e., is 0)
    if (suspend_counter && falling_edge)
        suspend_counter <= suspend_counter - 1;

    // While this is non-zero, the spi_clk is stretched (i.e., is 1)
    if (stretch_counter && rising_edge)
        stretch_counter <= stretch_counter - 1;

    // If we're in reset, initialize important signals
    if (resetn == 0) begin
        fsm_state       <= 0;
        stretch_counter <= 0;
        suspend_counter <= 0;
        spi_cs_n        <= RELEASE_CHIP_SELECT;
    end

    else case(fsm_state)
    
        // Wait for someone to tell us to start
        FSM_IDLE:
            case (start)
                START_WRITE:
                    begin
                        user_addr    <= addr;
                        user_wdata   <= wdata;
                        user_op      <= OP_WRITE;
                        is_smem_read <= 0;
                        fsm_state    <= FSM_ASSERT_CHIP_SELECT;
                    end

                START_READ:
                    begin
                        user_addr    <= addr;                            
                        user_wdata   <= 0;
                        user_op      <= OP_READ;
                        is_smem_read <= (addr >= 32'h8000 && addr <= 32'h107FFF);
                        fsm_state    <= FSM_ASSERT_CHIP_SELECT; 
                    end
                
                START_SIM:
                    begin
                        sim_delay <= 4;
                        fsm_state <= FSM_SIM_DELAY;
                    end                   
            endcase

        // Wait for a falling edge, then assert chip-select, then,
        // wait for a couple more falling edges
        FSM_ASSERT_CHIP_SELECT:
            if (falling_edge) begin
                spi_cs_n   <= ASSERT_CHIP_SELECT;
                sleep      <= PORCH_CYCLES;
                fsm_state  <= FSM_FRONT_PORCH;
            end

        FSM_FRONT_PORCH:
            if (sleep == 0) begin
                spi_mosi   <= transaction[79];
                shift_reg  <= (transaction << 1);
                bit_number <= 79;
                fsm_state  <= FSM_TRANSACT;
            end
        
        FSM_TRANSACT:

            // If this is the rising edge of the 8th bit (on the 2nd transaction)
            // then we need to ensure that spi_clk remains high for an extra
            // free_clk cycle. 
            if (is_smem_read && rising_edge
                             && bit_number == READ_DATA_BOUNDARY
                             && !spi_clk_stretched)
                begin
                    stretch_counter <= 1;
                end

            // On falling edges we clock out a new bit to spi_mosi
            else if (falling_edge & !spi_clk_stretched & !spi_clk_suspended) begin
                if (bit_number == TRANSACTION_BOUNDARY)
                    suspend_counter <= TRANSACTION_SUSPEND;

                if (bit_number) begin
                    spi_mosi   <= shift_reg[79];
                    shift_reg  <= (shift_reg << 1);
                    bit_number <= bit_number - 1;
                end else begin
                    spi_mosi  <= 0;
                    sleep     <= PORCH_CYCLES + 2;
                    fsm_state <= FSM_BACK_PORCH;
                end
            end

        // During the back porch, we wait for two free_clk
        // cycles after chip-select has been released because
        // we want to ensure that there are time gaps between
        // consecutive chip regster/SMEM reads/writes.
        FSM_BACK_PORCH:
            begin
                if (sleep == 2) spi_cs_n  <= RELEASE_CHIP_SELECT;
                if (sleep == 0) fsm_state <= FSM_IDLE;
            end

        // A "simulation delay" operation just wastes a few clock cycles.
        // This causes the SPI to appear to be completing transactions
        // very quickly.  This is convenient for debugging with an ILA when
        // you want to fit a lot of SPI transactions onto the debugging
        // screen.
        FSM_SIM_DELAY:
            if (sim_delay)
                sim_delay <= sim_delay - 1;
            else
                fsm_state <= FSM_IDLE;

    endcase

end

// spi_clk is enabled during state FSM_TRANSACT while we're not suspended
assign spi_clk_en = (fsm_state == FSM_TRANSACT) & !spi_clk_suspended;

// spi_clk is driven from the free-running "free_clk".  
assign spi_clk = (free_clk & spi_clk_en) | spi_clk_stretched;

// We're busy as soon as we receive a transaction request
assign busy = start || (fsm_state != FSM_IDLE);
//=============================================================================


//=============================================================================
// We clock in data on the rising edge of spi_clk
//=============================================================================
reg[31:0] data_in;
always @(posedge clk) begin
    if (spi_clk & rising_edge) begin
        if (sim_select)        
            data_in <= {data_in[30:0], sim_miso};
        else
            data_in <= {data_in[30:0], spi_miso};
    end
end

// The "rdata" port is big-endian
assign rdata = byte_swap(data_in);
//=============================================================================


//=============================================================================
// Here we divide hsi_clk by 8
// Everything in this state machine is synchonous to hsi_clk
//=============================================================================
reg hsi_clk_div_8;
reg [1:0] hsi_clk_countdown;
always @(posedge hsi_clk) begin
    if (hsi_clk_countdown)
        hsi_clk_countdown <= hsi_clk_countdown - 1;
    else begin
        hsi_clk_div_8     <= ~hsi_clk_div_8;
        hsi_clk_countdown <= 3;
    end
end
//=============================================================================


//=============================================================================
// "free_clk" is the hsi_clk_div_8, but synchronized to "clk"
//=============================================================================
xpm_cdc_single #
(
    .DEST_SYNC_FF  (4),
    .INIT_SYNC_FF  (0),
    .SIM_ASSERT_CHK(0),
    .SRC_INPUT_REG (0)
)
xpm_cdc_free_clk
(
    .src_clk (             ),
    .src_in  (hsi_clk_div_8),
    .dest_clk(clk          ),
    .dest_out(free_clk     )
);
//=============================================================================



endmodule
