//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 18-Feb-26  DWW     1  Initial creation
//====================================================================================

/*
    Provides an AXI register interface to the chip_spi module
*/


module chip_spi_ctl # (parameter AW=8)
(
    input clk, resetn,

    output [31:0] addr,
    output [31:0] wdata,
    output [ 1:0] start_stb,
    input         busy,
    input  [31:0] rdata,
    
    //================== This is an AXI4-Lite slave interface ==================
        
    // "Specify write address"              -- Master --    -- Slave --
    input[AW-1:0]                           S_AXI_AWADDR,   
    input                                   S_AXI_AWVALID,  
    input[   2:0]                           S_AXI_AWPROT,
    output                                                  S_AXI_AWREADY,


    // "Write Data"                         -- Master --    -- Slave --
    input[31:0]                             S_AXI_WDATA,      
    input                                   S_AXI_WVALID,
    input[ 3:0]                             S_AXI_WSTRB,
    output                                                  S_AXI_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    output[1:0]                                             S_AXI_BRESP,
    output                                                  S_AXI_BVALID,
    input                                   S_AXI_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    input[AW-1:0]                           S_AXI_ARADDR,     
    input[   2:0]                           S_AXI_ARPROT,     
    input                                   S_AXI_ARVALID,
    output                                                  S_AXI_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    output[31:0]                                            S_AXI_RDATA,
    output                                                  S_AXI_RVALID,
    output[ 1:0]                                            S_AXI_RRESP,
    input                                   S_AXI_RREADY
    //==========================================================================
);  

//=========================  AXI Register Map  =============================
localparam REG_ADDR = 0;
localparam REG_DATA = 1;
//==========================================================================


//==========================================================================
// We'll communicate with the AXI4-Lite Slave core with these signals.
//==========================================================================
// AXI Slave Handler Interface for write requests
wire[  31:0]  ashi_windx;     // Input   Write register-index
wire[AW-1:0]  ashi_waddr;     // Input:  Write-address
wire[  31:0]  ashi_wdata;     // Input:  Write-data
wire          ashi_write;     // Input:  1 = Handle a write request
reg [   1:0]  ashi_wresp;     // Output: Write-response (OKAY, DECERR, SLVERR)
wire          ashi_widle;     // Output: 1 = Write state machine is idle

// AXI Slave Handler Interface for read requests
wire[  31:0]  ashi_rindx;     // Input   Read register-index
wire[AW-1:0]  ashi_raddr;     // Input:  Read-address
wire          ashi_read;      // Input:  1 = Handle a read request
reg [  31:0]  ashi_rdata;     // Output: Read data
reg [   1:0]  ashi_rresp;     // Output: Read-response (OKAY, DECERR, SLVERR);
wire          ashi_ridle;     // Output: 1 = Read state machine is idle
//==========================================================================


//==========================================================================
// These are how we communicate with the chip_spi interface
//==========================================================================
reg [31:0] io_raddr;
reg [31:0] io_waddr;
reg [31:0] io_wdata;
reg        io_write_stb;
reg        io_read_stb;
wire[31:0] io_rdata;
wire       io_read_busy;
wire       io_write_busy;
//==========================================================================


// The state of the state-machines that handle AXI4-Lite read and AXI4-Lite write
reg ashi_write_state, ashi_read_state;

// The AXI4 slave state machines are idle when in state 0 and their "start" signals are low
assign ashi_widle = (ashi_write == 0) && (ashi_write_state == 0);
assign ashi_ridle = (ashi_read  == 0) && (ashi_read_state  == 0);
   
// These are the valid values for ashi_rresp and ashi_wresp
localparam OKAY   = 0;
localparam SLVERR = 2;
localparam DECERR = 3;

// This is where we store the address we want to read or write
reg [31:0] address;
//==========================================================================
// This state machine handles AXI4-Lite write requests
//==========================================================================
always @(posedge clk) begin

    io_write_stb <= 0;

    // If we're in reset, initialize important registers
    if (resetn == 0) begin
        ashi_write_state  <= 0;
    end
    
    // Otherwise, we're not in reset...
    else case (ashi_write_state)
        
        // If an AXI write-request has occured...
        0:  if (ashi_write) begin
       
                // Assume for the moment that the result will be OKAY
                ashi_wresp <= OKAY;              
            
                // ashi_windex = index of register to be written
                case (ashi_windx)
               
                    REG_ADDR:
                        address <= ashi_wdata;

                    REG_DATA:
                        begin
                            io_waddr         <= address;
                            io_wdata         <= ashi_wdata;
                            io_write_stb     <= 1;
                            ashi_write_state <= 1;
                        end

                    // Writes to any other register are a decode-error
                    default: ashi_wresp <= DECERR;
                endcase
            end

        // Here we wait for an SPI write operation to complete
        1:  if (!io_write_busy) begin
                ashi_write_state <= 0;
            end

    endcase
end
//==========================================================================



//==========================================================================
// World's simplest state machine for handling AXI4-Lite read requests
//==========================================================================
always @(posedge clk) begin

    // This will strobe high for one clock-cycle at a time
    io_read_stb <= 0;

    // If we're in reset, initialize important registers
    if (resetn == 0) begin
        ashi_read_state <= 0;
    end
    
    else case(ashi_read_state)

        0:  if (ashi_read) begin
   
                // Assume for the moment that the result will be OKAY
                ashi_rresp <= OKAY;              
        
                // ashi_rindex = index of register to be read
                case (ashi_rindx)
            
                    // Allow a read from any valid register                
                    REG_ADDR:
                        ashi_rdata <= address;

                    REG_DATA:
                        begin
                            io_raddr        <= address;
                            io_read_stb     <= 1;
                            ashi_read_state <= 1;
                        end

                    // Reads of any other register are a decode-error
                    default: ashi_rresp <= DECERR;
                endcase
            end

        1:  if (!io_read_busy) begin
                ashi_rdata      <= io_rdata;
                ashi_read_state <= 0;
            end
    endcase
end
//==========================================================================



//==========================================================================
// This connects us to an AXI4-Lite slave core
//==========================================================================
axi4_lite_slave#(.AW(AW)) i_axi4lite_slave
(
    .clk            (clk),
    .resetn         (resetn),
    
    // AXI AW channel
    .AXI_AWADDR     (S_AXI_AWADDR),
    .AXI_AWPROT     (S_AXI_AWPROT),
    .AXI_AWVALID    (S_AXI_AWVALID),   
    .AXI_AWREADY    (S_AXI_AWREADY),
    
    // AXI W channel
    .AXI_WDATA      (S_AXI_WDATA),
    .AXI_WVALID     (S_AXI_WVALID),
    .AXI_WSTRB      (S_AXI_WSTRB),
    .AXI_WREADY     (S_AXI_WREADY),

    // AXI B channel
    .AXI_BRESP      (S_AXI_BRESP),
    .AXI_BVALID     (S_AXI_BVALID),
    .AXI_BREADY     (S_AXI_BREADY),

    // AXI AR channel
    .AXI_ARADDR     (S_AXI_ARADDR), 
    .AXI_ARPROT     (S_AXI_ARPROT),
    .AXI_ARVALID    (S_AXI_ARVALID),
    .AXI_ARREADY    (S_AXI_ARREADY),

    // AXI R channel
    .AXI_RDATA      (S_AXI_RDATA),
    .AXI_RVALID     (S_AXI_RVALID),
    .AXI_RRESP      (S_AXI_RRESP),
    .AXI_RREADY     (S_AXI_RREADY),

    // ASHI write-request registers
    .ASHI_WADDR     (ashi_waddr),
    .ASHI_WINDX     (ashi_windx),
    .ASHI_WDATA     (ashi_wdata),
    .ASHI_WRITE     (ashi_write),
    .ASHI_WRESP     (ashi_wresp),
    .ASHI_WIDLE     (ashi_widle),

    // ASHI read registers
    .ASHI_RADDR     (ashi_raddr),
    .ASHI_RINDX     (ashi_rindx),
    .ASHI_RDATA     (ashi_rdata),
    .ASHI_READ      (ashi_read ),
    .ASHI_RRESP     (ashi_rresp),
    .ASHI_RIDLE     (ashi_ridle)
);
//==========================================================================


//==========================================================================
// This is an interface to the chip_spi module
//==========================================================================
chip_spi_if i_chip_spi_if
(
    .clk            (clk),
    .resetn         (resetn),

    // Client-side interface for write-transactions
    .io_waddr       (io_waddr),
    .io_wdata       (io_wdata),
    .io_write_stb   (io_write_stb),
    .io_write_busy  (io_write_busy),

    // Client-side interface for read transactions
    .io_raddr       (io_raddr),
    .io_read_stb    (io_read_stb),
    .io_rdata       (io_rdata),
    .io_read_busy   (io_read_busy),

    // Interface to the chip_spi module
    .spi_addr       (addr),
    .spi_wdata      (wdata),
    .spi_start_stb  (start_stb),
    .spi_busy       (busy),
    .spi_rdata      (rdata)
);
//==========================================================================


endmodule
