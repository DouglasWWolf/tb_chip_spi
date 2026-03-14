module chip_sim # (parameter ARRAY_SIZE = 16)
(
    input clk, resetn,

    // The SPI bus
    input      spi_clk,
    input      spi_mosi,
    input      spi_cs_n,
    output reg spi_miso,

    // The HSI bus
    input       hsi_clk,
    input[31:0] hsi_data,
    input       hsi_cmd,
    input       hsi_valid
);

localparam OP_READ  = 0;
localparam OP_WRITE = 1;

// The output side of the HSI FIFO
wire[31:0] fifo_out_tdata;
wire       fifo_out_tuser;
wire       fifo_out_tvalid;

// Storage for chip registers and SMEM
reg[31:0] reg_space [0:ARRAY_SIZE-1];
reg[31:0] smem_space[0:ARRAY_SIZE-1];


//=============================================================================
// This function swaps big-endian to little-endian or vice-versa
//=============================================================================
function [31:0] byte_swap (input [31:0] value);
    byte_swap = {value[7:0], value[15:8], value[23:16], value[31:24]};
endfunction
//=============================================================================


//=============================================================================
// State machine that writes to "smem_space"
//=============================================================================
reg       smem_write_stb  [1:0];
reg[31:0] smem_write_word [1:0];
reg[31:0] smem_write_index[1:0];
//-----------------------------------------------------------------------------
always @(posedge clk) begin
    
    if (smem_write_stb[0] && smem_write_index[0] < ARRAY_SIZE)
        smem_space[smem_write_index[0]] <= smem_write_word[0];
    
    if (smem_write_stb[1] && smem_write_index[1] < ARRAY_SIZE)
        smem_space[smem_write_index[1]] <= smem_write_word[1];

end
//=============================================================================

//=============================================================================
// This writes data arriving from the HSI bus into SMEM.  When data pours 
// out of the FIFO:
//
//    fifo_tuser = HSI "command" flag 
//    fifo_tdata = HSI data
//=============================================================================
reg[29:0] hsi_smem_index;
//-----------------------------------------------------------------------------
always @(posedge clk) begin
    
    // This strobes high for a single cycle at a time
    smem_write_stb[1] <= 0;

    if (fifo_out_tvalid) begin
        if (fifo_out_tuser)
            hsi_smem_index <= (fifo_out_tdata * 256) / 4;
        else begin
            smem_write_index[1] <= hsi_smem_index;
            smem_write_word [1] <= fifo_out_tdata;
            smem_write_stb  [1] <= 1;
            hsi_smem_index      <= hsi_smem_index + 1;
        end

    end
end
//=============================================================================


wire rising_edge, falling_edge;

reg [7:0] command;
reg[31:0] wdata;
reg[31:0] rdata;
reg[31:0] upper27;

wire      op         = command[7];
wire      mode       = command[6];
wire      map_select = command[5];
wire[4:0] lower5     = command[4:0];

// This is the chip-space address we're going to read or write
wire[31:0] address = {upper27[26:0], lower5};

// In the case of an SMEM access, this is the offset into SMEM
wire[31:0] smem_offset = address - 32'h8000;

// Determine whether "address" is in SMEM
wire is_smem = (address >= 32'h8000 && address <= 32'h107FFF);

// The array index that corresponds to "address"
wire[29:0] addr_index = (is_smem) ? smem_offset[31:2] : address[31:2];

//=============================================================================
// Count falling edges 
//=============================================================================
reg [6:0] bit_count;
//-----------------------------------------------------------------------------
always @(posedge clk) begin
    if (resetn == 0 || spi_cs_n == 1)
        bit_count <= 1;
    else if (falling_edge) 
        bit_count <= (bit_count < 40) ? bit_count + 1 : 1;
end
//=============================================================================


//=============================================================================
// Clock bits into either the "command" register or the "wdata" register
//=============================================================================
always @(posedge clk) begin

    if (resetn == 0 || spi_cs_n == 1) begin
        command <= 0;
    end

    else if (rising_edge) begin
        if (bit_count <= 8) begin
            command <= {command[6:0], spi_mosi};
        end else begin
            wdata   <= { wdata[30:0], spi_mosi};
        end
    end
end
//=============================================================================


//=============================================================================
// On the falling edge of the 40th bit, handle write operations
//=============================================================================
always @(posedge clk) begin
    
    smem_write_stb[0] <= 0;
    if (falling_edge && bit_count == 40 && op == OP_WRITE) begin
        if (map_select)
            upper27 <= byte_swap(wdata);
        else if (addr_index < ARRAY_SIZE) begin
            if (is_smem) begin
                smem_write_index[0] <= addr_index;
                smem_write_word [0] <= byte_swap(wdata);
                smem_write_stb  [0] <= 1;
            end else
                reg_space[addr_index] <= byte_swap(wdata);
        end
    end
end
//=============================================================================

//=============================================================================
// During a read operation, on the falling edge of the 8th cycle (the command   
// byte), read the desired data
//=============================================================================
always @(posedge clk) begin

    if (bit_count < 8)
        rdata <= 0;

    if (falling_edge && op == OP_READ) begin
        if (bit_count == 8) begin
            if (map_select)
                rdata <= 0;
            else if (is_smem) begin
                rdata <= byte_swap(smem_space[addr_index]);
            end else begin
                rdata <= byte_swap(reg_space[addr_index]);
            end
        end

        // When we're clocking out data, shift rdata left by 1 bit
        if (bit_count > 8) rdata <= {rdata[30:0], 1'b0};

    end
end
//=============================================================================

//=============================================================================
// During a read operation, spi_miso is always the top-bit of rdata
//=============================================================================
always @* begin
    if (bit_count >= 8 && op == OP_READ && map_select == 0)
        spi_miso = rdata[31];
    else
        spi_miso = 0; 
end
//=============================================================================



//=============================================================================
// Keep track of the spi_clk edges 
//=============================================================================
reg prior_spi_clk;
always @(posedge clk) prior_spi_clk <= spi_clk;
assign rising_edge  = (prior_spi_clk == 0) & (spi_clk == 1);
assign falling_edge = (prior_spi_clk == 1) & (spi_clk == 0);
//=============================================================================


//=============================================================================
// This FIFO acts as a CDC to move incoming HSI data from the "hsi_clk" domain 
// to the "clk" domain
//
// TUSER is asserted on the HSI command word that precedes the SMEM row data
//=============================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH      (16),
   .TDATA_WIDTH     (32),
   .TUSER_WIDTH     ( 1),
   .FIFO_MEMORY_TYPE("auto"),
   .PACKET_FIFO     ("false"),
   .USE_ADV_FEATURES("0000"),
   .CDC_SYNC_STAGES (3),
   .CLOCKING_MODE   ("independent_clock")
)
hsi_fifo
(
    // Clock and reset
   .s_aclk   (hsi_clk),
   .m_aclk   (clk    ),
   .s_aresetn(resetn ),

    // This input bus of the FIFO
   .s_axis_tdata (hsi_data ),
   .s_axis_tuser (hsi_cmd  ),
   .s_axis_tvalid(hsi_valid),
   .s_axis_tready(         ),

    // The output bus of the FIFO
   .m_axis_tdata (fifo_out_tdata ),
   .m_axis_tuser (fifo_out_tuser ),
   .m_axis_tvalid(fifo_out_tvalid),
   .m_axis_tready(1              ),

    // Unused input stream signals
   .s_axis_tlast(),
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tkeep(),

    // Unused output stream signals
   .m_axis_tlast(),
   .m_axis_tdest(),
   .m_axis_tid  (),
   .m_axis_tstrb(),
   .m_axis_tkeep(),

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(),
   .dbiterr_axis(),
   .prog_empty_axis(),
   .prog_full_axis(),
   .rd_data_count_axis(),
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


endmodule
