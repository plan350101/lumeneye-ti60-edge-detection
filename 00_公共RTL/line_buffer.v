// ============================================================
//  line_buffer.v
//  Simple dual-clock (simple dual-port) line memory.
//
//  Written in the camera PCLK domain, read back in the display
//  clock domain.  Efinity infers a block RAM for this pattern.
//
//  640 x 16 bit = 10240 bit, which fits in one 10 kbit BRAM block.
// ============================================================

module line_buffer #(
    parameter WIDTH = 640,
    parameter DW    = 16,
    parameter AW    = 10
)(
    input  wire          wr_clk,
    input  wire          wr_en,
    input  wire [AW-1:0] wr_addr,
    input  wire [DW-1:0] wr_data,

    input  wire          rd_clk,
    input  wire          rd_en,
    input  wire [AW-1:0] rd_addr,
    output reg  [DW-1:0] rd_data
);

    (* ramstyle = "block" *) reg [DW-1:0] mem [0:WIDTH-1];

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk) begin
        if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
