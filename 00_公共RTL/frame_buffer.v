// ============================================================
//  frame_buffer.v
//  双时钟（简单双口）帧缓存。
//
//  写口在摄像头像素时钟域（cmos_pclk），读口在显示像素时钟域
//  （clk_pixel），两个域完全异步。
//
//  ★ Ti60F225 的 BRAM 账本（实测，见 ramcalc/）
//      器件一共 256 块 EFX_RAM10；
//      每块**有效**容量 = 8192 bit（不是名义 10240 —— PT 只按
//      8bit 宽度打包，宽度不足 8 的会浪费掉高位）；
//      可用总量 = 256 x 8192 = 2 097 152 bit = 256 KB。
//    所以 640x480 的整帧：
//      8bpp -> 300 KB -> 300 块  ✗ 装不下
//      7bpp -> 263 块            ✗
//      6bpp -> 225 块            ~ 太挤
//      >>> 5bpp -> 188 块        ✓ 选它 (32 级灰度)
//      4bpp -> 150 块            ✓
//  这里 5bpp，即灰度只存 Y[7:3]。显示时再按 {g, g[4:2]} 抬回 8bit。
// ============================================================

module frame_buffer #(
    parameter DEPTH = 640*480,
    parameter DW    = 5,
    parameter AW    = 19
)(
    // ---- 写口：cmos_pclk 域 ----
    input  wire          wr_clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,

    // ---- 读口：clk_pixel 域 ----
    input  wire          rd_clk,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);

    (* ramstyle = "block" *) reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge wr_clk) begin
        if (we) mem[waddr] <= wdata;
    end

    always @(posedge rd_clk) begin
        if (re) rdata <= mem[raddr];
    end

endmodule
