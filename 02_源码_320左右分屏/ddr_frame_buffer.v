// ============================================================================
//  ddr_frame_buffer.v
//
//  把板载 DDR3 包装成一个"帧缓存"，用来替换原来占 150 块 RAM10 的两块 BRAM
//  乒乓缓存（u_fb0 / u_fb1）。
//
//  ---------------------------------------------------------------------------
//  为什么需要这个模块
//  ---------------------------------------------------------------------------
//  原设计：320x240 @8bpp 的显示帧缓存 = 2 块 BRAM 乒乓
//            2 x 76800 x 8bit = 1 228 800 bit / 8192 bit 每块 = 150 块 RAM10
//          加上时域降噪历史 / 边缘历史 / 行缓存 / ROM，RAM10 用到 252 / 256，
//          只剩 4 块，任何新功能都加不进去。
//  改成 DDR 后：两块 BRAM 全部省下，只留 1 行行缓冲（1 块 RAM10）。
//
//  ---------------------------------------------------------------------------
//  接口为什么和 BRAM 版 frame_buffer 不一样
//  ---------------------------------------------------------------------------
//  BRAM 版是**地址式**：给 raddr 就回 rdata，可以任意随机寻址。
//  DDR 侧（axi4_ctrl）是**帧流式**：只能"顺序写一帧"、"顺序读一帧"，
//  不能随机寻址，也不能重复读同一个数据。
//
//  所以这里的处理是：
//    * 写口：保留 we / waddr / wdata 三个信号，**只要求 waddr 顺序递增**
//            （本工程写侧本来就是 srow*320+scol 顺序递增，天然满足）。
//    * 读口：改成"rd_col 选列 + rd_fetch 请求取数"。DDR 的数据顺序流进来，
//            每个数据在 rd_fetch 的那一拍写入行缓冲 lbuf[rd_col]，
//            同时直通到 rdata（保证与原 BRAM 同样的 **1 拍读延迟**）。
//            显示侧第二次用同一行（垂直 2x 放大）时不再请求 DDR，
//            直接从行缓冲读 —— 这就是"不能重复读"的解法。
//    详细的行/列时序判断放在顶层做（它本来就有 dcol / dy_d / SPLIT），
//    本模块只负责数据通路，保持简单。
//
//  ---------------------------------------------------------------------------
//  ★ 关键参数：C_BURST_LEN 必须让 C_ADDR_INC 整除一帧字节数
//  ---------------------------------------------------------------------------
//    C_ADDR_INC = C_BURST_LEN * C_DATA_LEN/8 = C_BURST_LEN * 16
//    一帧字节数 = STORE_W * STORE_H = 76800
//    C_BURST_LEN = 64  ->  C_ADDR_INC = 1024  ->  76800 / 1024 = 75  ← 整除 ✓
//
//    为什么必须整除：axi4_ctrl 的写侧固定按 C_BURST_LEN 个 beat 切一次 burst
//    （axi_wlast = rc_burst >= C_BURST_LEN-1），如果一帧字节数不是 C_ADDR_INC
//    的整数倍，最后一笔 burst 会越过帧尾，读侧就会多读出无效字节，
//    进而把行缓冲的对齐彻底打乱（画面撕裂/错行）。
//
//  ---------------------------------------------------------------------------
//  资源账本
//  ---------------------------------------------------------------------------
//    省下：2 x 76800 x 8bit = 150 块 RAM10
//    新增：DdrCtrl 27 + W0_FIFO_8 8 + R0_FIFO_8 8 = 43 块（官方 baseline 实测）
//          + 行缓冲 320 x 8bit = 1 块
//    净效果：252 -> 约 146 块（省掉约 106 块），后面加功能才有地方放。
// ============================================================================

module ddr_frame_buffer #(
    parameter STORE_W       = 320,
    parameter STORE_H       = 240,
    parameter DW            = 8,                   // 每像素位深
    parameter AW            = 17,                  // STORE_W*STORE_H = 76800 < 2^17
    parameter C_ID_LEN      = 4,
    parameter C_BURST_LEN   = 64,                  // 见上面"关键参数"
    parameter C_RD_END_ADDR = STORE_W * STORE_H    // 76800，一帧字节数
)(
    // ================= 写口：摄像头像素域（cmos_pclk） =================
    input  wire                wr_clk,
    input  wire                wr_rst_n,
    input  wire                we,          // 写使能（一个有效像素一次）
    input  wire [AW-1:0]       waddr,       // 顺序递增的存储地址
    input  wire [DW-1:0]       wdata,
    // ★ 2026-10-18（接手后）: 摄像头帧边界（cmos_pclk 域，1 拍脉冲）。
    //   写侧 EOF 改由它产生，而不是由 waddr 回绕产生 —— 见下方 EOF 生成块的说明。
    input  wire                wr_frame_end,

    // ================= 读口：显示像素域（clk_pixel） =================
    input  wire                rd_clk,
    input  wire                rd_rst_n,
    input  wire                rd_frame,    // 显示帧起始，1 clk 脉冲（hcnt==0 && vcnt==0）
    input  wire [8:0]          rd_col,      // 0..STORE_W-1，当前要显示的存储列
    input  wire                rd_fetch,    // 本拍需要从 DDR 取一个新数据（顶层算好）
    output reg  [DW-1:0]       rdata,

    // ================= DDR AXI4 主口（接 DdrCtrl + AXI4_AWARMux） =================
    input  wire                axi_clk,     // = clk_sys 96 MHz
    input  wire                axi_reset,   // 高有效

    output wire [C_ID_LEN-1:0] axi_awid,
    output wire [31:0]         axi_awaddr,
    output wire [7:0]          axi_awlen,
    output wire [2:0]          axi_awsize,
    output wire [1:0]          axi_awburst,
    output wire                axi_awlock,
    output wire [3:0]          axi_awcache,
    output wire [2:0]          axi_awprot,
    output wire [3:0]          axi_awqos,
    output wire                axi_awvalid,
    input  wire                axi_awready,

    output wire [127:0]        axi_wdata,
    output wire [15:0]         axi_wstrb,
    output wire                axi_wlast,
    output wire                axi_wvalid,
    input  wire                axi_wready,

    input  wire [C_ID_LEN-1:0] axi_bid,
    input  wire [1:0]          axi_bresp,
    input  wire                axi_bvalid,
    output wire                axi_bready,

    output wire [C_ID_LEN-1:0] axi_arid,
    output wire [31:0]         axi_araddr,
    output wire [7:0]          axi_arlen,
    output wire [2:0]          axi_arsize,
    output wire [1:0]          axi_arburst,
    output wire                axi_arlock,
    output wire [3:0]          axi_arcache,
    output wire [2:0]          axi_arprot,
    output wire [3:0]          axi_arqos,
    output wire                axi_arvalid,
    input  wire                axi_arready,

    input  wire [C_ID_LEN-1:0] axi_rid,
    input  wire [127:0]        axi_rdata,
    input  wire [1:0]          axi_rresp,
    input  wire                axi_rlast,
    input  wire                axi_rvalid,
    output wire                axi_rready
);

    // ========================================================================
    //  1) 写侧：从顺序地址推出"帧边界"，生成 wframe_vsync
    // ========================================================================
    //  axi4_ctrl 要求 wframe_vsync 的**下降沿**代表"一帧写完了，交出去"。
    //  写地址每帧从 0 顺序涨到 STORE_W*STORE_H-1 附近，下一帧重新从 0 开始，
    //  所以"本拍地址比上一拍小"就是帧边界 —— 不需要额外的握手信号。
    //
    //  ⚠️⚠️⚠️ 2026-10-16 19:45 —— 这里踩过一个**致命坑**，是"画面横向有内容、
    //     纵向完全不相关"这个怪象的根因，务必不要改回去：
    //
    //     axi4_ctrl 内部用 **axi_clk(=clk_sys, 96 MHz)** 采样 wframe_vsync，
    //     而本模块工作在 **wr_clk(=cmos_pclk, 也≈96 MHz)** —— 两者同频，
    //     但**来源完全独立**（一个来自 hdmi_pll，一个来自摄像头 PCLK 引脚）。
    //     原来只把 vsync 拉低 **1 个 wr_clk（≈10.4 ns）**，宽度恰好等于目的
    //     时钟周期 ⇒ 采样沿可以**长期落在脉冲之外**，下降沿被整段整段地漏掉。
    //
    //     漏掉的后果（连锁）：
    //       ① axi4_ctrl 收不到 EOF ⇒ rc_wframe_index 不换 buffer，
    //          r_wframe_index_last 永远停在初值 0；
    //       ② 读侧每次 rframe_vsync 的 flush 把 rc_rframe_index 指向 0，
    //          也就是**写侧正在写的那一块**；
    //       ③ ⇒ 读写同块、且两边各自扫描 ⇒ **整屏撕裂**：
    //          每一行的 319 个字节仍然是完整的一行（所以横向相关高达 ~0.9），
    //          但行与行来自不同的扫描位置（所以纵向相关≈0，行均值乱跳）。
    //
    //     官方 example_top.v 是直接把**传感器的 VSYNC（宽达若干行）**接进来的，
    //     天然没有这个窄脉冲问题。
    //
    //     修法：把低电平**展宽到 512 拍（≈5.3 µs）**——是目的时钟周期的 512 倍，
    //     任何同步器都不可能漏采；又只有一帧(33 ms)的 0.016%，期间只会顺带
    //     多冲掉几十字节（≈0.2 个存储行，肉眼无感）。
    // ========================================================================
    localparam [11:0] WVS_HOLD = 12'd512;      // vsync 低电平保持拍数（≈5.3 µs）

    reg  [AW-1:0] waddr_d;
    reg  [11:0]   wvs_cnt;
    reg           wframe_vsync;
    wire          w_addr_wrap = we && (waddr < waddr_d);   // 保留作参考（不再用于 EOF）

    //  ★★ 2026-10-18（接手后）：EOF 生成从"写地址回绕"改成"摄像头帧边界帧脉冲"。
    //    原因（上板实测）：相机 30fps 恒定、显示 60Hz 恒定，但整幅图逐帧左右抖。
    //    根因是写地址回绕恰好发生在**新帧第一个写字节**那一拍；wframe_vsync 经
    //    clk_sys 两级同步有 2~3 拍延迟，这段时间里新帧最前面的 0~N 个字节已经
    //    进了 W0_FIFO，EOF flush 把它们写进**旧 buffer 尾部**，于是新 buffer
    //    从第 N 个字节开始 → 整幅图水平偏移 N 个存储像素（每帧 N 不同 ⇒ 抖动）。
    //    改用 row 计数回绕产生的 frame_end（它在新帧第一批写字节之前约 2 行，
    //    即 ~40 µs）：EOF/flush 在新帧数据到来之前完成，新 buffer 从字节 0 开始。
    //    旧地址回绕逻辑保留但不再参与 EOF（不影响既有修复）。

    //  ⚠️⚠️⚠️ 2026-10-17 16:5x —— 第二个致命坑（比上面那个更隐蔽）：
    //
    //     顶层 waddr = srow*320 + scol，其中 scol = (pcol-2)>>1。
    //     **行消隐期间 pcol 被清 0**，于是 scol = (0-2)>>1 在 9 位里 = **511**
    //     （不是 0！）。所以地址序列是：
    //          … 320r+318   （本行最后一个写字节）
    //          … 320r+511   （消隐期间，只变地址不写）
    //          … 320(r+1)+0 （下一行第一个写字节）  <- 比上一拍小！
    //     原来 `waddr_d <= waddr` **每拍都更新**（不管 we），于是下一行第一拍
    //     必然满足 waddr < waddr_d ⇒ **每一行都误判成"帧结束"**。
    //
    //     后果链：每行发一次 EOF → rc_w_ptr 每行归零、rc_wframe_index 不换块
    //     ⇒ 整帧只写进 buffer 最前面约 1 KB，后面全是 DDR **上电随机内容**。
    //     我们之前把 1:1 窗口取在 30 KB 处，读到的正是那片随机噪声
    //     （256 个值全出现、无任何结构），而"横向有内容"是因为窗口偶尔
    //     落在写过的那 1 KB 上。第 33 轮把 vsync 展宽 512 拍之所以完全无效，
    //     就是因为这个脉冲**根本没在该来的时候来**。
    //
    //     修法：waddr_d 只在**真正写出一个字节**时更新 ⇒ 比较的是
    //     "相邻两个已写地址"，行内严格递增，只有真正的帧回绕才会变小。
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            waddr_d      <= {AW{1'b0}};
            wvs_cnt      <= 12'd0;
            wframe_vsync <= 1'b1;
        end else begin
            if (we) waddr_d <= waddr;          // ★ 只在写字节时更新（消隐不算）
            if (wr_frame_end)          wvs_cnt <= WVS_HOLD;
            else if (wvs_cnt != 12'd0) wvs_cnt <= wvs_cnt - 12'd1;
            // frame_end 当拍 + 之后 WVS_HOLD 拍都保持低电平 ⇒ 宽脉冲，CDC 可靠
            wframe_vsync <= (wr_frame_end || (wvs_cnt != 12'd0)) ? 1'b0 : 1'b1;
        end
    end

    // ========================================================================
    //  2) 读侧：rframe_vsync（显示帧起始处产生下降沿 -> 复位 RFIFO、换读指针）
    // ========================================================================
    //  axi4_ctrl 的 rframe_vsync 下降沿会复位读侧 FIFO，并让读指针切到
    //  "最新写完的那一块 buffer"，同时把 DDR 读地址清零重新从头读。
    //  这正好等价于原设计"读侧只在显示帧起始换 bank"的消撕裂语义。
    reg rframe_vsync_r;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n)     rframe_vsync_r <= 1'b1;
        else if (rd_frame) rframe_vsync_r <= 1'b0;   // 帧起始 -> 拉低 -> 下一拍出现下降沿
        else               rframe_vsync_r <= 1'b1;
    end

    // ========================================================================
    //  3) 行缓冲 + 读数据通路
    // ========================================================================
    //  lbuf 是 1 行（STORE_W 个像素），只在"第一次用这一行"时被填。
    //  显示侧同一行的第二次使用（垂直 2x 放大）不再请求 DDR，直接读 lbuf。
    (* ramstyle = "block" *) reg [DW-1:0] lbuf [0:STORE_W-1];

    wire [DW-1:0] rframe_data;      // axi4_ctrl 顺序吐出来的像素

    always @(posedge rd_clk) begin
        if (rd_fetch) lbuf[rd_col] <= rframe_data;
    end

    //  ★ 读延迟必须与原 BRAM 版一致（都是 1 拍）：
    //    rd_fetch 那一拍照样 1 拍延迟输出，否则整条显示通路的像素对齐会错一格。
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) rdata <= {DW{1'b0}};
        else if (rd_fetch) rdata <= rframe_data;
        else               rdata <= lbuf[rd_col];
    end

    // ========================================================================
    //  4) axi4_ctrl 例化（官方 DDR3 软核的 AXI4 帧流前端）
    // ========================================================================
    axi4_ctrl #(
        .C_ID_LEN      (C_ID_LEN),
        .C_BURST_LEN   (C_BURST_LEN),
        .C_RD_END_ADDR (C_RD_END_ADDR),
        .C_W_WIDTH     (DW),        // 8  -> 内部例化 W0_FIFO_8
        .C_R_WIDTH     (DW)         // 8  -> 内部例化 R0_FIFO_8
    ) u_axi4_ctrl (
        .axi_clk        (axi_clk),
        .axi_reset      (axi_reset),

        .axi_awid       (axi_awid),
        .axi_awaddr     (axi_awaddr),
        .axi_awlen      (axi_awlen),
        .axi_awsize     (axi_awsize),
        .axi_awburst    (axi_awburst),
        .axi_awlock     (axi_awlock),
        .axi_awcache    (axi_awcache),
        .axi_awprot     (axi_awprot),
        .axi_awqos      (axi_awqos),
        .axi_awvalid    (axi_awvalid),
        .axi_awready    (axi_awready),

        .axi_wdata      (axi_wdata),
        .axi_wstrb      (axi_wstrb),
        .axi_wlast      (axi_wlast),
        .axi_wvalid     (axi_wvalid),
        .axi_wready     (axi_wready),

        .axi_bid        (axi_bid),
        .axi_bresp      (axi_bresp),
        .axi_bvalid     (axi_bvalid),
        .axi_bready     (axi_bready),

        .axi_arid       (axi_arid),
        .axi_araddr     (axi_araddr),
        .axi_arlen      (axi_arlen),
        .axi_arsize     (axi_arsize),
        .axi_arburst    (axi_arburst),
        .axi_arlock     (axi_arlock),
        .axi_arcache    (axi_arcache),
        .axi_arprot     (axi_arprot),
        .axi_arqos      (axi_arqos),
        .axi_arvalid    (axi_arvalid),
        .axi_arready    (axi_arready),

        .axi_rid        (axi_rid),
        .axi_rdata      (axi_rdata),
        .axi_rresp      (axi_rresp),
        .axi_rlast      (axi_rlast),
        .axi_rvalid     (axi_rvalid),
        .axi_rready     (axi_rready),

        .wframe_pclk    (wr_clk),
        .wframe_vsync   (wframe_vsync),
        .wframe_data_en (we),
        .wframe_data    (wdata),

        .rframe_pclk    (rd_clk),
        .rframe_vsync   (rframe_vsync_r),
        .rframe_data_en (rd_fetch),
        .rframe_data    (rframe_data),

        .tp_o           ()
    );

endmodule
