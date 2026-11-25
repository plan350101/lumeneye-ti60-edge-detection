// ============================================================
//  dvp_capture.v
//  OV5640 DVP (Digital Video Port) capture, 8-bit RGB565 mode.
//
//  In RGB565 the sensor sends TWO bytes per pixel on D[7:0]:
//      byte0 (high) = {R[4:0], G[5:3]}
//      byte1 (low)  = {G[2:0], B[4:0]}
//  so the assembled pixel is
//      pixel[15:0]  = {R[4:0], G[5:3], G[2:0], B[4:0]}
//
//  VSYNC : frame sync  (polarity selectable, default active-high)
//  HREF  : line valid  (active-high)
//  PCLK  : data is valid on the PCLK rising edge
//
//  Everything here runs in the camera PCLK domain.
// ============================================================

module dvp_capture #(
    parameter IMG_W     = 640,
    parameter IMG_H     = 480,
    parameter VSYNC_POL = 1'b1      // 1 = active high
)(
    input  wire        pclk,
    input  wire        rst_n,       // synchronous reset, pclk domain
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  data,

    output reg         de,          // 1 = pixel valid this cycle
    output reg  [15:0] pixel,       // assembled RGB565
    output reg  [9:0]  x_cnt,       // column 0..IMG_W-1
    output reg  [9:0]  y_cnt,       // row 0..IMG_H-1

    // status (useful for LED bring-up)
    output reg         vs_toggle,   // flips once per frame
    output reg  [15:0] line_len     // last HREF byte count (debug)
);

    // ---- input synchronizers (2FF) ----
    reg vs_meta, vs_sync;
    reg hr_meta, hr_sync;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vs_meta <= 1'b0; vs_sync <= 1'b0;
            hr_meta <= 1'b0; hr_sync <= 1'b0;
        end else begin
            vs_meta <= vsync;
            vs_sync <= vs_meta;
            hr_meta <= href;
            hr_sync <= hr_meta;
        end
    end

    wire vs_act = VSYNC_POL ? vs_sync  : ~vs_sync;
    reg  vs_act_d;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) vs_act_d <= 1'b0;
        else        vs_act_d <= vs_act;
    end

    wire vs_rise = vs_act & ~vs_act_d;      // start of frame

    // ---- HREF edges ----
    reg hr_d;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) hr_d <= 1'b0;
        else        hr_d <= hr_sync;
    end

    wire href_hi = hr_sync & ~hr_d;         // rising edge of HREF
    wire href_lo = ~hr_sync & hr_d;         // falling edge of HREF

    // ============================================================
    //  ★★ 第 24 轮：真实帧边界检测（HREF 长间隔 = 新帧）★★
    // ------------------------------------------------------------
    // VSYNC 引脚不可信（第 9c 轮起），旧方案用"y_cnt 数到 479 回绕"猜帧
    // 边界，但 cap_y 的 0 点与传感器真实帧起始之间有**随机偏移**（上电
    // 相位决定）。后果：上一帧底部 K 行被写进新帧缓存顶部 → 屏幕顶部
    // 出现一条"重复景物带"（用户拍到的：上一帧的桌沿/线缆出现在画面
    // 最顶上）；偏移量随上电变化 → 带时宽时窄、反复出现。
    //
    // 可靠的帧边界特征：帧与帧之间 HREF 停止一大段时间（帧尾消隐
    // ≈ VTS-480 行 ≈ 25ms），而行间隔只有 ≈70us —— 相差三个数量级。
    // 数"相邻两个 HREF 上升沿的间隔"，超过阈值即判定新帧开始，
    // 把 y_cnt 清零 —— 行计数从此永远与真实帧起点对齐。
    // 若传感器没有帧尾大消隐（阈值永不触发），行为退化为旧的纯回绕
    // 方案 —— 不会比现状更糟。
    reg [23:0] href_gap;        // 距上一个 HREF 上升沿的 PCLK 数（饱和）
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n)             href_gap <= 24'd0;
        else if (hr_sync)       href_gap <= 24'd0;
        else if (!href_gap[23]) href_gap <= href_gap + 24'd1;   // 饱和不溢出
    end
    // 阈值 1M PCLK：行间隔（~数千 PCLK）远小于它，帧尾消隐（~百万级）大于它
    localparam [23:0] GAP_TH = 24'd1000000;
    wire new_frame = href_hi & (href_gap >= GAP_TH);

    // ---- byte assembly ----
    reg        hi_sel;      // 0 = expecting high byte, 1 = expecting low byte
    reg [7:0]  byte_hi;
    reg [15:0] byte_cnt;    // bytes seen in current HREF

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            de       <= 1'b0;
            pixel    <= 16'd0;
            hi_sel   <= 1'b0;
            byte_hi  <= 8'd0;
            byte_cnt <= 16'd0;
            x_cnt    <= 10'd0;
            y_cnt    <= 10'd0;
            vs_toggle<= 1'b0;
            line_len <= 16'd0;
        end else begin
            de <= 1'b0;

            if (href_hi) begin
                hi_sel   <= 1'b0;
                byte_cnt <= 16'd0;
            end

            if (hr_sync) begin
                byte_cnt <= byte_cnt + 16'd1;
                if (!hi_sel) begin
                    byte_hi <= data;
                    hi_sel  <= 1'b1;
                end else begin
                    pixel  <= {byte_hi, data};   // RGB565
                    de     <= 1'b1;
                    hi_sel <= 1'b0;
                    if (x_cnt == IMG_W - 1)
                        x_cnt <= 10'd0;
                    else
                        x_cnt <= x_cnt + 10'd1;
                end
            end else begin
                hi_sel <= 1'b0;
            end

            if (href_lo) begin
                line_len <= byte_cnt;
                if (byte_cnt > 16'd16) begin
                    if (y_cnt == IMG_H - 1)
                        y_cnt <= 10'd0;
                    else
                        y_cnt <= y_cnt + 10'd1;
                end
            end

            if (vs_rise) begin
                vs_toggle <= ~vs_toggle;   // 仅诊断保留（第 24 轮起不再清 y_cnt，
                                           // VSYNC 不可信，清零时机错会打乱帧对齐）
            end

            // ★ 第 24 轮：真实帧起点 —— 新帧第一个 HREF 上升沿清零行计数。
            //   放在 vs_rise 块之后（同拍冲突时 new_frame 赢，last-assignment）。
            if (new_frame) begin
                x_cnt <= 10'd0;
                y_cnt <= 10'd0;
            end

        end
    end

endmodule
