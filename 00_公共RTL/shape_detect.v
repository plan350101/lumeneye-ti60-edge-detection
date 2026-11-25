// ============================================================
//  shape_detect.v  —— 简易目标识别：圆 / 矩形（高阶 #6）
//  2026-11-02 A 整理版：x/y 直接由顶层输入（pcol/row_d1），
//  帧末分类 + 上一帧 bbox 边框重合度；算法与 C 的交付一致。
//  资源：~0 RAM10（行移位寄存器 SRL 化），约 250 LUT4。
// ============================================================
module shape_detect #(
    parameter IMG_W = 640,
    parameter [9:0]  ROI_X0 = 10'd64,
    parameter [9:0]  ROI_X1 = 10'd575,
    parameter [9:0]  ROI_Y0 = 10'd48,
    parameter [9:0]  ROI_Y1 = 10'd431,
    parameter [9:0]  TOL    = 10'd3,
    parameter [17:0] MIN_EDGE = 18'd350,
    parameter [9:0]  MIN_BOX  = 10'd60,
    parameter [9:0]  LINE_GAP = 10'd3      // 兼容原接口，本版不再使用
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de,            // 每拍一个有效像素
    input  wire       edge_i,        // 1 = 该像素是边缘（原名 edge 是关键字）
    input  wire       frame_start,   // 帧边界脉冲（接 frame_end）
    input  wire [9:0] x_cur,         // 当前像素 x（顶层接 pcol）
    input  wire [9:0] y_cur,         // 当前像素 y（顶层接 row_d1）

    output reg  [9:0] bbox_x0,
    output reg  [9:0] bbox_x1,
    output reg  [9:0] bbox_y0,
    output reg  [9:0] bbox_y1,
    output reg  [1:0] shape_class,   // 0=无, 1=圆, 2=矩形
    output reg        valid
);

    // ---------------- 上一行 / 左邻（孤立点剔除） ----------------
    reg [IMG_W-1:0] row_sr;
    reg             edge_l;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_sr <= {IMG_W{1'b0}};
            edge_l <= 1'b0;
        end else if (de) begin
            row_sr <= {row_sr[IMG_W-2:0], edge_i};
            edge_l <= edge_i;
        end
    end
    wire n_up   = row_sr[IMG_W-1];
    wire n_upl  = row_sr[IMG_W-2];
    wire n_left = edge_l;
    wire supported = edge_i & (n_up | n_left | n_upl);

    // ---------------- ROI + 上一帧 bbox 边框带 ----------------
    wire in_roi = (x_cur >= ROI_X0) && (x_cur <= ROI_X1) &&
                  (y_cur >= ROI_Y0) && (y_cur <= ROI_Y1);

    reg  [9:0] px0, px1, py0, py1;
    reg        pb_ok;
    wire [9:0] px0p = px0 + TOL;
    wire [9:0] px1m = (px1 > TOL) ? (px1 - TOL) : 10'd0;
    wire [9:0] py0p = py0 + TOL;
    wire [9:0] py1m = (py1 > TOL) ? (py1 - TOL) : 10'd0;
    wire on_border = (x_cur <= px0p) || (x_cur >= px1m) ||
                     (y_cur <= py0p) || (y_cur >= py1m);

    wire cnt_en = de & supported & in_roi & ~frame_start;
    wire cnt_bx = cnt_en & on_border & pb_ok;

    // ---------------- 帧内累加 ----------------
    reg [9:0]  xmin, xmax, ymin, ymax;
    reg [17:0] cnt_e, cnt_b;

    // ---------------- 帧末分类 ----------------
    wire        has_e = (cnt_e >= MIN_EDGE);
    wire [9:0]  bw    = has_e ? (xmax - xmin + 10'd1) : 10'd0;
    wire [9:0]  bh    = has_e ? (ymax - ymin + 10'd1) : 10'd0;
    wire        big_e = has_e && (bw >= MIN_BOX) && (bh >= MIN_BOX);
    wire        is_rect = (cnt_b * 10) >  (cnt_e * 7);
    wire        circ_lo = (cnt_b * 20) >= (cnt_e * 3);
    wire        asp_ok  = ((bw * 7) <= (bh * 10)) && ((bw * 10) >= (bh * 7));
    wire [1:0]  cls_nx  = !big_e ? 2'd0 :
                          is_rect ? 2'd2 :
                          (circ_lo && asp_ok) ? 2'd1 : 2'd0;

    reg [1:0] warm;   // 前 2 帧不判

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bbox_x0 <= 10'd0; bbox_x1 <= 10'd0;
            bbox_y0 <= 10'd0; bbox_y1 <= 10'd0;
            shape_class <= 2'd0; valid <= 1'b0;
            px0 <= 10'd0; px1 <= 10'd0; py0 <= 10'd0; py1 <= 10'd0;
            pb_ok <= 1'b0; warm <= 2'd0;
            xmin <= 10'h3FF; xmax <= 10'd0;
            ymin <= 10'h3FF; ymax <= 10'd0;
            cnt_e <= 18'd0; cnt_b <= 18'd0;
        end else if (frame_start) begin
            // 发布本帧 bbox（上一帧统计出来的）
            bbox_x0 <= has_e ? xmin : 10'd0;
            bbox_x1 <= has_e ? xmax : 10'd0;
            bbox_y0 <= has_e ? ymin : 10'd0;
            bbox_y1 <= has_e ? ymax : 10'd0;
            // 类别（warmup）
            if (warm != 2'd2) begin
                warm        <= warm + 2'd1;
                shape_class <= 2'd0;
                valid       <= 1'b0;
            end else begin
                shape_class <= cls_nx;
                valid       <= (cls_nx != 2'd0);
            end
            // 本帧 bbox 给下一帧做边框统计
            px0 <= has_e ? xmin : 10'd0;
            px1 <= has_e ? xmax : 10'd0;
            py0 <= has_e ? ymin : 10'd0;
            py1 <= has_e ? ymax : 10'd0;
            pb_ok <= has_e;
            // 清累加
            xmin <= 10'h3FF; xmax <= 10'd0;
            ymin <= 10'h3FF; ymax <= 10'd0;
            cnt_e <= 18'd0; cnt_b <= 18'd0;
        end else if (cnt_en) begin
            if (x_cur < xmin) xmin <= x_cur;
            if (x_cur > xmax) xmax <= x_cur;
            if (y_cur < ymin) ymin <= y_cur;
            if (y_cur > ymax) ymax <= y_cur;
            if (!cnt_e[17]) cnt_e <= cnt_e + 18'd1;
            if (cnt_bx && !cnt_b[17]) cnt_b <= cnt_b + 18'd1;
        end
    end

endmodule