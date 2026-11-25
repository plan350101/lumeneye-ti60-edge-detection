// ============================================================
//  shape_detect.v  —— 简易目标识别（黑形状/白底，面积填充率法）
//  2026-11-02 A 重写：不再依赖边缘覆盖率/bbox，而是直接看“黑色区域”：
//    - 黑形状 = gray < DARK_THR（默认 80）；
//    - 统计 ROI 内暗像素的 bbox + 暗像素数；
//    - 填充率 = 暗像素数 / bbox 面积：
//        矩形 ≈ 0.90~1.00 -> class=2（实心块）
//        圆   ≈ 0.60~0.90 (pi/4≈0.785) -> class=1（空心块）
//    - 长宽比 0.70~1.43；
//    - 暗像素太少/bbox 太小 -> class=0。
//  前提：白底 + 黑色形状、目标居中占满 ROI（干净背景）。
//  资源：0 RAM10。
// ============================================================
module shape_detect #(
    parameter IMG_W = 640,
    parameter [9:0]  ROI_X0 = 10'd200,
    parameter [9:0]  ROI_X1 = 10'd439,
    parameter [9:0]  ROI_Y0 = 10'd150,
    parameter [9:0]  ROI_Y1 = 10'd329,
    parameter [17:0] MIN_DARK = 18'd2000,
    parameter [9:0]  MIN_BOX  = 10'd60,
    parameter [7:0]  DARK_THR = 8'd80,
    parameter [9:0]  LINE_GAP = 10'd3
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de,
    input  wire [7:0] gray,
    input  wire       frame_start,
    output reg  [9:0] bbox_x0,
    output reg  [9:0] bbox_x1,
    output reg  [9:0] bbox_y0,
    output reg  [9:0] bbox_y1,
    output reg  [1:0] shape_class,
    output reg        valid
);

    reg [9:0] xcnt, ycnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xcnt <= 10'd0; ycnt <= 10'd0;
        end else if (frame_start) begin
            xcnt <= 10'd0; ycnt <= 10'd0;
        end else if (de) begin
            if (xcnt == IMG_W-1) begin
                xcnt <= 10'd0;
                ycnt <= ycnt + 10'd1;
            end else begin
                xcnt <= xcnt + 10'd1;
            end
        end
    end
    wire [9:0] x_cur = xcnt;
    wire [9:0] y_cur = ycnt;

    wire dark   = (gray < DARK_THR);
    wire in_roi = (x_cur >= ROI_X0) && (x_cur <= ROI_X1) &&
                  (y_cur >= ROI_Y0) && (y_cur <= ROI_Y1);
    wire cnt_en = de & dark & in_roi & ~frame_start;

    reg [9:0]  xmin, xmax, ymin, ymax;
    reg [17:0] dark_cnt;

    wire        has_e = (dark_cnt >= MIN_DARK);
    wire [9:0]  bw    = has_e ? (xmax - xmin + 10'd1) : 10'd0;
    wire [9:0]  bh    = has_e ? (ymax - ymin + 10'd1) : 10'd0;
    wire [19:0] area  = bw * bh;
    wire        big_e = has_e && (bw >= MIN_BOX) && (bh >= MIN_BOX);
    wire        fill_hi  = (dark_cnt * 10) >= (area * 9);
    wire        fill_mid = (dark_cnt * 10) >= (area * 6);
    wire        asp_ok   = ((bw * 7) <= (bh * 10)) && ((bw * 10) >= (bh * 7));
    wire [1:0]  cls_nx   = !big_e ? 2'd0 :
                           (fill_hi && asp_ok) ? 2'd2 :
                           (fill_mid && asp_ok) ? 2'd1 : 2'd0;

    reg [1:0] warm;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bbox_x0 <= 10'd0; bbox_x1 <= 10'd0;
            bbox_y0 <= 10'd0; bbox_y1 <= 10'd0;
            shape_class <= 2'd0; valid <= 1'b0; warm <= 2'd0;
            xmin <= 10'h3FF; xmax <= 10'd0;
            ymin <= 10'h3FF; ymax <= 10'd0;
            dark_cnt <= 18'd0;
        end else if (frame_start) begin
            bbox_x0 <= has_e ? xmin : 10'd0;
            bbox_x1 <= has_e ? xmax : 10'd0;
            bbox_y0 <= has_e ? ymin : 10'd0;
            bbox_y1 <= has_e ? ymax : 10'd0;
            if (warm != 2'd2) begin
                warm        <= warm + 2'd1;
                shape_class <= 2'd0;
                valid       <= 1'b0;
            end else begin
                shape_class <= cls_nx;
                valid       <= (cls_nx != 2'd0);
            end
            xmin <= 10'h3FF; xmax <= 10'd0;
            ymin <= 10'h3FF; ymax <= 10'd0;
            dark_cnt <= 18'd0;
        end else if (cnt_en) begin
            if (x_cur < xmin) xmin <= x_cur;
            if (x_cur > xmax) xmax <= x_cur;
            if (y_cur < ymin) ymin <= y_cur;
            if (y_cur > ymax) ymax <= y_cur;
            if (!dark_cnt[17]) dark_cnt <= dark_cnt + 18'd1;
        end
    end

endmodule