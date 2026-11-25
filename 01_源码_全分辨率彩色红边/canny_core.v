// ============================================================
//  canny_core.v  —— 3x3 Canny（A 线；同步 BRAM 行缓存版）
// ------------------------------------------------------------
//  GAUSS_EN=0：简化版（Sobel 用原始灰度）
//  GAUSS_EN=1：严格版（3x3 高斯 -> Sobel）
//  两版共用方向量化 / NMS / 双阈值 + 简单滞后
//
//  行缓存：raw 3 行 + gauss 3 行 + mag/dir 3 行，全部用
//          line_buffer（(* ramstyle="block" *)，同步读，1 拍延迟）
//  延迟：约 1 行 + 4 拍（输出对应上一行像素）
//  端口约 edge_o（不能叫 edge，Verilog 关键字）
// ============================================================

module canny_core #(
    parameter [7:0] THR_LO = 8'd48,
    parameter [7:0] THR_HI = 8'd64,
    parameter integer W = 640,
    parameter integer GAUSS_EN = 0
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de,
    input  wire [7:0] din,
    input  wire [7:0] thr_lo,
    input  wire [7:0] thr_hi,
    output wire       de_out,
    output wire       edge_o
);

    // ---------------- 输入位置 + 流水位置 ----------------
    reg [9:0] col, row;
    reg de1, de2, de3, de4;
    reg [9:0] col1, col2, col3, col4;
    reg [9:0] row1, row2, row3, row4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col <= 0; row <= 0;
        end else if (de) begin
            if (col == W-1) begin col <= 0; row <= row + 1; end
            else col <= col + 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de1<=0; de2<=0; de3<=0; de4<=0;
            col1<=0; col2<=0; col3<=0; col4<=0;
            row1<=0; row2<=0; row3<=0; row4<=0;
        end else begin
            de1<=de;  col1<=col;  row1<=row;
            de2<=de1; col2<=col1; row2<=row1;
            de3<=de2; col3<=col2; row3<=row2;
            de4<=de3; col4<=col3; row4<=row3;
        end
    end

    // ---------------- raw 3 行 line buffer ----------------
    reg [1:0] raw_wr;
    wire [7:0] rq0, rq1, rq2;
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_raw0 (
        .wr_clk(clk), .wr_en(de & (raw_wr==2'd0)), .wr_addr(col), .wr_data(din),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col), .rd_data(rq0));
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_raw1 (
        .wr_clk(clk), .wr_en(de & (raw_wr==2'd1)), .wr_addr(col), .wr_data(din),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col), .rd_data(rq1));
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_raw2 (
        .wr_clk(clk), .wr_en(de & (raw_wr==2'd2)), .wr_addr(col), .wr_data(din),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col), .rd_data(rq2));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) raw_wr <= 2'd0;
        else if (de && (col == W-1)) raw_wr <= (raw_wr == 2'd2) ? 2'd0 : raw_wr + 2'd1;
    end
    reg [1:0] raw_wr_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) raw_wr_d1 <= 2'd0;
        else        raw_wr_d1 <= raw_wr;
    end

    wire [7:0] rp1 = (raw_wr_d1==2'd0) ? rq2 : (raw_wr_d1==2'd1) ? rq0 : rq1;
    wire [7:0] rp2 = (raw_wr_d1==2'd0) ? rq1 : (raw_wr_d1==2'd1) ? rq2 : rq0;

    reg [7:0] d1, d2, d3;
    reg [7:0] r1t1, r1t2, r2t1, r2t2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d1<=0; d2<=0; d3<=0;
            r1t1<=0; r1t2<=0; r2t1<=0; r2t2<=0;
        end else if (de) begin
            d1<=din; d2<=d1; d3<=d2;
            r1t1<=rp1; r1t2<=r1t1;
            r2t1<=rp2; r2t2<=r2t1;
        end
    end

    // stage1 原始 3x3 窗口（中心 = d1）
    wire [7:0] p11 = r2t2, p12 = r2t1, p13 = rp2;
    wire [7:0] p21 = r1t2, p22 = r1t1, p23 = rp1;
    wire [7:0] p31 = d3,   p32 = d2,   p33 = d1;

    // 高斯
    wire [12:0] gsum = {5'b0,p11} + {4'b0,p12,1'b0} + {5'b0,p13}
                     + {4'b0,p21,1'b0} + {3'b0,p22,2'b0} + {4'b0,p23,1'b0}
                     + {5'b0,p31} + {4'b0,p32,1'b0} + {5'b0,p33};
    wire [7:0] gauss_s1 = gsum[11:4];

    // raw Sobel（lite）
    wire signed [11:0] gx_r = (p13 + 2*p23 + p33) - (p11 + 2*p21 + p31);
    wire signed [11:0] gy_r = (p31 + 2*p32 + p33) - (p11 + 2*p12 + p13);
    wire [10:0] axr = gx_r[11] ? (~gx_r + 12'd1) : gx_r[10:0];
    wire [10:0] ayr = gy_r[11] ? (~gy_r + 12'd1) : gy_r[10:0];
    wire [11:0] mag_lite = axr + ayr;
    wire [11:0] axr2 = {axr,1'b0};
    wire [11:0] ayr2 = {ayr,1'b0};
    wire [1:0] dir_lite = (axr > ayr2) ? 2'd0 : (ayr > axr2) ? 2'd2 : (gx_r[11]==gy_r[11]) ? 2'd1 : 2'd3;

    // ---------------- gauss 3 行 line buffer ----------------
    reg [1:0] g_wr;
    wire [7:0] gq0, gq1, gq2;
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_g0 (
        .wr_clk(clk), .wr_en(de1 & (g_wr==2'd0)), .wr_addr(col1), .wr_data(gauss_s1),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col1), .rd_data(gq0));
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_g1 (
        .wr_clk(clk), .wr_en(de1 & (g_wr==2'd1)), .wr_addr(col1), .wr_data(gauss_s1),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col1), .rd_data(gq1));
    line_buffer #(.WIDTH(W), .DW(8), .AW(10)) u_g2 (
        .wr_clk(clk), .wr_en(de1 & (g_wr==2'd2)), .wr_addr(col1), .wr_data(gauss_s1),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col1), .rd_data(gq2));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) g_wr <= 2'd0;
        else if (de1 && (col1 == W-1)) g_wr <= (g_wr == 2'd2) ? 2'd0 : g_wr + 2'd1;
    end
    reg [1:0] g_wr_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) g_wr_d1 <= 2'd0;
        else        g_wr_d1 <= g_wr;
    end

    wire [7:0] gp1 = (g_wr_d1==2'd0) ? gq2 : (g_wr_d1==2'd1) ? gq0 : gq1;
    wire [7:0] gp2 = (g_wr_d1==2'd0) ? gq1 : (g_wr_d1==2'd1) ? gq2 : gq0;

    reg [7:0] gd1, gd2, gd3;
    reg [7:0] g1t1, g1t2, g2t1, g2t2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gd1<=0; gd2<=0; gd3<=0;
            g1t1<=0; g1t2<=0; g2t1<=0; g2t2<=0;
        end else if (de1) begin
            gd1<=gauss_s1; gd2<=gd1; gd3<=gd2;
            g1t1<=gp1; g1t2<=g1t1;
            g2t1<=gp2; g2t2<=g2t1;
        end
    end

    // stage2 gauss 窗口（中心 = gd1）
    wire [7:0] q11 = g2t2, q12 = g2t1, q13 = gp2;
    wire [7:0] q21 = g1t2, q22 = g1t1, q23 = gp1;
    wire [7:0] q31 = gd3,  q32 = gd2,  q33 = gd1;

    wire signed [11:0] gx_g = (q13 + 2*q23 + q33) - (q11 + 2*q21 + q31);
    wire signed [11:0] gy_g = (q31 + 2*q32 + q33) - (q11 + 2*q12 + q13);
    wire [10:0] axg = gx_g[11] ? (~gx_g + 12'd1) : gx_g[10:0];
    wire [10:0] ayg = gy_g[11] ? (~gy_g + 12'd1) : gy_g[10:0];
    wire [11:0] mag_strict = axg + ayg;
    wire [11:0] axg2 = {axg,1'b0};
    wire [11:0] ayg2 = {ayg,1'b0};
    wire [1:0] dir_strict = (axg > ayg2) ? 2'd0 : (ayg > axg2) ? 2'd2 : (gx_g[11]==gy_g[11]) ? 2'd1 : 2'd3;

    // lite 的 mag/dir 延迟 1 拍，和 strict 同 stage2
    reg [11:0] mag_lite_d1;
    reg [1:0]  dir_lite_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_lite_d1 <= 12'd0; dir_lite_d1 <= 2'd0;
        end else if (de1) begin
            mag_lite_d1 <= mag_lite; dir_lite_d1 <= dir_lite;
        end
    end

    wire [11:0] mag_s2 = GAUSS_EN ? mag_strict : mag_lite_d1;
    wire [1:0]  dir_s2 = GAUSS_EN ? dir_strict : dir_lite_d1;

    // ---------------- mag/dir 3 行 line buffer ----------------
    reg [1:0] m_wr;
    wire [13:0] mq0, mq1, mq2;
    line_buffer #(.WIDTH(W), .DW(14), .AW(10)) u_m0 (
        .wr_clk(clk), .wr_en(de2 & (m_wr==2'd0)), .wr_addr(col2), .wr_data({dir_s2, mag_s2}),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col2), .rd_data(mq0));
    line_buffer #(.WIDTH(W), .DW(14), .AW(10)) u_m1 (
        .wr_clk(clk), .wr_en(de2 & (m_wr==2'd1)), .wr_addr(col2), .wr_data({dir_s2, mag_s2}),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col2), .rd_data(mq1));
    line_buffer #(.WIDTH(W), .DW(14), .AW(10)) u_m2 (
        .wr_clk(clk), .wr_en(de2 & (m_wr==2'd2)), .wr_addr(col2), .wr_data({dir_s2, mag_s2}),
        .rd_clk(clk), .rd_en(1'b1), .rd_addr(col2), .rd_data(mq2));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) m_wr <= 2'd0;
        else if (de2 && (col2 == W-1)) m_wr <= (m_wr == 2'd2) ? 2'd0 : m_wr + 2'd1;
    end
    reg [1:0] m_wr_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) m_wr_d1 <= 2'd0;
        else        m_wr_d1 <= m_wr;
    end

    wire [13:0] mp1 = (m_wr_d1==2'd0) ? mq2 : (m_wr_d1==2'd1) ? mq0 : mq1;
    wire [13:0] mp2 = (m_wr_d1==2'd0) ? mq1 : (m_wr_d1==2'd1) ? mq2 : mq0;

    reg [13:0] m1t1, m1t2, m2t1, m2t2;
    reg [11:0] magd1, magd2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m1t1<=0; m1t2<=0; m2t1<=0; m2t2<=0;
            magd1<=0; magd2<=0;
        end else if (de2) begin
            m1t1<=mp1; m1t2<=m1t1;
            m2t1<=mp2; m2t2<=m2t1;
            magd1<=mag_s2; magd2<=magd1;
        end
    end

    // NMS（stage4，中心 = m1t1，对应上一行 col(t)）
    wire [11:0] mc = m1t1[11:0];
    wire [1:0]  dc = m1t1[13:12];
    wire [11:0] ml = m1t2[11:0];
    wire [11:0] mr = mp1[11:0];
    wire [11:0] mu = m2t1[11:0];
    wire [11:0] md = magd2;
    wire [11:0] mul = m2t2[11:0];
    wire [11:0] mur = mp2[11:0];

    wire nms = (dc == 2'd0) ? ((mc > ml) && (mc >= mr)) :
               (dc == 2'd2) ? ((mc > mu) && (mc >= md)) :
               (dc == 2'd1) ? ((mc > mul) && (mc >= mur)) :
                              ((mc > mur) && (mc >= mul));
    wire [11:0] nms_mag = nms ? mc : 12'd0;

    wire strong = (nms_mag > {4'b0, thr_hi});
    wire weak   = (nms_mag > {4'b0, thr_lo});

    reg [W-1:0] strong_sr;
    reg         strong_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strong_sr <= {W{1'b0}};
            strong_prev <= 1'b0;
        end else if (de4) begin
            strong_sr <= {strong_sr[W-2:0], strong};
            strong_prev <= strong;
        end
    end
    wire strong_above = strong_sr[W-1];

    wire valid = de4 & (row4 >= 10'd4) & (col4 >= 10'd2) & (col4 <= W-3);
    assign de_out = valid;
    assign edge_o = valid & (strong | (weak & (strong_prev | strong_above)));

endmodule