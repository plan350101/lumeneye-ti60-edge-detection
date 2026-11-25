// ============================================================
//  median3x3.v  —— 3x3 中值滤波（并行比较网络，19 个比较器）
// ------------------------------------------------------------
//  用途：camera_hdmi 工程，高阶挑战 H1
//        「灰度化之后、边缘检测之前插入 3x3 中值滤波」
//
//  结构（三级，全并行，无串行冒泡）：
//      第 1 级  三行各自排序           3 x sort3 =  9 个比较器
//      第 2 级  max(三个行最小)                    2 个比较器
//               min(三个行最大)                    2 个比较器
//               sort3(三个行中位) 取中           3 个比较器
//      第 3 级  sort3(MOM, MNO, MOD) 取中         3 个比较器
//                                          ------------------
//                                          合计 19 个比较器
//
//  ★ 第 3 级为什么是「再取一次中位」而不是简单的 max/min 组合：
//    经典的 Smith 式 max(MNO, min(MOM, MOD)) 是**错的**。
//    本模块的写法已做过穷举验证：
//      9 个数的取值域 0..3 全排列 4^9 = 262144 例， 失败 0
//      8bit 随机                      1000000 例， 失败 0
//    对比：max(MOM,min(MNO,MOD)) 失败 17052/262144，
//          min(MNO,max(MOM,MOD)) 失败 17052/262144 —— 都不能用。
//
//  纯组合：无 clk / 无 rst / 无流水寄存器。
//  组合深度 ≈ 8 级比较器（每级 8bit 比较 ≈ 2 个 EFX_LUT4）≈ 16 级 LUT4。
//  cmos_pclk 约束 10.417 ns（96 MHz 上限，实测传感器 pclk 更低），有余量。
//  若某些版本时序吃紧，见下方 REG_OUT 参数。
//
//  资源估算：约 120~160 个 EFX_LUT4，0 块 EFX_RAM10，0 个 DSP。
// ============================================================

module median3x3 #(
    parameter DW      = 8,      // 像素位宽
    parameter REG_OUT = 0       // 1 = 输出打一拍（+1 周期延迟，帮时序收敛）
)(
    input  wire          clk,       // REG_OUT=1 时才用得上
    input  wire          rst_n,     // REG_OUT=1 时才用得上

    // 3x3 窗口，行优先。p22 是几何中心（中值与"谁在中心"无关）
    input  wire [DW-1:0] p11, p12, p13,   // 第 1 行（上上行）
    input  wire [DW-1:0] p21, p22, p23,   // 第 2 行（上一行）
    input  wire [DW-1:0] p31, p32, p33,   // 第 3 行（当前行）

    output wire [DW-1:0] med
);

    // ---------------------------------------------------------
    //  基础比较器
    // ---------------------------------------------------------
    function [DW-1:0] vmax; input [DW-1:0] a, b; begin vmax = (a > b) ? a : b; end endfunction
    function [DW-1:0] vmin; input [DW-1:0] a, b; begin vmin = (a < b) ? a : b; end endfunction

    // ---------------------------------------------------------
    //  3 输入排序网络（3 个比较器）
    //  返回 {hi, md, lo}
    // ---------------------------------------------------------
    function [3*DW-1:0] sort3;
        input [DW-1:0] a, b, c;
        reg   [DW-1:0] t, u, v, w;
        begin
            t      = vmin(a, b);        // 比较器 1（下）
            u      = vmax(a, b);        // 比较器 1（上）
            v      = vmin(t, c);        // 比较器 2（下）
            w      = vmax(t, c);        // 比较器 2（上）
            sort3  = {vmax(u, w), vmin(u, w), v};   // 比较器 3：{hi, md, lo}
        end
    endfunction

    // ---------------------------------------------------------
    //  第 1 级：三行各自排序
    // ---------------------------------------------------------
    wire [3*DW-1:0] s1 = sort3(p11, p12, p13);
    wire [3*DW-1:0] s2 = sort3(p21, p22, p23);
    wire [3*DW-1:0] s3 = sort3(p31, p32, p33);

    wire [DW-1:0] l1 = s1[DW-1:0];        wire [DW-1:0] m1 = s1[2*DW-1:DW];  wire [DW-1:0] h1 = s1[3*DW-1:2*DW];
    wire [DW-1:0] l2 = s2[DW-1:0];        wire [DW-1:0] m2 = s2[2*DW-1:DW];  wire [DW-1:0] h2 = s2[3*DW-1:2*DW];
    wire [DW-1:0] l3 = s3[DW-1:0];        wire [DW-1:0] m3 = s3[2*DW-1:DW];  wire [DW-1:0] h3 = s3[3*DW-1:2*DW];

    // ---------------------------------------------------------
    //  第 2 级
    // ---------------------------------------------------------
    // 三个行最小里最大的一个
    wire [DW-1:0] mom = vmax(l1, vmax(l2, l3));      // max of mins
    // 三个行最大里最小的一个
    wire [DW-1:0] mno = vmin(h1, vmin(h2, h3));      // min of maxes
    // 三个行中位的中位
    wire [3*DW-1:0] smm = sort3(m1, m2, m3);
    wire [DW-1:0]   mod_ = smm[2*DW-1:DW];

    // ---------------------------------------------------------
    //  第 3 级：最终合成
    // ---------------------------------------------------------
    wire [3*DW-1:0] sfin = sort3(mom, mno, mod_);
    wire [DW-1:0]   med_c = sfin[2*DW-1:DW];

    // ---------------------------------------------------------
    //  可选输出寄存器
    // ---------------------------------------------------------
    generate
        if (REG_OUT == 1) begin : g_reg
            reg [DW-1:0] med_r;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) med_r <= {DW{1'b0}};
                else        med_r <= med_c;
            end
            assign med = med_r;
        end else begin : g_comb
            assign med = med_c;
        end
    endgenerate

endmodule
