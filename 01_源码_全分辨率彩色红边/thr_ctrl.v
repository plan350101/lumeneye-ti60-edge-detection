// ============================================================
//  thr_ctrl.v  —— 按键实时阈值调节（高阶挑战 #2）
// ------------------------------------------------------------
//  功能：
//    K1（+）每次按下 thr_lo += STEP，thr_hi 跟随 = thr_lo + GAP
//    K2（-）每次按下 thr_lo -= STEP，thr_hi 跟随
//    按住不放超过 RPT_DLY 后自动连发，每 RPT_PER 一次（方便现场演示连续加减）
//
//  引脚（由 A 加到 camera_hdmi.isf / peri.xml，本模块只关心名字）：
//    key_plus  = K1 = ball K1 = GPIOL_P_07   （低有效，板上上拉）
//    key_minus = K2 = ball K2 = GPIOL_N_06   （低有效，板上上拉）
//
//  ★ 复位值就是比赛默认值 LO=48 / HI=64，与 camera_hdmi.v 原
//    SOBEL_THR / SOBEL_HI 完全一致 —— 所以接上后即使不按键，
//    画面与改之前一模一样，可以直接对比。
//
//  资源预估：约 55 个 FF、约 60 个 LUT4、0 块 RAM10、0 个 DSP
//  延迟    ：按键 -> thr_lo/thr_hi 生效 = DEB_CNT + 2 拍
//            （DEB_CNT 默认 2^20，见下方时基说明）
//
//  ★★ 时基说明（很重要，别当成 bug）★★
//    本模块按接口契约跑在 cmos_pclk 上，而相机像素时钟不是定值：
//    OV5640 640x480 实测约 24~48 MHz，SDC 里按 96 MHz 上限约束。
//    DEB_CNT 默认 2^20 = 1048576，对应：
//        96 MHz -> 10.9 ms      48 MHz -> 21.8 ms      24 MHz -> 43.7 ms
//    三者都是合格的消抖时间（10~44 ms），手按一次 100 ms 完全够。
//    如果实测发现反应偏慢/偏快，只改 DEB_CNT 一个参数即可。
//
//    连发用 DEB_CNT 的倍数表达，不引入额外大常数：
//        RPT_DLY = 8*DEB_CNT （初次按住后约 87 ms@96MHz / 350 ms@24MHz）
//        RPT_PER = 2*DEB_CNT （之后每 22 ms@96MHz / 87 ms@24MHz 一次）
//
//  约束：可综合 Verilog-2001，无 latch / 无 initial / 无 $display
// ============================================================

module thr_ctrl #(
    parameter [7:0]  INIT_LO = 8'd48,      // 复位默认阈值（= 原 SOBEL_THR）
    parameter [7:0]  INIT_HI = 8'd64,      // 复位默认阈值（= 原 SOBEL_HI）
    parameter [7:0]  STEP    = 8'd8,       // 每次步进
    parameter [7:0]  GAP     = 8'd16,      // thr_hi = thr_lo + GAP（固定）
    parameter [7:0]  LO_MIN  = 8'd8,       // thr_lo 下限
    parameter [7:0]  LO_MAX  = 8'd232,     // thr_lo 上限（232 + 16 = 248，不溢出）
    parameter [23:0] DEB_CNT = 24'd1_048_576   // 消抖计数，见文件头时基说明
)(
    input  wire       clk,          // cmos_pclk
    input  wire       rst_n,        // 低有效异步复位
    input  wire       key_plus,     // K1，低有效，异步
    input  wire       key_minus,    // K2，低有效，异步
    output wire [7:0] thr_lo,
    output wire [7:0] thr_hi
);

    // ---------------------------------------------------------
    //  1. 按键同步 + 消抖（每个键一套）
    // ---------------------------------------------------------
    // 两级同步器
    reg [1:0] ps, ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ps <= 2'b11;
            ms <= 2'b11;
        end else begin
            ps <= {ps[0], key_plus};
            ms <= {ms[0], key_minus};
        end
    end

    // 消抖后稳定电平（1 = 松开，0 = 按下）
    reg        p_stable, m_stable;
    reg [23:0] p_cnt,    m_cnt;

    wire p_diff = (ps[1] != p_stable);
    wire m_diff = (ms[1] != m_stable);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_stable <= 1'b1;
            p_cnt    <= 24'd0;
        end else if (p_diff) begin
            if (p_cnt == DEB_CNT) begin
                p_stable <= ps[1];
                p_cnt    <= 24'd0;
            end else begin
                p_cnt <= p_cnt + 24'd1;
            end
        end else begin
            p_cnt <= 24'd0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_stable <= 1'b1;
            m_cnt    <= 24'd0;
        end else if (m_diff) begin
            if (m_cnt == DEB_CNT) begin
                m_stable <= ms[1];
                m_cnt    <= 24'd0;
            end else begin
                m_cnt <= m_cnt + 24'd1;
            end
        end else begin
            m_cnt <= 24'd0;
        end
    end

    // ---------------------------------------------------------
    //  2. 按下沿 + 按住连发
    // ---------------------------------------------------------
    localparam [23:0] RPT_DLY = {DEB_CNT[20:0], 3'b000};      // 8 * DEB_CNT
    localparam [23:0] RPT_PER = {DEB_CNT[22:0], 1'b0};        // 2 * DEB_CNT
    localparam [23:0] RPT_RST = RPT_DLY - RPT_PER;            // 6 * DEB_CNT

    reg        p_d, m_d;          // 上一拍稳定值，用来找下降沿
    reg [23:0] p_t, m_t;          // 按住计时

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_d <= 1'b1;
            m_d <= 1'b1;
            p_t <= 24'd0;
            m_t <= 24'd0;
        end else begin
            p_d <= p_stable;
            m_d <= m_stable;

            // 松开就清零；按住就一路数到 RPT_DLY，然后回卷到 RPT_RST
            if (p_stable)                                p_t <= 24'd0;
            else if (p_t == RPT_DLY)                     p_t <= RPT_RST;
            else if (p_t != 24'hFFFFFF)                  p_t <= p_t + 24'd1;

            if (m_stable)                                m_t <= 24'd0;
            else if (m_t == RPT_DLY)                     m_t <= RPT_RST;
            else if (m_t != 24'hFFFFFF)                  m_t <= m_t + 24'd1;
        end
    end

    // 首次按下（下降沿）或按住满 RPT_DLY 时产生一次“走一步”
    wire step_up   = (p_d & ~p_stable) | (~p_stable & (p_t == RPT_DLY));
    wire step_down = (m_d & ~m_stable) | (~m_stable & (m_t == RPT_DLY));

    // ★ 两个键同拍按下时，以“加”为准（避免亚稳态导致的不确定）
    //   注意不能写成 (step_up & ~step_down) —— 那样同按时两个都变 0，等于没反应。
    wire do_up   = step_up;
    wire do_down = step_down & ~step_up;

    // ---------------------------------------------------------
    //  3. 阈值寄存器（clamp + 固定 GAP）
    //  ★ 2026-11-02 修复：板上 rst_n=KEY0 按键，上电未按时不会走复位分支，
    //    FPGA 寄存器初值为 0 → 原实现输出 0/0，边缘爆炸。
    //    改为：未按键前输出 INIT_LO/INIT_HI（与改 K1 之前的固定阈值一致）；
    //    第一次按键从 INIT 开始加减。
    // ---------------------------------------------------------
    reg  [7:0] lo_r, hi_r;
    reg        loaded;
    wire [7:0] lo_cur = loaded ? lo_r : INIT_LO;
    wire [7:0] lo_inc = (lo_cur > (LO_MAX - STEP)) ? LO_MAX : (lo_cur + STEP);
    wire [7:0] lo_dec = (lo_cur < (LO_MIN + STEP)) ? LO_MIN : (lo_cur - STEP);

    wire [7:0] lo_nx = do_up ? lo_inc : do_down ? lo_dec : lo_cur;
    wire [8:0] hi_sum = {1'b0, lo_nx} + {1'b0, GAP};
    wire [7:0] hi_nx = (hi_sum > 9'd248) ? 8'd248 : hi_sum[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lo_r <= INIT_LO;
            hi_r <= INIT_HI;
            loaded <= 1'b0;
        end else if (do_up | do_down) begin
            lo_r <= lo_nx;
            hi_r <= hi_nx;
            loaded <= 1'b1;
        end
    end
    assign thr_lo = loaded ? lo_r : INIT_LO;
    assign thr_hi = loaded ? hi_r : INIT_HI;

endmodule
