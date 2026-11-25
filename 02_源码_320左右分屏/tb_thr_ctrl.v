// ============================================================
//  tb_thr_ctrl.v —— thr_ctrl 自检 testbench
// ------------------------------------------------------------
//  仿真（iverilog）：
//      iverilog -g2005 -o tb_thr.vvp tb_thr_ctrl.v ../thr_ctrl.v
//      vvp tb_thr.vvp
//  期望输出最后一行：  ***** TB PASS *****
//
//  覆盖：复位值 / 单次加 / 单次减 / 抖动不动作 / 下限 clamp /
//        上限 clamp / 按住连发 / 两键同按以“加”为准
// ============================================================
`timescale 1ns/1ps

module tb_thr_ctrl;

    localparam [23:0] DEB = 24'd8;      // 仿真把消抖压到 8 拍

    reg  clk, rst_n, kp, km;
    wire [7:0] lo, hi;

    thr_ctrl #(
        .INIT_LO (8'd48),
        .INIT_HI (8'd64),
        .STEP    (8'd8),
        .GAP     (8'd16),
        .LO_MIN  (8'd8),
        .LO_MAX  (8'd232),
        .DEB_CNT (DEB)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .key_plus(kp), .key_minus(km),
        .thr_lo(lo), .thr_hi(hi)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;               // 100 MHz，只为仿真

    integer errors = 0;

    task chk;
        input [7:0] e_lo, e_hi;
        input [127:0] tag;
        begin
            if ((lo !== e_lo) || (hi !== e_hi)) begin
                $display("  [FAIL] %0s : got lo=%0d hi=%0d, expect lo=%0d hi=%0d",
                         tag, lo, hi, e_lo, e_hi);
                errors = errors + 1;
            end else begin
                $display("  [ ok ] %0s : lo=%0d hi=%0d", tag, lo, hi);
            end
        end
    endtask

    // 按住 n 拍再松开
    task press_plus;
        input integer n;
        begin
            kp = 1'b0; #(n*10); kp = 1'b1; #(40*10);
        end
    endtask
    task press_minus;
        input integer n;
        begin
            km = 1'b0; #(n*10); km = 1'b1; #(40*10);
        end
    endtask

    integer i;

    initial begin
        $dumpfile("tb_thr_ctrl.vcd");
        $dumpvars(0, tb_thr_ctrl);

        rst_n = 1'b0; kp = 1'b1; km = 1'b1;
        #200;
        rst_n = 1'b1;
        #200;

        $display("---- 1. 复位默认值 ----");
        chk(8'd48, 8'd64, "reset");

        $display("---- 2. 单次 + ----");
        press_plus(30);                 // 30 拍 > DEB，稳一点
        chk(8'd56, 8'd72, "one plus");

        $display("---- 3. 连续两次 + ----");
        press_plus(30);
        press_plus(30);
        chk(8'd72, 8'd88, "two more plus");

        $display("---- 4. 抖动不应动作（按住 3 拍就松开，短于消抖）----");
        kp = 1'b0; #30; kp = 1'b1; #200;
        chk(8'd72, 8'd88, "glitch ignored");

        $display("---- 5. 单次 - ----");
        press_minus(30);
        chk(8'd64, 8'd80, "one minus");

        $display("---- 6. 按住连发（应连加多次）----");
        kp = 1'b0; #(DEB*40*10); kp = 1'b1; #400;
        if (lo <= 8'd64) begin
            $display("  [FAIL] auto-repeat : lo=%0d 没有超过 %0d", lo, 8'd64);
            errors = errors + 1;
        end else begin
            $display("  [ ok ] auto-repeat : lo=%0d", lo);
        end

        $display("---- 7. 上限 clamp ----");
        for (i = 0; i < 40; i = i + 1) press_plus(30);
        chk(8'd232, 8'd248, "clamp hi");

        $display("---- 8. 下限 clamp ----");
        for (i = 0; i < 40; i = i + 1) press_minus(30);
        chk(8'd8, 8'd24, "clamp lo");

        $display("---- 9. 两键同按 -> 以加为准 ----");
        kp = 1'b0; km = 1'b0; #(40*10);
        kp = 1'b1; km = 1'b1; #400;
        chk(8'd16, 8'd32, "both -> plus wins");

        if (errors == 0) $display("\n***** TB PASS *****");
        else             $display("\n***** TB FAIL : %0d errors *****", errors);

        $finish;
    end

endmodule
