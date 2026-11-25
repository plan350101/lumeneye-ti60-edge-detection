`timescale 1ns/1ps
// 2026-11-02 A：negde 驱动、posedge 采样的无竞争 tb（验证 clean shape_detect）
// 2026-11-25 修正：模块已改为内部 x/y 计数（xcnt/ycnt），移除 tb 中残留的
//              .x_cur/.y_cur 端口连接与 xin/yin 信号，恢复可编译状态
module tb_shape_detect;
    localparam IMG_W = 160;
    localparam IMG_H = 120;
    localparam integer RECT_X0 = 45, RECT_X1 = 115, RECT_Y0 = 35, RECT_Y1 = 85;
    localparam integer CIRC_CX = 80, CIRC_CY = 60, CIRC_R = 28;

    reg clk = 0, rst_n = 0, de = 0, edge_i = 0, frame_start = 0;
    wire [9:0] bx0, bx1, by0, by1;
    wire [1:0] cls;
    wire vld;
    always #5 clk = ~clk;

    shape_detect #(
        .IMG_W(IMG_W), .ROI_X0(10'd16), .ROI_X1(10'd143),
        .ROI_Y0(10'd12), .ROI_Y1(10'd107), .TOL(10'd3),
        .MIN_EDGE(18'd40), .MIN_BOX(10'd20), .LINE_GAP(10'd3)
    ) dut (
        .clk(clk), .rst_n(rst_n), .de(de), .edge_i(edge_i),
        .frame_start(frame_start),
        .bbox_x0(bx0), .bbox_x1(bx1), .bbox_y0(by0), .bbox_y1(by1),
        .shape_class(cls), .valid(vld)
    );

    integer errors = 0;
    function in_shape;
        input integer kind, x, y;
        integer dx, dy, d2;
        begin
            in_shape = 1'b0;
            if (kind == 1) begin
                if ((x >= RECT_X0) && (x <= RECT_X1) && (y >= RECT_Y0) && (y <= RECT_Y1))
                    if ((x <= RECT_X0+1) || (x >= RECT_X1-1) || (y <= RECT_Y0+1) || (y >= RECT_Y1-1))
                        in_shape = 1'b1;
            end else if (kind == 2) begin
                dx = x - CIRC_CX; dy = y - CIRC_CY; d2 = dx*dx + dy*dy;
                if ((d2 <= CIRC_R*CIRC_R) && (d2 >= (CIRC_R-2)*(CIRC_R-2)))
                    in_shape = 1'b1;
            end
        end
    endfunction

    task send_frame;
        input integer kind;
        integer x, y;
        begin
            @(negedge clk); frame_start = 1'b1; de = 1'b0; edge_i = 1'b0;
            @(negedge clk); frame_start = 1'b0;
            repeat (10) @(negedge clk);
            for (y = 0; y < IMG_H; y = y + 1) begin
                for (x = 0; x < IMG_W; x = x + 1) begin
                    @(negedge clk);
                    de = 1'b1; edge_i = in_shape(kind, x, y);
                end
                @(negedge clk); de = 1'b0; edge_i = 1'b0;
                repeat (8) @(negedge clk);
            end
            repeat (20) @(negedge clk);
        end
    endtask

    task expect_cls;
        input [1:0] e_cls;
        input [127:0] tag;
        begin
            if (cls !== e_cls) begin
                $display("  [FAIL] %0s : class=%0d expect %0d (valid=%0b bbox=%0d,%0d-%0d,%0d)",
                         tag, cls, e_cls, vld, bx0, bx1, by0, by1);
                errors = errors + 1;
            end else
                $display("  [ ok ] %0s : class=%0d bbox=%0d,%0d-%0d,%0d valid=%0b",
                         tag, cls, bx0, bx1, by0, by1, vld);
        end
    endtask

    initial begin
        rst_n = 0; de = 0; edge_i = 0; frame_start = 0;
        repeat (10) @(negedge clk); rst_n = 1; repeat (3) @(negedge clk);
        send_frame(0); send_frame(0); send_frame(0); send_frame(0);
        expect_cls(2'd0, "blank");
        send_frame(1); send_frame(1); send_frame(1); send_frame(1);
        expect_cls(2'd2, "rectangle");
        send_frame(2); send_frame(2); send_frame(2); send_frame(2);
        expect_cls(2'd1, "circle");
        send_frame(0); send_frame(0); send_frame(0); send_frame(0);
        expect_cls(2'd0, "back to blank");
        send_frame(1); send_frame(1); send_frame(1); send_frame(1);
        expect_cls(2'd2, "rectangle again");
        if (errors == 0) $display("\n***** TB PASS *****");
        else             $display("\n***** TB FAIL : %0d errors *****", errors);
        $finish;
    end
endmodule