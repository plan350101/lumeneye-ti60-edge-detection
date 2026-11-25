#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UART 软显示器 —— 把 FPGA（VF-Ti60F225）经板载 CH340N 打回的
64x48 缩略图重建成图片，用于在没有 HDMI 显示器时"看画面"。

FPGA 侧协议（见 camera_hdmi.v 第 13 节）:
    '@' <2位hex帧号> <2位hex状态字> CR LF          <- 帧头（7 字符）
    48 行，每行 64 个像素、每像素 2 位 hex（共 128 字符）+ CR LF
    115200-8-N-1，约 1.5 帧/秒

    ★ 缩略图 = 显示画面 640x480 的 **10x10 盒式平均**（不是点抽样）。
      板内输出 = vacc>>7 = 0.78 x 方块平均灰度（线性、不截顶），
      所以本脚本默认乘 1.28 还原真实灰度；要原始值加 --raw。

    状态字 bit0..7 = hdmi_pll / ddr_pll / cal_done / cal_pass /
                    cfg_done / DVP / any_ack / 恒1(版本标记)
    —— 所以"校准过没过、摄像头配没配上"在串口里是**写成字的**，不用靠 LED 猜。

用法:
    python uart_view.py --list                    列出串口
    python uart_view.py --port COM5               接收并持续刷新 frame.png
    python uart_view.py --port COM5 --scale 8     放大 8 倍（最近邻）
    python uart_view.py --file log.txt            解析串口助手保存的日志
    python uart_view.py --selftest                不用硬件，自测脚本链路

依赖: pip install pyserial pillow
"""
import argparse
import sys
import time

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

W, H = 64, 48
HEX_PER_ROW = W * 2          # 128

# 板内盒式平均把灰度压到 0.78 倍（vacc>>7），这里乘回去。
# 199 = 全白方块的理论最大值，于是满量程正好映射到 255。
GAIN_NUM, GAIN_DEN = 255, 199

STAT_BITS = [
    (0, "hdmi_pll"), (1, "ddr_pll"),  (2, "cal_done"), (3, "cal_pass"),
    (4, "cfg_done"), (5, "DVP"),      (6, "any_ack"),  (7, "ver3"),
]

# 2026-10-18 新增：帧头 [5:13] = 相机帧周期，单位 clk_sys 周期 = 1/96MHz
CLK_SYS_HZ = 96_000_000


def decode_tp(hdr):
    """帧头里解析 8 位 hex 帧周期，返回 (period, ms, fps)；没有/非法返回 None。"""
    if len(hdr) < 13:
        return None
    txt = hdr[5:13]
    try:
        period = int(txt, 16)
    except ValueError:
        return None
    if period == 0:
        return None
    ms = period * 1e3 / CLK_SYS_HZ
    return period, ms, CLK_SYS_HZ / period


def decode_status(txt):
    """'AB' -> 'hdmi_pll=1 ddr_pll=1 cal_done=1 cal_pass=1 ...'"""
    try:
        v = int(txt, 16)
    except ValueError:
        return f"(状态字无法解析: {txt!r})"
    parts = [f"{n}={1 if (v >> b) & 1 else 0}" for b, n in STAT_BITS]
    return " ".join(parts)


def render(rows, frm, scale, out, show, quiet=False, raw=False):
    """把 48 行 hex 文本渲染成 PNG。返回 True 表示成功。"""
    from PIL import Image

    px = bytearray()
    for r in rows:
        for i in range(0, HEX_PER_ROW, 2):
            try:
                v = int(r[i:i + 2], 16)
            except ValueError:
                return False
            if not raw:
                v = min(255, v * GAIN_NUM // GAIN_DEN)
            px.append(v)
    if len(px) != W * H:
        return False

    img = Image.frombytes("L", (W, H), bytes(px))
    img = img.resize((W * scale, H * scale), Image.NEAREST)
    img.save(out)
    if not quiet:
        print(f"帧号 {frm} -> {out}")
    if show:
        img.show()
    return True


def run_stream(line_iter, scale, out, show, raw=False):
    """消费一个"行"迭代器，按协议组帧。"""
    rows = []
    frm = "??"
    stat = "??"
    nframes = 0
    last = time.time()
    for ln in line_iter:
        line = ln.strip()
        if line.startswith("@"):
            rows = []
            frm = line[1:3] or "??"
            stat = line[3:5] if len(line) >= 5 else "??"
            print(f"\n帧头: 帧号={frm} 状态[{stat}] {decode_status(stat)}")
            tp = decode_tp(line)
            if tp:
                print(f"      相机帧周期={tp[0]} 拍 ({tp[1]:.2f} ms) ≈ {tp[2]:.2f} fps")
            continue
        if len(line) < HEX_PER_ROW:
            continue
        rows.append(line[:HEX_PER_ROW])
        if len(rows) == H:
            if render(rows, frm, scale, out, show, raw=raw):
                nframes += 1
                now = time.time()
                dt = max(now - last, 1e-6)
                last = now
                print(f"\r帧号 {frm} | 已收 {nframes} 帧 | {1/dt:.2f} fps | 写出 {out}      ",
                      end="", flush=True)
            rows = []
    print()
    return nframes


def lines_from_serial(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=3)
    print(f"已打开 {port} @ {baud}。等待帧头 '@' ...  (Ctrl+C 退出)")
    try:
        while True:
            raw = ser.readline()
            if not raw:
                print("… 3 秒无数据。检查: (1) USB 是否识别成 COM 口 "
                      "(2) 波特率 115200 (3) fpga_txd 是否在 E10")
                continue
            yield raw.decode("ascii", "ignore")
    finally:
        ser.close()


def lines_from_file(path):
    with open(path, "r", errors="ignore", encoding="utf-8") as f:
        for ln in f:
            yield ln


def selftest(scale, out):
    """不用硬件：造一帧梯形渐变 + 边框，走完整解析渲染链路。"""
    lines = ["@5A3F"]
    for y in range(H):
        chars = []
        for x in range(W):
            if x in (0, W - 1) or y in (0, H - 1):
                v = 255
            else:
                v = (x * 4 + y * 5) % 256
            chars.append(f"{v:02X}")
        lines.append("".join(chars))
    print("自测：生成 1 帧模拟数据 -> 解析 -> 渲染")
    n = run_stream(iter(lines), scale, out, False)
    print(f"自测完成：{n} 帧。请查看 {out}（应为一个对角渐变 + 白边框的 64x48 图）")
    return 0 if n == 1 else 1


def main():
    ap = argparse.ArgumentParser(description="FPGA UART 软显示器（64x48 缩略图重建）")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--port", help="串口名，如 COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--scale", type=int, default=8, help="放大倍数（最近邻），默认 8")
    ap.add_argument("--out", default="frame.png", help="输出 PNG 路径，默认 frame.png")
    ap.add_argument("--show", action="store_true", help="每帧另弹窗口（实验性）")
    ap.add_argument("--raw", action="store_true",
                    help="不做 1.28 倍灰度还原（直接用板内 0.78 倍的原始值）")
    ap.add_argument("--file", help="改为解析串口助手保存的文本日志")
    ap.add_argument("--selftest", action="store_true", help="不用硬件，自测")
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.scale, args.out)

    if args.file:
        return 0 if run_stream(lines_from_file(args.file), args.scale,
                               args.out, args.show, args.raw) >= 1 else 1

    try:
        import serial.tools.list_ports as lp
    except ImportError:
        print("缺少 pyserial，请先安装：\n    pip install pyserial pillow")
        return 1

    if args.list or not args.port:
        print("可用串口：")
        for p in lp.comports():
            print(f"  {p.device}   {p.description}")
        if not args.port:
            print("\n用 --port COMx 指定一个（板载 CH340N 一般显示为 "
                  "'USB-SERIAL CH340'）。")
            return 0
        return 0

    try:
        import PIL  # noqa: F401
    except ImportError:
        print("缺少 Pillow，请先安装：\n    pip install pyserial pillow")
        return 1

    try:
        run_stream(lines_from_serial(args.port, args.baud), args.scale,
                   args.out, args.show, args.raw)
    except KeyboardInterrupt:
        print("\n已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
