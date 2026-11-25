# -*- coding: utf-8 -*-
"""一键探针：找 CH340 -> 抓 N 秒串口 -> 解析 v3 遥测帧头 -> 简要判读。

帧头: '@' + 帧号(2) + 状态(2) + 相机周期(8) + 行周期(6) + 显示周期(6)
      + hold(2) + 读指针切换周期(8)
"""
import os
import re
import sys
import time

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import serial                          # noqa: E402
import serial.tools.list_ports as lp   # noqa: E402

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "_uart_raw.txt")
CLK = 96_000_000
HTS = 1896
HDR = re.compile(r"^@([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})"
                 r"([0-9A-Fa-f]{8})([0-9A-Fa-f]{6})([0-9A-Fa-f]{6})"
                 r"([0-9A-Fa-f]{2})([0-9A-Fa-f]{8})?")
HDR6 = re.compile(r"^@([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})"
                  r"([0-9A-Fa-f]{8})([0-9A-Fa-f]{6})([0-9A-Fa-f]{6})"
                  r"([0-9A-Fa-f]{2})")


def find_ch340():
    cands = []
    for p in lp.comports():
        desc = " ".join(x for x in (p.description, p.product, p.manufacturer, p.hwid) if x)
        is_ch340 = ("CH340" in desc.upper()) or ("CH341" in desc.upper()) \
                   or (p.vid == 0x1A86 and p.pid == 0x7523)
        print("  %-6s | %s%s" % (p.device, desc, "  <== CH340" if is_ch340 else ""))
        if is_ch340:
            cands.append(p.device)
    return cands


def main():
    print("=== 串口枚举 ===")
    cands = find_ch340()
    if not cands:
        print("\n[FAIL] 没找到 CH340：检查串口 Mini-USB、驱动、板子是否上电。")
        return
    port = cands[0]
    print("\n使用 %s，抓 %.1f s ..." % (port, SECS))
    ser = serial.Serial(port, 115200, timeout=0.2)
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < SECS:
        d = ser.read(8192)
        if d:
            buf += d
    ser.close()

    txt = bytes(buf).decode("ascii", "ignore")
    with open(OUT, "w", encoding="utf-8", newline="") as f:
        f.write(txt)
    print("收到 %d 字节 -> %s" % (len(buf), OUT))
    if not buf:
        print("\n[FAIL] 0 字节：先怀疑 SRAM 配置丢了，重烧 .bit。")
        return
    print("开头预览: %r" % txt[:160].replace("\r", "\\r").replace("\n", "\\n"))

    rows = []
    for line in txt.replace("\r", "\n").split("\n"):
        s = line.strip()
        m = HDR.match(s)
        if m and m.group(7):
            rows.append(tuple(int(m.group(i), 16) for i in range(1, 8)))
        else:
            m = HDR6.match(s)
            if m:
                rows.append(tuple(int(m.group(i), 16) for i in range(1, 7)) + (0,))
    if not rows:
        print("\n[FAIL] 没解析到 v3 帧头。请确认烧的是最新 camera_hdmi_tp.bit。")
        return

    print("\n=== 遥测 ===")
    print(" 帧号 状态   相机周期   行周期  显示周期 hold 读切周期  相机fps 刷新Hz  PCLK")
    for fn, st, cam, ln, dp, hd, rp in rows:
        print("  %02X  0x%02X  %9d  %7d  %7d  %4d  %8d  %7.2f  %6.2f  %5.2f"
              % (fn, st, cam, ln, dp, hd, rp,
                 CLK / cam if cam else 0,
                 CLK / rp if rp else 0,
                 CLK * HTS / ln / 1e6 if ln else 0))

    print("\n完整统计请跑: python _tp_stat.py", OUT)


if __name__ == "__main__":
    main()
