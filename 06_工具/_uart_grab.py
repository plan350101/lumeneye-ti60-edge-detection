# -*- coding: utf-8 -*-
"""有界抓帧：读串口 N 秒 -> 存原始文本 -> 复用 uart_view 解析/渲染。
用法: python _uart_grab.py [COM口] [秒数] [输出png]
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import serial            # noqa: E402
import uart_view as uv   # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "frame.png")

    ser = serial.Serial(port, 115200, timeout=0.2)
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < secs:
        d = ser.read(8192)
        if d:
            buf += d
    ser.close()

    txt = bytes(buf).decode("ascii", "ignore")
    rawp = os.path.join(HERE, "_uart_raw.txt")
    with open(rawp, "w", encoding="utf-8", newline="") as f:
        f.write(txt)

    lines = [l for l in txt.replace("\r", "\n").split("\n")]
    hdrs = [l.strip() for l in lines if l.strip().startswith("@")]
    print("=== 抓帧报告 ===")
    print("读 %s %.0f s：%d 字节，帧头 %d 个" % (port, secs, len(buf), len(hdrs)))
    for h in hdrs[:8]:
        if len(h) >= 5:
            print("  帧号=%s 状态字=%s  ->  %s"
                  % (h[1:3], h[3:5], uv.decode_status(h[3:5])))
            tp = uv.decode_tp(h)
            if tp:
                print("      相机帧周期=%d 拍 (%.2f ms) ≈ %.2f fps"
                      % (tp[0], tp[1], tp[2]))
        else:
            print("  帧头(截断)=%r" % h)
    print("--- run_stream ---")
    n = uv.run_stream(iter(lines), 8, out, False)
    print("=== 完成：渲染 %d 帧 -> %s ===" % (n, out))
    print("原始文本存于:", rawp)


main()
