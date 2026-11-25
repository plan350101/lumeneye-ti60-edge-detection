# -*- coding: utf-8 -*-
"""遥测统计 v3：相机帧周期 / 行周期 / 显示帧周期 / hold / DDR读指针切换周期。

用法:
    python _tp_stat.py [_uart_raw.txt]

帧头格式 (2026-10-18 v3):
    '@' + 帧号(2) + 状态(2)
        + 相机帧周期(8) + 行周期(6) + 显示帧周期(6) + hold(2) + 读指针周期(8) + CRLF
全部是 clk_sys 96MHz 计数（1 拍 = 10.4167 ns）。

关键判据:
  显示帧周期 ≈ 1.60M          -> 60Hz（同时证明 clk_sys=96MHz）
  读指针切换周期 ≈ 1.9M       -> 屏幕内容 ~51Hz 刷新
  读指针切换周期 ≈ 6.4M       -> 屏幕内容 ~15Hz 刷新（R3：读侧 4:1 保持）
  hold = 1~2                  -> 相机帧间隔 1~2 个显示帧（源 ~51fps）
"""
import re
import statistics
import sys

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

CLK = 96_000_000
HTS, VTS = 1896, 984
PATH = sys.argv[1] if len(sys.argv) > 1 else "_uart_raw.txt"

HDR = re.compile(r"^@([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})"
                 r"([0-9A-Fa-f]{8})([0-9A-Fa-f]{6})([0-9A-Fa-f]{6})"
                 r"([0-9A-Fa-f]{2})([0-9A-Fa-f]{8})?")
HDR6 = re.compile(r"^@([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})"
                  r"([0-9A-Fa-f]{8})([0-9A-Fa-f]{6})([0-9A-Fa-f]{6})"
                  r"([0-9A-Fa-f]{2})")


def load(path):
    rows = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            m = HDR.match(s)
            if m and m.group(7):
                rows.append(tuple(int(m.group(i), 16) for i in range(1, 8)))
            elif HDR6.match(s):
                m = HDR6.match(s)
                rows.append(tuple(int(m.group(i), 16) for i in range(1, 7)) + (0,))
    return rows


def med(xs):
    return statistics.median(xs) if xs else 0


def main():
    rows = load(PATH)
    print("=" * 92)
    print("遥测统计 v3  文件:", PATH)
    print("=" * 92)
    if not rows:
        print("没有解析到 v3 帧头。检查是否烧了最新 camera_hdmi_tp.bit / 串口是否抓到完整行。")
        return
    print("帧头数: %d   帧号: %d..%d   状态字: %s"
          % (len(rows), rows[0][0], rows[-1][0],
             " ".join(sorted({"0x%02X" % r[1] for r in rows}))))

    print("\n 帧号 状态   相机周期   行周期  显示周期 hold  相机fps  PCLK(MHz) 行/帧  读切周期  刷新Hz")
    for r in rows:
        fn, st, cam, ln, dp, hd, rp = r
        fps = CLK / cam if cam else 0
        pclk = CLK * HTS / ln / 1e6 if ln else 0
        lpf = cam / ln if ln else 0
        rhz = CLK / rp if rp else 0
        print("  %02X  0x%02X  %9d  %7d  %7d  %4d  %7.2f  %8.2f  %5.1f  %8d  %6.2f"
              % (fn, st, cam, ln, dp, hd, fps, pclk, lpf, rp, rhz))

    cams = [r[2] for r in rows if r[2]]
    lns = [r[3] for r in rows if r[3]]
    dps = [r[4] for r in rows if r[4]]
    hds = [r[5] for r in rows]
    rps = [r[6] for r in rows if r[6]]

    if cams:
        fps = [CLK / c for c in cams]
        print("\n--- 相机帧率 ---")
        print("fps: min=%.2f median=%.2f max=%.2f  (周期 median=%d 拍)"
              % (min(fps), med(fps), max(fps), int(med(cams))))
    if lns:
        print("\n--- 行周期 ---")
        print("line: median=%d 拍 => PCLK median=%.2f MHz"
              % (int(med(lns)), CLK * HTS / med(lns) / 1e6))
    if dps:
        print("\n--- 显示帧周期 ---")
        print("disp: median=%d 拍 => %.2f Hz" % (int(med(dps)), CLK / med(dps)))
    if hds:
        print("\n--- hold 分布 ---")
        print({v: hds.count(v) for v in sorted(set(hds))})
    if rps:
        print("\n--- DDR 读指针切换周期（= 屏幕内容实际刷新）---")
        print("周期: min=%d median=%d max=%d 拍" % (min(rps), int(med(rps)), max(rps)))
        print("刷新率: min=%.2f median=%.2f max=%.2f Hz"
              % (CLK / max(rps), CLK / med(rps), CLK / min(rps)))

    print("\n--- 判读 ---")
    if lns:
        pclk = CLK * HTS / med(lns) / 1e6
        print("PCLK ≈ %.2f MHz" % pclk)
    if dps:
        print("显示帧率 ≈ %.2f Hz %s"
              % (CLK / med(dps), "(clk_sys=96MHz 自洽)" if 1.3e6 < med(dps) < 1.9e6 else "(异常)"))
    if rps:
        hz = CLK / med(rps)
        if 40 < hz < 65:
            print("★ 屏幕内容刷新 ≈ %.1f Hz => 读侧基本每帧都在换，抽动不是 4:1 保持。" % hz)
        elif 10 < hz < 25:
            print("★ 屏幕内容刷新 ≈ %.1f Hz => 读侧掉帧！R3 成立（每 ~%.1f 个显示帧才换一次）。"
                  % (hz, 60.0 / hz))
        else:
            print("★ 屏幕内容刷新 ≈ %.1f Hz（非典型）" % hz)
        if cams:
            print("   相机 %.1f fps vs 读侧切换 %.1f Hz  %s"
                  % (CLK / med(cams), hz,
                     "=> 读侧没跟上相机（问题在读侧）" if hz < 0.8 * CLK / med(cams)
                     else "=> 读侧跟上了相机"))
    if cams and lns:
        print("\n自洽性: cam/line = %.1f （若 line 量的是 1 行，应 ≈ VTS=%d；若 2 行则 ≈ VTS/2=492）"
              % (med(cams) / med(lns), VTS))


if __name__ == "__main__":
    main()
