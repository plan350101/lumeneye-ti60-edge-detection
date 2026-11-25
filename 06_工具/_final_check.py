# -*- coding: utf-8 -*-
"""提交前一键判读：抓串口 -> 检查 v2/v3 遥测帧头 -> PASS/FAIL。

用法:
    python _final_check.py [秒数]      # 默认 6 秒

通过条件（两套 bit 通用）:
    状态字   == 0xFF
    相机周期 == 3,200,448 拍 (30.00 fps, ±0.5%)
    显示周期 == 1,600,000 拍 (60.00 Hz, ±0.1%)
    hold     == 2
    至少解析到 1 帧
退出码: 0 = PASS, 1 = FAIL
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

SECS = 0.0
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
try:
    SECS = float(_args[0]) if _args else 6.0
except ValueError:
    SECS = 6.0
PNG = "--png" in sys.argv
CLK = 96_000_000
EXP_CAM = 3_200_448      # 30.00 fps
EXP_DISP = 1_600_000     # 60.00 Hz
EXP_HOLD = 2
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
        if is_ch340:
            print("  找到 CH340: %s | %s" % (p.device, desc))
            cands.append(p.device)
    return cands


def main():
    print("=== 提交前一键判读 ===")
    cands = find_ch340()
    if not cands:
        print("\n[FAIL] 没找到 CH340：检查串口 Mini-USB、驱动、板子是否上电。")
        return 1
    port = cands[0]
    print("使用 %s，抓 %.1f s ..." % (port, SECS))
    ser = serial.Serial(port, 115200, timeout=0.2)
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < SECS:
        d = ser.read(8192)
        if d:
            buf += d
    ser.close()

    txt = bytes(buf).decode("ascii", "ignore")
    if not buf:
        print("\n[FAIL] 0 字节：先怀疑 SRAM 配置丢了，重烧 .bit。")
        return 1

    # 原始文本落盘，便于 _shot.py / _tp_stat.py 复查
    try:
        with open(os.path.join(HERE, "_final_check_raw.txt"), "w", encoding="utf-8", newline="") as f:
            f.write(txt)
    except Exception:
        pass

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
        print("\n[FAIL] 没解析到遥测帧头（收到 %d 字节）。检查烧的是最新 bit。" % len(buf))
        return 1

    # 取最后 8 帧做判据（避开上电/切换瞬态）
    tail = rows[-8:] if len(rows) >= 8 else rows
    st_ok = all(r[1] == 0xFF for r in tail)
    cam_ok = all(abs(r[2] - EXP_CAM) <= EXP_CAM * 0.005 for r in tail)
    disp_ok = all(abs(r[4] - EXP_DISP) <= EXP_DISP * 0.001 for r in tail)
    hold_ok = all(r[5] == EXP_HOLD for r in tail)
    cam_fps = [CLK / r[2] for r in tail if r[2]]
    disp_hz = [CLK / r[4] for r in tail if r[4]]
    hold_vals = sorted(set(r[5] for r in tail))

    print("\n 帧数: %d（判据用最后 %d 帧）" % (len(rows), len(tail)))
    print(" 状态字   : %s  期望 FF" % (" ".join("%02X" % r[1] for r in tail[-4:])))
    print(" 相机周期 : %d 拍 (%.2f fps)  期望 %d (30.00fps)" % (tail[-1][2], cam_fps[-1] if cam_fps else 0, EXP_CAM))
    print(" 显示周期 : %d 拍 (%.2f Hz)   期望 %d (60.00Hz)" % (tail[-1][4], disp_hz[-1] if disp_hz else 0, EXP_DISP))
    print(" hold     : %s                     期望 2" % hold_vals)

    checks = [
        ("状态字 FF", st_ok),
        ("相机 30.00fps", cam_ok),
        ("显示 60.00Hz", disp_ok),
        ("hold=2", hold_ok),
    ]
    print("\n=== 判读 ===")
    for name, ok in checks:
        print(" [%s] %s" % ("PASS" if ok else "FAIL", name))
    all_ok = all(ok for _, ok in checks)
    print("\n" + ("==> PASS：这块 bit 的帧率/显示/状态全部正常。" if all_ok
                  else "==> FAIL：有不达标项，见上表。"))

    # 可选：从同一段串口数据里渲染一张 64×48 缩略图（红边=红，饱和=白）
    if PNG:
        frames = []
        cur = None
        for line in txt.replace("\r", "\n").split("\n"):
            s = line.strip()
            if s.startswith("@") and len(s) >= 5:
                cur = {"hdr": s, "rows": []}
                frames.append(cur)
            elif cur is not None and len(s) == 128:
                try:
                    cur["rows"].append([int(s[i:i + 2], 16) for i in range(0, 128, 2)])
                except ValueError:
                    pass
        frames = [f for f in frames if len(f["rows"]) == 48]
        if frames:
            best = max(frames, key=lambda f: sum(1 for r in f["rows"] for v in r if v == 0xE0))
            R = best["rows"]
            from PIL import Image
            im = Image.new("RGB", (64, 48))
            px = []
            for r in R:
                for v in r:
                    if v == 0xE0:
                        px.append((230, 20, 20))
                    elif v >= 250:
                        px.append((255, 255, 255))
                    else:
                        px.append((v, v, v))
            im.putdata(px)
            im.resize((512, 384), Image.NEAREST).save(os.path.join(HERE, "_final_check_red.png"))
            g = Image.new("L", (64, 48))
            g.putdata([v for r in R for v in r])
            g.resize((512, 384), Image.NEAREST).save(os.path.join(HERE, "_final_check_gray.png"))
            print("缩略图已写 _final_check_red.png / _final_check_gray.png（帧 %s）" % best["hdr"])
        else:
            print("未解析到完整 64×48 缩略图，跳过 PNG。")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
