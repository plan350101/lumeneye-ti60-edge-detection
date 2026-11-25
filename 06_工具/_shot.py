# -*- coding: utf-8 -*-
"""一次性判读：逐帧统计表 + 挑红边最多的一帧渲染灰阶/伪彩两张图。"""
import sys
from PIL import Image

PATH = sys.argv[1] if len(sys.argv) > 1 else "_uart_raw.txt"
PRE = sys.argv[2] if len(sys.argv) > 2 else "shot"


def load(path):
    frames, cur = [], None
    for l in open(path, encoding="utf-8", errors="ignore"):
        l = l.rstrip("\r\n")
        if l.startswith("@") and len(l) >= 5:
            cur = {"hdr": l, "rows": []}
            frames.append(cur)
        elif cur is not None and len(l) == 128:
            try:
                cur["rows"].append([int(l[i:i + 2], 16) for i in range(0, 128, 2)])
            except ValueError:
                pass
    return [f for f in frames if len(f["rows"]) == 48]


def corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    za = [v - ma for v in a]
    zb = [v - mb for v in b]
    num = sum(za[i] * zb[i] for i in range(n))
    da = sum(v * v for v in za) ** 0.5
    db = sum(v * v for v in zb) ** 0.5
    return num / (da * db) if da > 1e-9 and db > 1e-9 else float("nan")


fs = load(PATH)
print("完整帧数 %d  帧号 %s" % (len(fs), " ".join(f["hdr"][1:5] for f in fs)))
print("\n帧     均值   饱和255  红边E0  横相关  纵相关")
for f in fs:
    R = f["rows"]
    allv = [v for r in R for v in r]
    red = sum(1 for v in allv if v == 0xE0)
    sat = sum(1 for v in allv if v == 255)
    hc = [corr(R[i][:-1], R[i][1:]) for i in range(48)]
    vc = [corr(R[i], R[i + 1]) for i in range(47)]
    hc = [x for x in hc if x == x]
    vc = [x for x in vc if x == x]
    print("%s %6.1f  %6d  %6d  %+0.3f  %+0.3f" % (f["hdr"], sum(allv) / 3072, sat, red,
                                                  sum(hc) / len(hc), sum(vc) / len(vc)))

# 挑"红边最多"的帧做展示；若全都没红边就挑最接近中位亮度的那帧
best = max(fs, key=lambda f: sum(1 for r in f["rows"] for v in r if v == 0xE0))
nred = sum(1 for r in best["rows"] for v in r if v == 0xE0)
print("\n展示帧 %s（红边 %d）" % (best["hdr"], nred))
R = best["rows"]

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
im.resize((512, 384), Image.NEAREST).save(PRE + "_red.png")

g = Image.new("L", (64, 48))
g.putdata([v for r in R for v in r])
g.resize((512, 384), Image.NEAREST).save(PRE + "_gray.png")
print("已写 %s_red.png / %s_gray.png" % (PRE, PRE))

# 红边包围盒 + 重心，用来回答"物体在画面哪个位置"
pts = [(i, k) for i, r in enumerate(R) for k, v in enumerate(r) if v == 0xE0]
if pts:
    ys = [p[0] for p in pts]
    xs = [p[1] for p in pts]
    print("红边包围盒: 行 %d..%d  列 %d..%d" % (min(ys), max(ys), min(xs), max(xs)))
    print("红边重心: 行 %.1f / 47   列 %.1f / 63  (0=上/左)" % (sum(ys) / len(ys), sum(xs) / len(xs)))
    print("包围盒占窗口: 高 %.0f%%  宽 %.0f%%" % (100.0 * (max(ys) - min(ys) + 1) / 48,
                                                 100.0 * (max(xs) - min(xs) + 1) / 64))

print("\n=== 展示帧 1:1 字符画 (64x48) ===")
chars = " .:-=+*#%@"
for i, r in enumerate(R):
    print("%2d %3.0f|%s|" % (i, sum(r) / 64, "".join(chars[min(9, v * 10 // 256)] for v in r)))
