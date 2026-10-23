#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判断截图是不是黑屏/空窗（不依赖视觉模型）：整图 + 指定矩形区域的平均灰度与近黑占比。

用法：
  python3 scripts/img_luma.py build/logs/frame.png
  python3 scripts/img_luma.py build/logs/frame.png --region 5,29,1280,720
判据（经验值）：MuJoCo 正常渲染的窗口区域平均灰度 > 40 且近黑像素 < 60%；
全黑/未渲染时平均灰度 < 10。
"""
import sys

try:
    from PIL import Image
except ImportError:
    raise SystemExit("缺 Pillow：pip install pillow（本机已装可跳过）")

path = sys.argv[1]
region = None
if "--region" in sys.argv:
    x, y, w, h = (int(v) for v in sys.argv[sys.argv.index("--region") + 1].split(","))
    region = (x, y, x + w, y + h)

im = Image.open(path).convert("L")
target = im.crop(region) if region else im
hist = target.histogram()
total = sum(hist) or 1
mean = sum(i * c for i, c in enumerate(hist)) / total
dark = sum(hist[:16]) * 100.0 / total
bright = sum(hist[128:]) * 100.0 / total
label = "区域" if region else "整图"
print("%s %s：尺寸=%dx%d 平均灰度=%.1f 近黑(<16)占比=%.1f%% 亮(>=128)占比=%.1f%%"
      % (label, region or "全屏", target.size[0], target.size[1], mean, dark, bright))
print("判读：", "正常有内容" if mean > 40 and dark < 60 else ("偏暗/可能是空窗" if mean > 10 else "疑似黑屏"))
