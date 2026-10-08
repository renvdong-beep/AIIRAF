#!/usr/bin/env bash
# 演示录屏（规范化自本会话的临时 one-liner；2026-10-08）。
#
# 为什么必须有脚本（AGENTS.md 5.2：重复/易错/需人工介入的操作先做成可复跑脚本）：
#   本会话用内联命令录屏时**连续踩两次**：`xwininfo -tree` 的输出行里同时含窗口 id
#   （`0x5200007`）、客户区几何（`1280x720+5+29`）与绝对位置（`+5+355`），
#   用 `grep -oE '[0-9]+x[0-9]+'` 会先命中 **id**（`0x5200007`）⇒ ffmpeg 收到非法
#   `-video_size` 直接失败、白等一场。脚本把"取值 + 校验 + 失败给中文原因"固定下来。
#
# 用法（仓库根）：
#   bash scripts/record_demo.sh [输出路径] [秒数] [--whole-screen]
#     缺省：输出 /home/<user>/iraf_2arm_go2_demo_full.mp4，120 s，整屏 1920x1080
#     --whole-screen  抓整个屏幕（推荐：使用者可手动把 MuJoCo 全屏，画面最大且不会被裁）
#     不加该参数则只抓 MuJoCo 窗口区域（要求窗口未被其它窗口遮挡）
# 退出码：0 成功；2 参数/环境错；3 取窗口几何失败；4 ffmpeg 失败；5 校验失败
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$HOME/iraf_2arm_go2_demo_full.mp4}"
SECS="${2:-120}"
MODE="${3:-window}"           # window | --whole-screen
FPS="${FPS:-30}"
SCENE="${SCENE:-scenes/handoff_lab}"
SCENARIO="${SCENARIO:-nominal}"
WORLD="${WORLD:-joint}"
RENDER_HZ="${RENDER_HZ:-30}"
HOLD_S="${HOLD_S:-200}"       # 仿真结束后窗口保持秒数（要够长：既让使用者能全屏，也够录完）
WAIT_S="${WAIT_S:-20}"        # 开窗后等待（给使用者手动全屏的时间）

[[ -n "${DISPLAY:-}" ]] || { echo "错误：未设置 DISPLAY（本机约定 DISPLAY=:0）" >&2; exit 2; }
command -v ffmpeg >/dev/null || { echo "错误：找不到 ffmpeg" >&2; exit 2; }
command -v xwininfo >/dev/null || { echo "错误：找不到 xwininfo（x11-utils）" >&2; exit 2; }
[[ "$SECS" =~ ^[0-9]+$ ]] && [[ "$SECS" -ge 5 ]] || { echo "错误：秒数必须是 ≥5 的整数（实际 '$SECS'）" >&2; exit 2; }

echo "[录屏] 场景=$SCENE 情景=$SCENARIO 世界=$WORLD 输出=$OUT 时长=${SECS}s 模式=$MODE"
# 1) 起仿真 + 交互窗口（只渲染、时间由 owner 驻留线程推进）
cd "$REPO"
LOG="build/diagnostics/demo-record-$(date +%Y%m%d-%H%M%S).log"
mkdir -p build/diagnostics
( timeout $((SECS + HOLD_S + 120)) env DISPLAY="$DISPLAY" MUJOCO_GL=glfw PYTHONPATH=src \
    python3 scripts/scenario.py run --scene "$SCENE" --scenario "$SCENARIO" --world "$WORLD" \
      --display interactive_viewer --render-hz "$RENDER_HZ" --seconds "$HOLD_S" > "$LOG" 2>&1 ) &
SIM_PID=$!
echo "[录屏] 仿真已起（pid=$SIM_PID，日志 $LOG）；等待 ${WAIT_S}s 供手动全屏"
sleep "$WAIT_S"

# 2) 取录制区域（两条路径都必须**显式校验**，否则给中文原因退出）
if [[ "$MODE" == "--whole-screen" ]]; then
  SIZE="$(DISPLAY="$DISPLAY" xwininfo -root | awk '/Width:/{w=$2} /Height:/{h=$2} END{print w"x"h}')"
  POS="+0+0"
else
  LINE="$(DISPLAY="$DISPLAY" xwininfo -root -tree | grep -i '"MuJoCo' | head -1 || true)"
  [[ -n "$LINE" ]] || { echo "错误：找不到 MuJoCo 窗口（确认仿真已起、DISPLAY=$DISPLAY 正确）" >&2; kill "$SIM_PID" 2>/dev/null || true; exit 3; }
  # 客户区几何是倒数第二个字段（如 1280x720+5+29），绝对位置是最后一个（如 +5+355）
  GEO="$(awk '{print $(NF-1)}' <<<"$LINE")"
  POS="$(awk '{print $NF}' <<<"$LINE")"
  SIZE="$(grep -oE '^[0-9]+x[0-9]+' <<<"$GEO" || true)"
  [[ "$SIZE" =~ ^[0-9]+x[0-9]+$ ]] || { echo "错误：窗口几何解析失败（取到 '$GEO'）：$LINE" >&2; kill "$SIM_PID" 2>/dev/null || true; exit 3; }
fi
[[ "$POS" =~ ^\+[0-9]+\+[0-9]+$ ]] || { echo "错误：窗口位置解析失败（取到 '$POS'）" >&2; kill "$SIM_PID" 2>/dev/null || true; exit 3; }
echo "[录屏] 录制区域 SIZE=$SIZE POS=$POS（窗口行：${LINE:-整屏}）"

# 3) 录屏
ffmpeg -y -f x11grab -video_size "$SIZE" -framerate "$FPS" -i "${DISPLAY}${POS}" \
       -t "$SECS" -pix_fmt yuv420p -movflags +faststart "$OUT" > /tmp/record_demo_ffmpeg.log 2>&1 || {
  echo "错误：ffmpeg 失败，见 /tmp/record_demo_ffmpeg.log（末尾 3 行如下）" >&2
  tail -3 /tmp/record_demo_ffmpeg.log >&2 || true; exit 4; }
[[ -s "$OUT" ]] || { echo "错误：输出文件为空：$OUT" >&2; exit 4; }

# 4) 校验（程序化，不依赖视觉）：抽帧差分必须证明"画面在动"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
ffmpeg -y -ss 1 -i "$OUT" -frames:v 1 "$TMP/a.png" -loglevel error
ffmpeg -y -ss "$((SECS/2))" -i "$OUT" -frames:v 1 "$TMP/b.png" -loglevel error
python3 - "$TMP/a.png" "$TMP/b.png" <<'PY' || exit 5
import sys
from PIL import Image
import numpy as np
a=np.asarray(Image.open(sys.argv[1]).convert('RGB'),dtype=np.int16)
b=np.asarray(Image.open(sys.argv[2]).convert('RGB'),dtype=np.int16)
mv=100.0*float((np.abs(a-b).max(axis=2)>8).mean())
print("[录屏] 帧差校验：变化像素 %.2f%%" % mv)
if mv < 0.5:
    print("[录屏] 校验失败：画面几乎静止 ⇒ 可能录到了空桌面或被遮挡的窗口", file=sys.stderr); sys.exit(5)
PY

echo "[录屏] 完成：$OUT（$(du -h "$OUT" | cut -f1)）"
# 5) 仓库内留一份备份（本会话踩过：家目录里的 mp4 被外部清掉了一次）
mkdir -p build/diagnostics && cp -f "$OUT" build/diagnostics/ && echo "[录屏] 备份：build/diagnostics/$(basename "$OUT")"
