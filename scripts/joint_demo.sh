#!/usr/bin/env bash
# 联合世界（Go2 + Piper + UR5e）9 步链路 + MuJoCo 窗口演示。
#
# 用法：joint_demo.sh [hold_s] [render_hz]
#   hold_s     链路结束后窗口再保持的秒数（0=立即关窗）
#   render_hz  窗口渲染频率（默认 10Hz；过高会与物理线程抢核）
#
# 为什么单独一层：run_full_scenario.sh 的 window 分支需要"边跑边看"的入口，
# 而 scenario.py 的 --display interactive_viewer 是**只渲染不推进**（时间由 owner
# 驻留线程推进），必须显式给出 render-hz 与结束后保持时间。
set -u
HOLD=${1:-60}
HZ=${2:-10}
UID_=$(id -u)

# X display 自动解析：桌面进程参数优先，其次 /tmp/.X11-unix socket，最后回落 :0
resolve_display() {
  local n
  n=$(ps -u "$UID_" -o args= 2>/dev/null | grep -oE -- '--display :[0-9]+' \
      | grep -oE '[0-9]+' | sort -n | head -n1)
  [ -n "$n" ] && { printf ':%s' "$n"; return 0; }
  for s in /tmp/.X11-unix/X*; do
    [ -S "$s" ] || continue
    n=${s##*X}
    case "$n" in ''|*[!0-9]*) continue ;; esac
    printf ':%s' "$n"; return 0
  done
  printf ':0'
}

export XDG_RUNTIME_DIR="/run/user/${UID_}"
# 上层脚本（run_full_scenario.sh）可能已经导出了过期的 DISPLAY=:0：
# 只有 socket 真的存在才沿用，否则重新解析。
if [ -n "${DISPLAY:-}" ] && [ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
  :
else
  DISPLAY="$(resolve_display)"
fi
export DISPLAY
export XAUTHORITY="${XAUTHORITY:-/run/user/${UID_}/gdm/Xauthority}"
[ -r "$XAUTHORITY" ] || XAUTHORITY="/run/user/${UID_}/gdm/Xauthority"
export XAUTHORITY

cd /home/coretek/AIIRAF || exit 2
export PYTHONPATH=src
echo "[joint_demo] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY hold=${HOLD}s render_hz=${HZ}"
exec python3 scripts/scenario.py run \
  --scene scenes/handoff_lab --scenario nominal \
  --world joint \
  --display interactive_viewer --render-hz "$HZ" --seconds "$HOLD"