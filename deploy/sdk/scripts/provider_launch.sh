#!/usr/bin/env bash
# IRAF 仿真 Provider 启动器：解析当前可用的 X display 后 exec 包内入口。
#
# 为什么需要它：provider 的 DISPLAY/XAUTHORITY 是**装配单里的静态值**，而 GDM 每次重启
# 可能把 X 分到不同的 display 号（实测 :0 → :1）。静态值一旦过期，窗口打不开，
# MuJoCo 只能退回软渲染或直接失败。本脚本在启动瞬间解析真值，装出来的单元就不会再被
# display 号绑死。
#
# 解析顺序：
#   1) 运行中的桌面进程参数里的 --display :N（xfwm4 / xfsettingsd / xfce4-panel / gnome-shell）
#   2) /tmp/.X11-unix/X<N> 中存在且属主的 socket（取 display 号最小的）
#   3) 回落到装配单里的 DISPLAY
set -u

current_uid=$(id -u)

pick_from_desktop() {
  local n
  n=$(ps -u "$current_uid" -o args= 2>/dev/null \
      | grep -oE -- '--display :[0-9]+' \
      | grep -oE '[0-9]+' | sort -n | head -n1)
  [ -n "$n" ] && { printf ':%s' "$n"; return 0; }
  return 1
}

pick_from_socket() {
  local s n
  for s in /tmp/.X11-unix/X*; do
    [ -S "$s" ] || continue
    n=${s##*X}
    case "$n" in ''|*[!0-9]*) continue ;; esac
    printf ':%s' "$n"
    return 0
  done
  return 1
}

resolved=$(pick_from_desktop) || resolved=$(pick_from_socket) || resolved="${DISPLAY:-}"

if [ -n "$resolved" ]; then
  socket="/tmp/.X11-unix/X${resolved#:}"
  if [ -S "$socket" ]; then
    export DISPLAY="$resolved"
  else
    echo "provider_launch: display $resolved 无 socket，沿用 $DISPLAY" >&2
  fi
fi

if [ -z "${XAUTHORITY:-}" ] || [ ! -r "${XAUTHORITY:-/nonexistent}" ]; then
  for candidate in "/run/user/$current_uid/gdm/Xauthority" "$HOME/.Xauthority"; do
    if [ -r "$candidate" ]; then export XAUTHORITY="$candidate"; break; fi
  done
fi

echo "provider_launch: DISPLAY=${DISPLAY:-} XAUTHORITY=${XAUTHORITY:-}" >&2
exec "${IRAF_PYTHON_BIN:-/usr/bin/python3}" -m iraf_adapters.remote.provider_cli