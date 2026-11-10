#!/usr/bin/env bash
# 板卡侧注册表对齐（0.4.3）：joint 改名 handoff_lab_joint、删除失败的 piper-pick-y05。
# 何时用：x86 的 manifest 已按 handoff_lab_joint 生成（2026-11-10），而板卡注册表仍是旧名。
# 用法（板卡 root）：bash board-align-registry-0.4.3.sh [--dry-run]
# 退出码：0 成功 / 1 参数错误 / 2 校验失败
set -uo pipefail
REG=${REG:-/etc/iraf}
DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

echo "== 1) 删除 piper-pick-y05（失败适配）"
for f in "$REG/scenes/piper-pick-y05.yaml" "$REG/providers/iraf-piper-pick-y05.yaml"; do
  [ -f "$f" ] || { echo "   跳过（不存在）：$f"; continue; }
  [ "$DRY" = 1 ] && echo "   [dry-run] 将删除 $f" || { mv "$f" "$f.removed-$(date +%Y%m%d%H%M%S)"; echo "   已移走 $f"; }
done

echo "== 2) joint-handoff 的 switch_name -> handoff_lab_joint"
S="$REG/scenes/joint-handoff.yaml"
if [ -f "$S" ]; then
  grep -q '^switch_name: handoff_lab_joint$' "$S" && echo "   已是新名，无需改" || {
    [ "$DRY" = 1 ] && echo "   [dry-run] 将改 $S 的 switch_name" || {
      sed -i 's/^switch_name: joint$/switch_name: handoff_lab_joint/' "$S"
      grep -q '^aliases:' "$S" || sed -i 's/^switch_name: handoff_lab_joint$/switch_name: handoff_lab_joint\naliases: [joint]/' "$S"
      echo "   已改写 $S"
    }
  }
else
  echo "   缺少 $S" >&2; exit 2
fi

echo "== 3) 重新封签 + 校验"
[ "$DRY" = 1 ] && { echo "   [dry-run] 跳过 --seal 与校验"; exit 0; }
python3 /root/iraf-instance/scripts/registry_check.py --root "$REG" --seal
python3 /root/iraf-instance/scripts/registry_check.py --root "$REG" --profiles-root /opt/iraf-sdk/0.4.3/profiles
rc=$?
echo "== 4) 重启 MCP（会打断在用 chat 会话，按运维规程同时重启 dashboard）"
[ $rc -eq 0 ] && { systemctl restart iraf-mcp-http; systemctl restart agentos-dashboard; echo "   已重启 iraf-mcp-http + agentos-dashboard"; } || echo "   校验未过，未重启" >&2
exit $rc
