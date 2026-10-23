#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AgentOS **内容层**处置（不动框架/代码/容器，全部可回滚）。

做三件事：
1) 停用旁路 skill `iraf-mcp-task-submit`
   它是"用 host_command 跑脚本、在 shell 里再拉起我们的 stdio MCP"的旧路径，
   还写着板上已经不存在的 /opt/iraf-extras/*；模型会优先照它做，于是绕开任务级 MCP 工具。
   处理：整目录移动到**固定**备份区（不删除，可原样搬回）。
2) 把路由守则写进 SOUL.md（AgentOS 的 agent 身份/系统提示文件，每会话生效）。
   写入前备份 SOUL.md；已含则跳过（幂等）。
3) `--status` 只读体检：列出旁路 skill / SOUL 守则 / 备份区现状。

为什么收进仓库（2026-10-23 实机教训）：厂商换包（d20588cd → 8820f5a8）会把
/root/.intewell 清成空壳，手工在板上改的东西连同"改它的脚本"一起消失。
所以这一步必须在仓库里、可复跑、参数全在命令行/环境里给。

用法（**在开发机**跑，经 ssh stdin 送远端 python3 执行，不在板上留脚本）：
  sshpass -e ssh root@<板> "python3 - --apply"  < scripts/agentos_content_layer.py
  sshpass -e ssh root@<板> "python3 - --revert" < scripts/agentos_content_layer.py
  sshpass -e ssh root@<板> "python3 - --status" < scripts/agentos_content_layer.py
包装脚本见 scripts/agentos_content_layer.sh。
"""
import shutil
import sys
from pathlib import Path

HOME = Path("/root/.intewell")
SKILL = HOME / "skills" / "iraf-mcp-task-submit"
SOUL = HOME / "SOUL.md"
# 固定备份区：不再按时间戳堆目录，重复执行可直接覆盖同一份，revert 也不会"还原到更早的版本"
BACKUP = Path("/root/intewell-content-backup-current")
SKILL_BACKUP = BACKUP / SKILL.name
SOUL_BACKUP = BACKUP / "SOUL.md"

MARK = "机器人开发模式（IRAF）"

ROUTING = """

## 机器人开发模式（IRAF）

- 用户说「开始机器人开发模式」或「进入机器人开发模式」时：**立即**调用
  `mcp__iraf_task__iraf_enter_robot_mode`，不要先做环境自检、不要自己检查仓库。
- 该模式生效期间：
  1. 凡**可能涉及机器人本体或物理动作**的请求（移动、抓取、放置、标定、视觉、停止/急停），
     **必须先调用** `mcp__iraf_task__iraf_submit_task(text=<用户原话>)`，由 IRAF 的
     Skill Manager 选技能并执行；**禁止**用 execute_code / terminal / host_command 自行实现、
     自行驱动或"重新写一遍抓取"。
  2. 若返回 `error_code=IRAF-NOT-ROBOT-TASK`，说明该请求与机器人无关，按你自己的工具与流程处理；
     不要重试该工具，也不要把失败包装成成功。
  3. 平台自身事务（项目、代码、文件、构建、对话）由你判断处理，**不要**交给 IRAF。
  4. 核对执行事实时调用 `mcp__iraf_task__iraf_get_task(execution_id)`，不要采信模型自述。
  5. 失败先读返回里的 `next_steps`；**不要**修改仓内源码或重写抓取逻辑。
     MuJoCo 场景有状态：成功抓取会把目标带走，下一次抓取前需要复位场景（重启 Provider），
     否则会报 `distance≈0.374m` —— 那是环境状态，不是实现 bug。
- 用户说「结束机器人开发模式」时：调用 `mcp__iraf_task__iraf_exit_robot_mode`。
"""


def status() -> int:
    print("旁路 skill :", "仍在" if SKILL.is_dir() else "已停用", SKILL)
    if SOUL.is_file():
        has = MARK in SOUL.read_text(encoding="utf-8", errors="replace")
        print("SOUL.md    :", "含路由守则" if has else "**不含**路由守则", SOUL)
    else:
        print("SOUL.md    : 不存在", SOUL)
    print("备份区     :", "存在" if BACKUP.is_dir() else "不存在", BACKUP)
    return 0


def apply() -> int:
    BACKUP.mkdir(parents=True, exist_ok=True)
    if SKILL.is_dir():
        if SKILL_BACKUP.exists():
            shutil.rmtree(SKILL_BACKUP)   # 旧备份让位给当前版本，避免 revert 还原到过期内容
        shutil.move(str(SKILL), str(SKILL_BACKUP))
        print("已停用旁路 skill →", SKILL_BACKUP)
    else:
        print("旁路 skill 不在（已停用或本版本未生成）：", SKILL)

    if SOUL.is_file():
        text = SOUL.read_text(encoding="utf-8", errors="replace")
        if MARK in text:
            print("SOUL.md 已含路由守则，跳过追加")
        else:
            if not SOUL_BACKUP.exists():
                shutil.copy2(SOUL, SOUL_BACKUP)
            SOUL.write_text(text.rstrip() + "\n" + ROUTING, encoding="utf-8")
            print("已把路由守则追加到 SOUL.md（原文件备份在 %s）" % SOUL_BACKUP)
    else:
        # 没有 SOUL.md 就新建：内容层的守则不能因为"文件不存在"而静默失效
        SOUL.parent.mkdir(parents=True, exist_ok=True)
        SOUL.write_text(ROUTING.lstrip("\n"), encoding="utf-8")
        print("SOUL.md 不存在 → 已新建并写入路由守则：", SOUL)
    print("备份区：", BACKUP)
    return 0


def revert() -> int:
    ok = False
    if SOUL_BACKUP.is_file():
        shutil.copy2(SOUL_BACKUP, SOUL)
        print("已还原 SOUL.md ←", SOUL_BACKUP)
        ok = True
    if SKILL_BACKUP.is_dir():
        target = HOME / "skills" / SKILL.name
        if target.exists():
            shutil.rmtree(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(SKILL_BACKUP), str(target))
        print("已恢复旁路 skill ←", target)
        ok = True
    if not ok:
        print("没有可还原的备份：", BACKUP)
        return 2
    return 0


if __name__ == "__main__":
    if "--status" in sys.argv:
        raise SystemExit(status())
    if "--revert" in sys.argv:
        raise SystemExit(revert())
    raise SystemExit(apply())
