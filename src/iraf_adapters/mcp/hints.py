# -*- coding: utf-8 -*-
"""给调用方看的"该怎么用 / 失败该怎么办"文本（纯逻辑，不依赖 mcp，可单测）。

为什么单独成模块：这些内容要**随工具返回**一起给平台，而平台的问题往往不是调不通，
而是"看到失败就自己去改实现/重写抓取"。把处置规则写进返回值，比写在文档里有效。
2026-10-21/22 实机证据：AgentOS 在失败后读了我们的 mujoco_backend.py 十几个文件、
还改了它自己的 skill，却始终没调 iraf_submit_task —— 缺的就是这段"下一步"。
"""

WORKED_EXAMPLE = {
    "text": "抓取 box_01",
    "purpose": "语义映射示例：说明这类话应交给 iraf_submit_task。**不要求执行**，"
               "更不要在进入模式时拿它当链路自检（会弄脏 MuJoCo 场景）。",
    "resolved_skill": "pick_object",
    "expected": {"status": "SUCCEEDED", "robot_mode": "robot_dev"},
    "verify_with": "iraf_get_task(execution_id)",
    "notes": [
        "这是机器人动作的**唯一受支持入口**：不要用 execute_code/terminal/host_command 自行驱动。",
        "MuJoCo 场景有状态：成功抓取会把目标带走；下一次抓取前先复位场景（重启 Provider），"
        "否则会报 distance≈0.374m —— 那是环境状态，不是实现 bug。",
        "不要为了\"验证工具能用\"先执行一次本示例：进入模式（iraf_robot_mode）本身就是链路可用性证据；"
        "一次成功抓取会搬走目标，本次会话就没有干净场景可给用户演示了。",
    ],
}

FILTERED_EXAMPLE = {
    "text": "今天天气怎么样",
    "expected": {"status": "FAILED", "error_code": "IRAF-NOT-ROBOT-TASK"},
    "then": "由平台自己的能力处理；不要重试本工具，也不要把失败包装成成功。",
}


#: 联合世界（Go2 + Piper + UR5e）9 步交接链路的入口示例。
#: 为什么单独给一个常量：piper/ur5 是"单技能"语义（iraf_submit_task），而 handoff_lab 是
#: **场景级链路**（iraf_run_scenario，9 步、100–170s、必须异步轮询），调用姿势不同，
#: 平台容易把它当成一次抓取来问。把入口、异步语义、判据口径一起写进资源返回。
HANDOFF_LAB_EXAMPLE = {
    "text": "跑 handoff_lab 全量演示",
    "purpose": "联合世界 9 步交接链路（verify → dock → pick → place_in_tray → confirm → "
               "dock_station_b → unload → place_at_b_table）的语义入口：应交给 iraf_run_scenario。",
    "call": {
        "tool": "iraf_run_scenario",
        "arguments": {"scenario": "nominal", "display": "none",
                      "scene_id": "joint-handoff", "async_mode": True},
        "why_async": "整链约 100–170s；同步调用会让客户端超时。异步返回 run_id 后按 "
                     "next_poll_after_s（20s）轮询 iraf_get_run。",
    },
    "expected": {"status": "SUCCEEDED", "passed": True, "steps": 9,
                 "wall_seconds": "约 100–170s（x86 本地仿真；开窗更慢 10–30%）"},
    "verify_with": "iraf_get_run(run_id)；或用 iraf_get_task(execution_id) 读事件库",
    "notes": [
        "判定以**报告**为准（report.passed）；链路进程 rc≠0 只作 warning（收尾段 SEGV 已知）。",
        "display=window 需要 X 可用（本机 :1）；演示用 window，验收/回归用 none 更快。",
        "场景有状态：跑完会搬动物体；再次演示前先复位（重启 Provider 或 ensure 对应场景）。",
        "不要用 execute_code/terminal 自己驱动 MuJoCo；也不要把它退化成一次 pick_object 提交。",
    ],
}


#: 进入机器人开发模式后的**固定动作顺序**（给平台读）。
#: 实机教训（2026-10-22 dashboard /chat）：模型把"进入模式"理解成"先做全链路自检"，
#: 于是自行加了一段**冒烟提交**（用户当场拒绝）。而 MuJoCo 场景有状态，一次成功抓取
#: 就把目标搬走 ⇒ 冒烟吃掉本次唯一干净场景，用户真正的演示必现 distance≈0.374435m。
#: 所以把"进入=只读盘点、不许拿真任务探路"直接写进返回值，而不是写在文档里。
ENTER_NEXT_STEPS = [
    "① 先只读盘点：把 skills / intents / profile / safety_policy / simulation 如实报告给用户"
    "（这是\"现成能力清单\"），不要自编技能名或场景名。",
    "② 等用户给出真实语义需求。**不要为了验证链路而提交任务** —— 进入模式本身已经证明链路可用；"
    "examples 只是语义映射说明，不要求执行。",
    "③ 用户给出动作类需求后，调用 iraf_submit_task(text=<用户原话>)，"
    "由 IRAF Skill Manager 选技能，不要自编参数或自己执行。",
    "④ 提交后用 iraf_get_task(execution_id) 回查权威事实，不要采信模型自述。",
    "⑤ 场景有状态：一次成功抓取会把目标搬走，再次抓取前必须复位（重启 Capability Provider），"
    "否则会报 distance≈0.374435m；链路验收请在装机阶段用探针，不要在对话里拿真任务试。",
]


def enter_next_steps() -> list:
    """进入模式后的固定动作顺序（返回副本，调用方可安全序列化）。"""
    return list(ENTER_NEXT_STEPS)


def next_steps(result: dict) -> list:
    """按结果给出可执行的下一步（不猜、不粉饰）。"""
    status = str((result or {}).get("status") or "")
    code = str((result or {}).get("error_code") or "")
    reason = str((result or {}).get("reason") or "")
    steps = []
    if status == "SUCCEEDED":
        steps.append("核对事实：调用 iraf_get_task(execution_id=%s) 读事件库，"
                     "不要采信任何模型自述。" % (result or {}).get("execution_id"))
        steps.append("MuJoCo 场景有状态：成功抓取会把目标带走，下一次抓取前需要复位场景"
                     "（重新启动 Capability Provider），否则会报 distance≈0.374m。")
        return steps
    if code == "IRAF-NOT-ROBOT-TASK":
        steps.append("这不是机器人任务：请由平台自己的能力处理，不要重试本工具、不要伪造执行结果。")
        return steps
    if "distance=" in reason and ("末端未到达" in reason or "不一致" in reason):
        steps.append("典型原因是**场景状态**（目标已被上一次执行搬走）或定位参数不匹配："
                     "先复位场景（重启 Provider）再重试一次；仍失败再把 reason 原文反馈，"
                     "不要修改源码或自行重写抓取逻辑。")
    if code in {"IRAF-PROVIDER-ERROR", "IRAF-EXECUTION-FAILED"}:
        steps.append("先用 iraf_get_task 回查同一条 execution 的完整证据（qpos/evidence），"
                     "再判断是参数、场景还是 Provider 不可达。")
    if code.startswith("IRAF-POLICY-DENIED") or code.startswith("IRAF-UNAUTHENTICATED"):
        steps.append("这是策略/鉴权拒绝：不要重试或放宽参数，按运维流程处理。")
    if not steps:
        steps.append("用 iraf_get_task(execution_id) 回查权威事实；不要把失败表述为成功。")
    return steps
