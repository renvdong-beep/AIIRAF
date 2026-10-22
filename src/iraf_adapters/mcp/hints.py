# -*- coding: utf-8 -*-
"""给调用方看的"该怎么用 / 失败该怎么办"文本（纯逻辑，不依赖 mcp，可单测）。

为什么单独成模块：这些内容要**随工具返回**一起给平台，而平台的问题往往不是调不通，
而是"看到失败就自己去改实现/重写抓取"。把处置规则写进返回值，比写在文档里有效。
2026-10-21/22 实机证据：AgentOS 在失败后读了我们的 mujoco_backend.py 十几个文件、
还改了它自己的 skill，却始终没调 iraf_submit_task —— 缺的就是这段"下一步"。
"""

WORKED_EXAMPLE = {
    "text": "抓取 box_01",
    "resolved_skill": "pick_object",
    "expected": {"status": "SUCCEEDED", "robot_mode": "robot_dev"},
    "verify_with": "iraf_get_task(execution_id)",
    "notes": [
        "这是机器人动作的**唯一受支持入口**：不要用 execute_code/terminal/host_command 自行驱动。",
        "MuJoCo 场景有状态：成功抓取会把目标带走；下一次抓取前先复位场景（重启 Provider），"
        "否则会报 distance≈0.374m —— 那是环境状态，不是实现 bug。",
    ],
}

FILTERED_EXAMPLE = {
    "text": "今天天气怎么样",
    "expected": {"status": "FAILED", "error_code": "IRAF-NOT-ROBOT-TASK"},
    "then": "由平台自己的能力处理；不要重试本工具，也不要把失败包装成成功。",
}


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
    if code.startswith("IRAF-POLICY") or code.startswith("IRAF-UNAUTHENTICATED"):
        steps.append("这是策略/鉴权拒绝：不要重试或放宽参数，按运维流程处理。")
    if not steps:
        steps.append("用 iraf_get_task(execution_id) 回查权威事实；不要把失败表述为成功。")
    return steps
