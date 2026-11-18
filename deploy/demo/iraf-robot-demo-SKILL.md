---
name: iraf-robot-demo
description: "IRAF 端侧演示：用语义让 Piper/UR5 抓取、Go2/G1 前后左右移动，并在 8085 演示屏自动播放；支持在可抓取范围内随机取 box 位置。"
version: 0.1.0
author: Intewell Agent
tags: [iraf, robot, demo, plan, piper, ur5, go2, g1, kinematic]
---

# IRAF Robot Demo（端侧运动学演示）

当用户说「演示一下」「看看效果」「随机抓一个」「让狗/人形走一下」时使用本技能。
演示屏：**http://10.203.247.147:8085/**（8084 是调试台，不要用于客户演示）。

## 原则

- 只用 `iraf_edge_plan_task` 发起演示；它计算运动学计划并自动发 job，8085 会自动播放。
- 不声称物理/真机执行；回答里保留 `PLAN` 口径（`planned=true, physics=false`）。
- 目标必须在 `iraf_edge_target_ranges` 返回的范围内；随机目标用 `iraf_edge_random_target` 生成。

## 工具

| 工具 | 用途 |
|---|---|
| `iraf_edge_plan_task(robot, action, x, y, z, direction, distance, random_target)` | 计算计划并发布 job |
| `iraf_edge_random_target(robot)` | 在范围内随机取一个已验证的 box 位置 |
| `iraf_edge_target_ranges(robot)` | 查 x/y/z 范围 |

## 话术 → 调用

| 用户说 | 调用 |
|---|---|
| 「piper 抓取 box」 | `iraf_edge_plan_task(robot="piper", action="pick")` |
| 「ur5 随机位置抓一个」 | `iraf_edge_random_target(robot="ur5")` → `iraf_edge_plan_task(robot="ur5", action="pick", x=..., y=..., z=...)` |
| 「让狗往左走 0.4 米」 | `iraf_edge_plan_task(robot="go2", action="move", direction="left", distance=0.4)` |
| 「让 G1 往前走 0.5 米」 | `iraf_edge_plan_task(robot="g1", action="move", direction="forward", distance=0.5)` |
| 「范围是多少」 | `iraf_edge_target_ranges(robot="piper")` |

方向只支持 `forward / back / left / right`；距离 0–2 米。

## 回答模板

> 已下发演示：Piper 抓取 box @ (0.24, 0.33, 0.17)，计划 143 帧 / 4.6s，IK 误差 2.2e-7 m。
> 看 8085 演示屏（已自动播放），job_id=…。这是运动学计划，不是物理执行。

## 排障

- 8085 没动：确认 `iraf-demo` 服务在跑、`/var/lib/iraf/demo-jobs` 有新 json；刷新页面。
- 工具报 `RANGES-NOT-FOUND`：板卡 `/opt/iraf-demo/app/ranges.json` 缺失。
- 工具报 `PLAN-TASK-FAILED`：看返回 reason（模型/场景路径/权限）。
