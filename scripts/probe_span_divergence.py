#!/usr/bin/env python3
"""逐步比对：分叉到底进在**植物状态**里还是进在**控制取值**里（§11.88 量测⑳）。

背景：s02b 停靠的离散分支已收窄到「s03_pick 之内、第一拍、植物步号相同、取值不同」
（量测⑱）。这一步要做的是**二选一**判死：

  A. 若 s03 的 `state_before`（进入该步时的整机 qpos 指纹）在两支里**相同**，
     而该步**第一拍**的控制取值已不同，且该拍植物步号与 `before` 一致
     ⇒ 控制量不是由"植物状态"算出来的差异 ⇒ 差异在**控制器内部相位**
     （`stand` → `_run_control` 的 `q0 = qpos(调用开始那拍)` 与 `start = data.time`）
     ⇒ 修法：把 hold 调用**对齐到植物步号**，而不是让线程随挂钟起新的 hold 调用。

  B. 若 `state_before` 已不同（或首个不同的 ctrl 拍晚于 `before`）
     ⇒ 狗是**受害方**：状态先被别的东西改掉（另一台臂的控制 / 接触），
     需要转去比对臂侧写入，而不是继续改狗。

只读 build/diagnostics/*.log，不修改任何配置、不写仓库文件。

用法：
  python3 scripts/probe_span_divergence.py build/diagnostics/pair-round*.log
  python3 scripts/probe_span_divergence.py --detail s03_pick <logs...>
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os

NORMAL_YAW = -2.38582468189734
YAW_TOL = 1e-9


def parse(path: str):
    dock = None
    spans = None
    with open(path, errors="replace") as fh:
        for line in fh:
            if line.startswith("PLANT_STEP_SPANS "):
                try:
                    spans = json.loads(line.split(" ", 1)[1])
                except ValueError:
                    continue
            elif line.startswith("DOCK_HALT "):
                try:
                    d = json.loads(line[len("DOCK_HALT "):])
                except ValueError:
                    continue
                if str(d.get("station", "")).endswith("b"):
                    dock = d
    return dock, spans


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--detail", default="s03_pick")
    args = ap.parse_args()

    paths = []
    for pattern in args.logs:
        paths.extend(sorted(globmod.glob(pattern)))
    paths = sorted(set(paths))
    if not paths:
        print("无数据：参数未匹配到任何日志")
        return 2

    rounds = []
    for p in paths:
        dock, spans = parse(p)
        if dock is None or spans is None:
            continue
        yaw = dock.get("final_yaw_deg")
        branch = "常态" if (yaw is not None and abs(yaw - NORMAL_YAW) <= YAW_TOL) else "备选"
        rounds.append({"file": os.path.basename(p), "branch": branch, "yaw": yaw, "spans": spans})

    normal = next((r for r in rounds if r["branch"] == "常态"), None)
    anoms = [r for r in rounds if r["branch"] == "备选"]
    if normal is None:
        print("无数据：没有常态参照轮（无法比对）")
        return 3
    print("共 %d 轮：常态 %d / 备选 %d；参照 = %s" % (
        len(rounds), len(rounds) - len(anoms), len(anoms), normal["file"]))
    if not anoms:
        print("本批无备选轮 ⇒ 不构成结论（需更大批次）")
        return 4

    ref = {str(s.get("id")): s for s in normal["spans"]}
    ids = [str(s.get("id")) for s in normal["spans"]]
    fields = ("state_before", "state_after", "ctrl_digest", "ctrl_n", "dq_before", "dq_after",
              "hid_before", "hid_after")
    print("\n逐步对照（只列与常态不同的项）")
    print("%-4s %-22s %s" % ("#", "step", "与常态不同的字段"))
    for r in anoms[:3]:
        cur = {str(s.get("id")): s for s in r["spans"]}
        print("--- %s（yaw=%r）" % (r["file"], r["yaw"]))
        first_state = None
        first_ctrl = None
        for i, sid in enumerate(ids):
            a, b = ref.get(sid, {}), cur.get(sid, {})
            diffs = [f for f in fields if a.get(f) != b.get(f)]
            if "state_before" in diffs and first_state is None:
                first_state = (i, sid)
            if "ctrl_digest" in diffs and first_ctrl is None:
                first_ctrl = (i, sid)
            if diffs:
                print("  %-2d %-22s %s  before=%s" % (i + 1, sid, ",".join(diffs), b.get("before")))
        print("  ⇒ 第一个 state_before 不同的步骤 = %s" % (str(first_state) if first_state else "无"))
        print("  ⇒ 第一个 ctrl_digest 不同的步骤 = %s" % (str(first_ctrl) if first_ctrl else "无"))
        # 速度级逐元素定位（量测㉓）：位置相同、速度不同时会命中这里 ⇒ 给出**第一个不同的自由度序号**。
        # 序号→关节名的映射离线做（模型在 build/models/ 下），脚本保持零依赖。
        for sid in ids:
            a, b = ref.get(sid, {}), cur.get(sid, {})
            for key in ("dq_before", "dq_after"):
                va, vb = a.get(key), b.get(key)
                if not isinstance(va, list) or not isinstance(vb, list) or va == vb:
                    continue
                nmin = min(len(va), len(vb))
                k = next((j for j in range(nmin) if va[j] != vb[j]), None)
                ndiff = sum(1 for j in range(nmin) if va[j] != vb[j])
                print("  [%s] %s：首个不同的 qvel 元素 = %s（共 %d 个不同；长度 常态 %d / 备选 %d）"
                      % (sid, key, k, ndiff, len(va), len(vb)))
        # 旁路 ctrl 的动力学状态（量测㉔）与**零舍入完整输入集**（量测㉕）：逐子键给出"首次在哪个步骤边界不同"。
        # 这是分层判据：谁先不同 ⇒ 分叉先进入哪一层（约束开关 / 暖启动 / mocap / 外力 / 位置 / 速度 / ctrl）。
        dict_keys = ("x_before", "x_after", "hid_before", "hid_after")
        for key in dict_keys:
            sub_keys = set()
            for s in list(ref.values()) + list(cur.values()):
                got = s.get(key)
                if isinstance(got, dict):
                    sub_keys.update(got.keys())
            hits = []
            for sub in sorted(sub_keys):
                for i, sid in enumerate(ids):
                    va = (ref.get(sid, {}).get(key) or {}).get(sub)
                    vb = (cur.get(sid, {}).get(key) or {}).get(sub)
                    if va is None or vb is None or va == vb:
                        continue
                    hits.append((sub, i + 1, sid, va, vb))
                    break
            if not hits:
                print("  [%s] 共 %d 个子键，在全部步骤边界上**逐位相同**" % (key, len(sub_keys)))
                continue
            for sub, n, sid, va, vb in hits:
                print("  [%s] %s 首个不同 = 第 %d 步（%s）：常态=%s 备选=%s"
                      % (key, sub, n, sid, va, vb))
        # 操作序列摘要（量测㉗）：边界层 + 逐写层各给"首次不同"。
        for key in ("op_before", "op_after"):
            for i, sid in enumerate(ids):
                va = ref.get(sid, {}).get(key)
                vb = cur.get(sid, {}).get(key)
                if va is None or vb is None or list(va) == list(vb):
                    continue
                print("  [%s] 首个不同 = 第 %d 步（%s）：常态=%s 备选=%s" % (key, i + 1, sid, va, vb))
                break
            else:
                print("  [%s] 在全部步骤边界上相同" % key)

        # 控制批次落点（量测㉘）：正常"每拍恰好一批"⇒ 步距直方图只有 {1:…}；出现 0/2 即
        # "某步消费到上一拍的控制（或同一步被写两批）" ⇒ 直接坐实"写入↔推步"交错，并给出首现步号。
        for key in ("batch_before", "batch_after"):
            for label, spans_map in (("常态", ref), ("备选", cur)):
                rows = []
                for sid in ids:
                    v = spans_map.get(sid, {}).get(key)
                    if isinstance(v, dict) and "hist" in v:
                        rows.append((sid, v.get("hist") or {}, v.get("gap0"), v.get("gap2")))
                if not rows:
                    continue
                sid, hist, g0, g2 = rows[-1]
                # 只报"整条 run 的累计"，并把 0/2 两个信号单列（大数值步距=分段空闲，属正常）
                tight = {k: v for k, v in hist.items() if k in ("0", "1", "2", 0, 1, 2)}
                print("  [%s·%s] 步距{0,1,2}计数=%s；首次 gap0=%s；首次 gap2=%s" % (
                    key, label, tight, g0, g2))

        sid = args.detail
        a, b = ref.get(sid, {}), cur.get(sid, {})
        print("  [%s] 常态: before=%s dispatch=%s ctrl_n=%s state_before=%s" % (
            sid, a.get("before"), a.get("dispatch"), a.get("ctrl_n"), a.get("state_before")))
        print("  [%s] 备选: before=%s dispatch=%s ctrl_n=%s state_before=%s" % (
            sid, b.get("before"), b.get("dispatch"), b.get("ctrl_n"), b.get("state_before")))
        pa, pb = a.get("ctrl_pairs") or [], b.get("ctrl_pairs") or []
        if pa and pb:
            n = min(len(pa), len(pb))
            first = next((i for i in range(n) if pa[i] != pb[i]), None)
            print("  [%s] 明细拍数 常态=%d 备选=%d；首次分叉拍 = %s" % (
                sid, len(pa), len(pb), first))
            if first is not None:
                print("      常态该拍 = %s；备选该拍 = %s" % (pa[first], pb[first]))
                same_step = pa[first][0] == pb[first][0]
                boundary = a.get("before")
                print("      拍内植物步号 %s（常态=%s 备选=%s）；该步 before=%s ⇒ 首拍是否即进入该步: %s" % (
                    "相同" if same_step else "不同", pa[first][0], pb[first][0], boundary,
                    "是" if (same_step and pa[first][0] == boundary) else "否"))
                if same_step and pa[first][0] == boundary and a.get("state_before") == b.get("state_before"):
                    print("      ⇒ **判定 A**：同一植物状态、同一步号、首拍取值已不同"
                          " ⇒ 差异在控制器内部相位（stand 调用起点），不在植物状态")
                elif a.get("state_before") != b.get("state_before"):
                    print("      ⇒ **判定 B**：进入该步的植物状态已不同 ⇒ 狗是受害方，"
                          "差异在上游（另一台臂的写入/接触），需转查臂侧")
                else:
                    print("      ⇒ 证据不足以二选一（首拍步号 %s != before %s）" % (
                        pa[first][0], boundary))
                if len(pa[first]) >= 5:
                    # 分量指纹（量测⑳-B，需 `IRAF_DEBUG_CTRL_COMPONENTS=1`）：
                    # 每拍为 (步号, ctrl, desired, q, dq) 五个取值摘要 ⇒ 谁先不同即定性。
                    names = ("步号", "ctrl", "desired", "q", "dq")
                    fd = {nm: next((i for i in range(n) if pa[i][j] != pb[i][j]), None)
                          for j, nm in enumerate(names)}
                    print("      分量首次不同拍: " + "  ".join(
                        "%s=%s" % (nm, fd[nm]) for nm in names))
                    if fd["q"] == first or fd["dq"] == first:
                        print("      ⇒ **判定 B**：首拍 (q, dq) 已不同，而此前狗的取值逐拍相同"
                              " ⇒ 状态是被外部改掉的（另一台本体的写入/接触）⇒ 转查臂侧")
                    elif fd["desired"] == first and (fd["q"] is None or fd["q"] > first):
                        print("      ⇒ **判定 A**：首拍只有 desired 不同、(q, dq) 仍相同"
                              " ⇒ 植物状态未变，差在 hold 调用的目标相位（q0/alpha）")
                    else:
                        print("      ⇒ 分量口径未能二选一（见上面各分量的首次不同拍）")
                if len(pa[first]) >= 9:
                    # 零舍入层（量测㉖）：位序 5..8 = `ctrl/desired/q/dq` 的 float64 原始字节摘要。
                    # 意义：9 位取整会吞掉"亚分辨率"差异；这一层能回答"差异是否真在狗的输入里"。
                    xnames = ("ctrl_x", "desired_x", "q_x", "dq_x")
                    xfd = {nm: next((i for i in range(n) if pa[i][j] != pb[i][j]), None)
                           for j, nm in enumerate(xnames, start=5)}
                    print("      零舍入首次不同拍: " + "  ".join(
                        "%s=%s" % (nm, xfd[nm]) for nm in xnames))
                    if all(xfd[nm] is None for nm in xnames):
                        print("      ⇒ 零舍入下 ctrl/desired/q/dq **完全相同** ⇒ 差异不在狗的输入里"
                              " ⇒ 转向求解器调用次数/顺序这类**非状态输入**（例如额外 `mj_forward` 的时机）")
                if len(pa[first]) >= 10:
                    # 操作序列摘要（量测㉗）：位序 9 = `(操作次数, 操作序列摘要)`。
                    # 这是"非状态输入"的直接观测：若此处首次不同，就说明**操作序列**在那一刻起了分歧。
                    ofd = next((i for i in range(n) if list(pa[i][9]) != list(pb[i][9])), None)
                    print("      操作序列摘要首次不同拍 = %s（常态=%s 备选=%s）" % (
                        ofd, (pa[ofd][9] if ofd is not None else None),
                        (pb[ofd][9] if ofd is not None else None)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
