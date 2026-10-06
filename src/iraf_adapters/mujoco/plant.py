"""一份物理植物（MjModel + MjData）+ **唯一时间推进者**。

为什么需要（2026-09-24 实测，`build/iraf-a6a14/joint-model-two-backends-probe.json`）：
每个 Backend 实例都自持 `MjModel`/`MjData`（`MujocoBackend` 与四足后端都一样）⇒ **每个本体一个
独立世界**：把两个本体的机型声明都指向同一份联合 MJCF，一侧步进 200 步，另一侧的状态**逐位未变**。
联合场景（狗驮托盘 → 臂抓取）要的是"一份植物 + 多个控制器"，因此：

  · 谁推进时间必须**显式声明**（`owner`），非 owner 调用推进 ⇒ 显式失败（禁止把时间推两遍）；
  · guest 需要时间前进时只能 `wait_until(...)`（等价于"仿真推进了 N 步"，但由 owner 执行）；
  · 植物自带一把可重入锁：Viewer 的 `sync()` 复制 mjData 必须与步进互斥
    （否则 MuJoCo 报 "copy mjData while stack is in use"）。

本模块**不含任何数字默认值**：超时一律由调用方给出（缺声明即失败）。
"""

import hashlib
import threading
import time

import mujoco


class PlantError(RuntimeError):
    """植物层的显式失败基类。"""


class PlantOwnershipError(PlantError):
    """非 owner 试图推进时间（或 owner 身份不匹配）。"""


class PlantWaitTimeout(PlantError):
    """guest 等待 owner 推进时间超时。"""


class MujocoPlant:
    """持有 model/data、步数计数与**唯一**时间推进者。"""

    #: 需求闸门下"自由推进线程"允许的**最长停驻**（s）。取值与场景侧"驻留让位"的超时同量级
    #: （`scripts/scenario.py` 的 `_yield_residency_to_step` 用 60 s）——本键只用于把
    #: "停住且无人要时间"这种装配/接线错误**显式暴露**出来，不作为性能参数。
    #: 可用 `IRAF_PLANT_GATE_PARK_TIMEOUT_S` 覆盖（实验用）。
    GATE_PARK_TIMEOUT_S = 60.0

    def __init__(self, model, data, owner, owner_name=None, label=None):
        if model is None or data is None:
            raise PlantError("植物必须显式给出 model 与 data（本模块不构造默认模型）")
        if not hasattr(data, "qpos"):
            raise PlantError("data 看起来不是 MjData（没有 qpos）：%r" % (data,))
        if owner is None:
            raise PlantError("植物必须声明 owner（唯一时间推进者）；不猜")
        self.model = model
        self.data = data
        # ⚠ owner 是**对象身份**（控制器实例），不是名字：判定必须用 `is_owner()`。
        # 实测踩过：一处传对象、一处传声明名 ⇒ `step_once(self)` 报"只有 owner(unitree_go2) 能推进"，
        # 于是狗自己都推不动时间。名字只用于诊断与"装配期按声明名核对"。
        self._owner = owner
        self.owner_name = str(owner_name if owner_name is not None
                              else (owner if isinstance(owner, str) else type(owner).__name__))
        self.label = str(label) if label else self.owner_name
        self._lock = threading.RLock()
        self._step_index = 0
        # ---- 操作序列滚动摘要（2026-10-06 §11.88 量测㉗）----
        # 为什么需要：零舍入下"14750 的完整输入逐位相同、却仍分叉" ⇒ 完备性已证输入清单无遗漏
        # ⇒ 唯一剩下的类别是**操作序列**（谁在什么顺序上做了什么、各做了多少次）。
        # 本摘要把每次改变仿真的操作滚进 blake2b ⇒ "操作序列"也成为可逐拍比对的可观测量。
        # 线程安全：`_op_lock` 是**独立小锁**；调用方通常已持植物锁 ⇒ 锁序为"植物锁 → op 锁"单向，
        # 绝不反向取锁，不引入新的死锁环。
        self._op_lock = threading.Lock()
        self._op_hash = hashlib.blake2b(digest_size=8)
        self._op_seq = 0
        # ---- 「控制批次落点」直方图（2026-10-06 §11.88 量测㉘，只读探测）----
        # 目的：臂侧一次控制更新是**一批**通道写入（同一锁内原子），而它落在**哪一拍**由锁序决定。
        # 正常应"每拍恰好一批"⇒ 相邻批次的步距恒为 1；若出现 0 或 2，即"某步消费到上一拍的控制"
        # （或同一步被写两批）⇒ 直接坐实"写入↔推步"交错，且给出首次发生的步号。
        # 纯计数（不改变任何行为），直方图键数有限 ⇒ 不随步数增长。
        self._last_ctrl_batch_step = {}
        self.ctrl_batch_gap_hist = {}
        self.ctrl_batch_first_bad = None
        # 更锐利的两个信号（量测㉘ 修正）：
        #   gap=0 ⇒ 同一步内写了**两批**（双批）；
        #   gap=2 ⇒ 有一步**没拿到新批次**（消费到上一拍控制 = 竞态的直接签名）。
        # 各记"首次发生"的 `(发起方, 步号)`，供逐支比对。分段之间的**空闲**（臂不动）会给出很大步距，
        # 那属正常 ⇒ 判据只看 0 与 2。
        self.ctrl_batch_first_gap0 = None
        self.ctrl_batch_first_gap2 = None
        # ---- 「晚批」探测（量测㉙，只读）----
        # 动机：arm7 实测——异常轮**没有** gap=2（每步都拿到新批次），但同一植物步上的**操作次数差 10
        # = 臂一整批（8 通道 + begin + end）；且臂的写入步号逐位相同 ⇒ 差异在**同一步区间内的先后**：
        # 狗的前馈 `mj_forward`（**消费 ctrl**）与臂的控制批次谁先谁后。
        # 若臂的批次落在"本步已经发生过一次求解（`mj_forward` 或 `mj_step`）之后"，则这次求解用的是
        # **臂的旧控制** ⇒ 求解器暖启动路径不同 ⇒ 后续位级分叉。本计数器只在"某步已有求解后又收到批次"时 +1。
        self._solved_at_step = None
        self._ff_solved_at_step = None
        self._ff_solved_by_tag = None
        self.ctrl_late_batch_count = 0
        self.ctrl_late_batch_first = None
        # ---- 需求闸门（2026-10-05 §11.87）----
        # 为什么需要：owner 的**自由推进线程**（植物驻留）在 guest 等待期间并发 `mj_step` ⇒
        # guest 请求 `count` 步、实际推进 `count + 挂钟决定的超出量`（§11.56 实测 17×）；
        # 更糟的是 guest **两拍之间** owner 也在推进 ⇒ 臂的控制 dt 逐轮变化 ⇒ 各步骤开始时的
        # 世界状态逐轮不同 ⇒ 步骤结果**不可复现**（实测 A 站逐位可复现、B 站在 1.8785~5.2865° 之间跳）。
        # 闸门语义（**只约束"自由推进线程"**，不碰 owner 自己执行技能时的步进）：
        #   · 有 guest 在等（`_targets` 非空）⇒ 只推进到 `min(_targets)` 就停；
        #   · 没有 guest 在等 ⇒ 一步也不推（仿真冻结，而不是按挂钟空转）。
        # ⇒ 植物推进步数 = Σ(guest 请求) + owner 自己技能的步数，**与挂钟无关**。
        # ⚠ 闸门**只能在 guest 步执行期间开**（由场景在派发步骤时开/关）：owner 自己执行技能时若也开，
        # 驻留线程的 hold 周期会因"无人需求"停在 `await_step_quota` 里 ⇒ **不让位自锁**
        # （实测：`植物驻留未在 60 s 内让位（owner 上一轮 hold 未结束）`）。
        # `_gate_opt_in`（缺省关，实验开关 `IRAF_PLANT_DEMAND_GATE=1`）决定"允许被开"；
        # `_gate_enabled` 是**当前是否生效**，初始为 False ⇒ 未被场景打开时逐位不变。
        # 由**装配层**按场景声明施加（`scenario.py::_apply_plant_demand_gate`），默认关 ⇒ 逐位不变。
        # 为什么不在这里读环境变量：可复现性是验收口径，必须来自**声明**（`scene.plant_demand_gate`），
        # 不能依赖隐式环境变量（AGENTS.md 5.3）。实验覆盖只在装配层做，并把生效值打进日志/报告。
        self._gate_opt_in = False
        self._gate_enabled = False
        self._free_run_thread = None
        self._targets = []
        self._gate_cond = threading.Condition()

    # ---- 只读视图
    @property
    def step_index(self):
        with self._lock:
            return self._step_index

    @property
    def owner(self):
        """owner 的**标识字符串**（诊断/报告用）；判定请用 `is_owner()`。"""
        return self.owner_name

    @property
    def timestep(self):
        return float(self.model.opt.timestep)

    def lock(self):
        """植物互斥锁（可重入）：Viewer 同步与所有读写在它下面做。"""
        return self._lock

    def is_owner(self, who):
        """`who` 是否为本植物唯一的时间推进者。

        接受两种情况：**对象本身**（运行期各后端传 `self`），或**声明名**（装配期核对
        "注入的植物是不是我这台机型的"，此时还没有对象可比）。
        """
        if who is self._owner:
            return True
        return isinstance(who, str) and who == self.owner_name

    # ---- 需求闸门（2026-10-05 §11.87；是否装上由场景声明 `scene.plant_demand_gate` 决定）
    @property
    def gate_opt_in(self):
        """本植株是否**允许**被装上需求闸门（装配期由场景声明决定，运行期不变）。"""
        return bool(self._gate_opt_in)

    def set_gate_opt_in(self, enabled):
        """装配期施加场景声明的推进口径（§11.87）。缺省 False ⇒ 逐位不变。"""
        self._gate_opt_in = bool(enabled)
        return self._gate_opt_in

    @property
    def demand_gate(self):
        """闸门**当前是否生效**（由场景在派发 guest 步时开/关）。"""
        return bool(self._gate_enabled)

    def set_demand_gate(self, enabled):
        """开/关需求闸门（**只在装配/实验期由场景按步调用**，不改公开契约）。

        未 opt-in ⇒ 拒绝开启（返回 False，保持逐位不变）。
        """
        with self._gate_cond:
            self._gate_enabled = bool(enabled) and self._gate_opt_in
            if not self._gate_enabled:
                self._targets.clear()
            self._gate_cond.notify_all()
            return self._gate_enabled

    def set_free_run_thread(self, thread=None):
        """登记"自由推进线程"（植物驻留线程）：**只有它**受需求闸门约束。

        为什么不按"是不是 owner"判：owner（动物）**自己执行技能**时也在推进，那时没有 guest 在等，
        若也受闸门约束就会一步都推不动（自锁）。而**自由推进线程**才是"没人需要时间也在按挂钟空转"
        的那一个 ⇒ 闸门只约束它。缺省取当前线程。
        """
        self._free_run_thread = thread if thread is not None else threading.current_thread()
        return self._free_run_thread

    def await_step_quota(self, caller=None):
        """自由推进线程在**每一步之前**调用：只有 guest 有需求时才允许推进。

        闸门关闭 / 调用者不是自由推进线程 ⇒ **立即返回**（逐位不变）。
        返回当前步索引（便于调用方记录"是否真的推了"）。
        """
        if caller is not None and not self.is_owner(caller):
            raise PlantOwnershipError(
                "只有 owner(%s) 能推进时间，调用者=%s" % (self.owner, caller))
        if not self._gate_enabled or threading.current_thread() is not self._free_run_thread:
            return self.step_index
        # ⚠ 锁序纪律（2026-10-05 §11.87 实测死锁）：**持 `_gate_cond` 时不得再取 `_lock`**。
        # 实测 ABBA：驻留线程在 `step_once` 里（已释放 `_lock`）要 `_gate_cond`，
        # 而 guest 在 `_wait_until_gated` 里持 `_gate_cond` 后要 `_lock`（且动物后端的 `_lock`
        # 与植物 `_lock` 是**同一把**（`self.plant.lock()`）⇒ 同一线程可重入持有）⇒ 双方互等，
        # 两个线程全部睡死、CPU 3%、日志不再增长。故闸门内部只读裸 `_step_index`（int 读原子，
        # 顺序由条件变量保证），一律不碰 `_lock`。
        with self._gate_cond:
            deadline = time.monotonic() + self.GATE_PARK_TIMEOUT_S
            while True:
                if not self._gate_enabled:
                    # 场景关闸门（guest 步结束）后必须**立刻返回**，否则驻留线程会停在这里不让位
                    return self._step_index
                if not self._targets:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        # 有界等待（铁律 2.3：禁止无超时等待）：没人来要时间却停了这么久 = 装配/接线错了，
                        # 必须**显式失败**并说清现场，而不是永远静默停住（实测踩过：整轮只有 3% CPU、
                        # 日志不再增长、且因为 guest 侧超时太长而看不到任何报错）。
                        raise PlantError(
                            "需求闸门：自由推进线程已停驻 %.1f s 仍无任何 guest 需求（step_index=%d，"
                            "闸门开启中）。可能原因：① owner 自己的步骤期间被误开闸门；"
                            "② guest 已结束但闸门未关；③ 驻留线程已死。"
                            % (self.GATE_PARK_TIMEOUT_S, self._step_index))
                    self._gate_cond.wait(remaining)
                    continue
                target = min(self._targets)
                if self._step_index < target:
                    return self._step_index
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise PlantError(
                        "需求闸门：推进到 guest 目标后仍停驻 %.1f s 未被注销（step_index=%d，目标=%d）"
                        % (self.GATE_PARK_TIMEOUT_S, self._step_index, target))
                self._gate_cond.wait(remaining)

    # ---- 时间推进
    def step_once(self, caller):
        """推进一个物理步；**仅 owner** 可调。"""
        if not self.is_owner(caller):
            raise PlantOwnershipError(
                "只有 owner(%s) 能推进时间，调用者=%s：两个步进者会把仿真推两遍"
                % (self.owner, caller))
        with self._lock:
            mujoco.mj_step(self.model, self.data)
            self._step_index += 1
            index = self._step_index
            # 这次求解发生在**步号 index-1** 的区间里（量测㉙）：此后同一步再有控制写入即为"晚批"。
            self._solved_at_step = index - 1
            self.op_event("step")
        if self._gate_enabled:
            # ⚠ 只在**到达某个 guest 的目标**时唤醒：每步都 notify_all 会让每步都发生一次
            # 线程切换（实测整轮慢到跑不完）⇒ 这里按目标判断，等待方绝大多数步都不被唤醒。
            with self._gate_cond:
                if self._targets and index >= min(self._targets):
                    self._gate_cond.notify_all()
        return index

    def op_event(self, kind, tag=0):
        """把一次**改变仿真的操作**滚进操作序列摘要（量测㉗）。

        调用点（改变仿真的操作）：`step_once`（"step"）、狗的 PD 重力前馈（"ff"）、两侧的 ctrl 写入（"ctrl"）。
        参数 `tag` 用于区分同一类操作的发起方（例如狗=1 / 臂=2）。
        额外：`kind == "batch_end"` 时记一次"控制批次落点"（量测㉘）。
        """
        with self._op_lock:
            self._op_seq += 1
            self._op_hash.update(b"%d|%s|%d;" % (self._op_seq, str(kind).encode("ascii"), int(tag)))
            if kind == "ff":
                # 前馈内部会 `mj_forward`（一次求解，**消费 ctrl**）⇒ 记"本步已求解"（量测㉙）。
                self._ff_solved_at_step = int(self._step_index)
                self._ff_solved_by_tag = int(tag)
                self._solved_at_step = int(self._step_index)
            if kind == "batch_end":
                _step = int(self._step_index)
                # 「晚批」判定（量测㉙，修正版）：只看**前馈**情形 —— 本步 owner 已算过前馈（`mj_forward`，
                # **消费 ctrl**），而此刻写入者**不是**做那次前馈的人 ⇒ 那次求解消费的是写入者的**旧控制**
                # ⇒ 求解器暖启动路径不同。
                # 为什么去掉 `mj_step` 情形：`mj_step` 在自身内部把步号 +1 ⇒ 其后的写入必然落在**下一步号**，
                # 「同一步号内 `mj_step` 之后写入」这种事件在逻辑上不存在（空判据）。
                # 为什么不误报 owner：owner"先算前馈再写自家 ctrl"是设计内顺序，tag 相同 ⇒ 不计。
                _late = (self._ff_solved_at_step == _step
                         and self._ff_solved_by_tag is not None
                         and self._ff_solved_by_tag != int(tag))
                if _late:
                    self.ctrl_late_batch_count += 1
                    if self.ctrl_late_batch_first is None:
                        self.ctrl_late_batch_first = (int(tag), _step)
                _prev = self._last_ctrl_batch_step.get(int(tag))
                if _prev is not None:
                    _gap = _step - _prev
                    _hist = self.ctrl_batch_gap_hist.setdefault(int(tag), {})
                    _hist[_gap] = _hist.get(_gap, 0) + 1
                    if _gap != 1 and self.ctrl_batch_first_bad is None:
                        self.ctrl_batch_first_bad = (int(tag), _step, int(_gap))
                    if _gap == 0 and self.ctrl_batch_first_gap0 is None:
                        self.ctrl_batch_first_gap0 = (int(tag), _step)
                    if _gap == 2 and self.ctrl_batch_first_gap2 is None:
                        self.ctrl_batch_first_gap2 = (int(tag), _step)
                self._last_ctrl_batch_step[int(tag)] = _step

    def op_digest(self):
        """当前操作序列的 `(次数, 摘要)`：可逐拍采样、跨进程可比（blake2b，不受 PYTHONHASHSEED 影响）。"""
        with self._op_lock:
            return (self._op_seq, self._op_hash.copy().hexdigest())

    def advance(self, count, caller):
        count = int(count)
        if count < 1:
            raise PlantError("推进步数必须为正数：%r" % (count,))
        with self._lock:
            for _ in range(count):
                self.step_once(caller)
            return self._step_index

    # ---- guest 侧等待
    def wait_until(self, index, timeout, poll_seconds):
        """等 owner 把 `step_index` 推到 ≥ index；超时显式失败（不静默返回）。

        闸门开启时走**条件变量**路径：先把目标步索引登记成"需求"（自由推进线程据此才能推进），
        等到达后注销 ⇒ owner 的推进量**恰好等于 guest 的请求量**（无挂钟相关的超出量）。
        """
        index = int(index)
        timeout = float(timeout)
        poll_seconds = float(poll_seconds)
        if timeout <= 0 or poll_seconds <= 0:
            raise PlantError("等待超时与轮询间隔必须显式给出正数：%r / %r" % (timeout, poll_seconds))
        if self._gate_enabled:
            return self._wait_until_gated(index, timeout, poll_seconds)
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._step_index >= index:
                    return self._step_index
            if time.monotonic() >= deadline:
                raise PlantWaitTimeout(
                    "等待 owner(%s) 推进到第 %d 步超时（%.3f s，当前 %d 步）"
                    % (self.owner, index, timeout, self.step_index))
            time.sleep(poll_seconds)

    def _wait_until_gated(self, index, timeout, poll_seconds):
        """闸门路径：登记需求 → 等条件变量 → 注销（`try/finally` 保证异常也注销，避免死锁）。"""
        with self._gate_cond:
            self._targets.append(index)
            self._gate_cond.notify_all()
            try:
                deadline = time.monotonic() + timeout
                while True:
                    # ⚠ 锁序纪律（§11.87）：**持 `_gate_cond` 时不得取 `_lock`**（否则与驻留线程
                    # 在 `step_once` 里"先 `_lock` 后 `_gate_cond`"构成 ABBA 死锁）。裸读 int 即可。
                    if self._step_index >= index:
                        return self._step_index
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise PlantWaitTimeout(
                            "等待 owner(%s) 推进到第 %d 步超时（%.3f s，当前 %d 步；需求闸门开启）"
                            % (self.owner, index, timeout, self._step_index))
                    # 等待粒度上限 50 ms：到达目标时有 notify 立刻唤醒，这里只是超时检查的兜底
                    # （按调用方给的 poll_seconds=步长 会变成每 2 ms 醒一次、白烧 CPU）
                    self._gate_cond.wait(min(remaining, max(poll_seconds, 0.001), 0.05))
            finally:
                if index in self._targets:
                    self._targets.remove(index)
                self._gate_cond.notify_all()

    def diagnostics(self):
        return {"label": self.label, "owner": self.owner, "step_index": self.step_index,
                "timestep_s": self.timestep, "nq": int(self.model.nq), "nu": int(self.model.nu)}
