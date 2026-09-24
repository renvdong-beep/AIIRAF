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

    def __init__(self, model, data, owner, label=None):
        if model is None or data is None:
            raise PlantError("植物必须显式给出 model 与 data（本模块不构造默认模型）")
        if not hasattr(data, "qpos"):
            raise PlantError("data 看起来不是 MjData（没有 qpos）：%r" % (data,))
        self.model = model
        self.data = data
        self.owner = str(owner) if owner is not None else None
        if self.owner is None:
            raise PlantError("植物必须声明 owner（唯一时间推进者）；不猜")
        self.label = str(label) if label else self.owner
        self._lock = threading.RLock()
        self._step_index = 0

    # ---- 只读视图
    @property
    def step_index(self):
        with self._lock:
            return self._step_index

    @property
    def timestep(self):
        return float(self.model.opt.timestep)

    def lock(self):
        """植物互斥锁（可重入）：Viewer 同步与所有读写在它下面做。"""
        return self._lock

    def is_owner(self, who):
        return str(who) == self.owner

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
            return self._step_index

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
        """等 owner 把 `step_index` 推到 ≥ index；超时显式失败（不静默返回）。"""
        index = int(index)
        timeout = float(timeout)
        poll_seconds = float(poll_seconds)
        if timeout <= 0 or poll_seconds <= 0:
            raise PlantError("等待超时与轮询间隔必须显式给出正数：%r / %r" % (timeout, poll_seconds))
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

    def diagnostics(self):
        return {"label": self.label, "owner": self.owner, "step_index": self.step_index,
                "timestep_s": self.timestep, "nq": int(self.model.nq), "nu": int(self.model.nu)}
