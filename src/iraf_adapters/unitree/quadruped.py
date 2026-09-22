"""四足通用契约（步骤 16）：能力面与标准反馈平台无关，厂家细节不外泄。

本模块是**通用契约层**，只做四件事：

1. 定义四足能力的规范词表（`stand` / `stop` / `locomote` / `read_state` /
   `emergency_stop`）与标准反馈键名（`STATE_KEYS`）——厂家关节名、DDS domain、
   CRC 细节一律不得出现在这一层（AGENTS.md 铁律 2.5 / 1.13）。
   `assert_portable_surface()` 把这些词变成可执行门禁，而不是注释里的口号。
2. 校验 `declared ⊆ implemented` 的能力契约，并把「方法存在 ≠ 能力已实现」讲清楚：
   基类为每个能力都定义了方法（否则装配期发现不了悬空能力），但**未实现的能力必须
   显式拒绝**（`UnsupportedCapabilityError`），不得返回伪造成功（铁律 5）。
3. 提供执行台账（单调序号 + 终态不可回写）与安全闭锁（急停后必须授权复位），
   把 `iraf_core.authority.ControlAuthorityManager` 的租约与 fencing token 变成每一次
   运动调用的前置条件：无租约、旧 token、终态执行的控制请求一律拒绝（铁律 1.12 / 1.13）。
4. 提供与验收路径同语义的纯函数（`pd_torque` / `gravity_bias_torque`），并在单测里与
   `loopback.compute_ctrl` / `loopback.bias_torque` 逐位交叉比对，避免两份事实漂移。

分层与边界
----------
- 机型实现放 `unitree_go2.py`：关节↔执行器映射只在那里解析一处。
- 数字只来自声明（`profiles/<robot>_mujoco.yaml` + `config/<robot>_loopback.yaml`），
  本模块不含任何机型数字默认值；缺失即显式失败。
- 本层只交付**仿真**后端（`simulation: true`）；真机接入须另立 ADR 与 HIL 证据，
  目标端/真机验收在本战役中一律 DEFERRED（板卡不在场）。
"""

import math
import re

import numpy as np

from iraf_core.authority import LeaseConflict

#: 能力规范词表（平台无关；不得出现机型/厂家语义）。
CAPABILITIES = ("emergency_stop", "locomote", "read_state", "stand", "stop")

#: 会驱动执行器（产生物理动作）的能力子集。`read_state` 不在此列：它只读不改。
MOTION_CAPABILITIES = ("emergency_stop", "locomote", "stand", "stop")

ACTIVE_STATE = "ACTIVE"
#: 终态：一旦写入不得被回写（铁律 1.6 / 2.4）。
TERMINAL_STATES = ("SUCCEEDED", "FAILED", "CANCELLED", "SAFETY_STOP", "STOPPED")

#: 标准反馈键（通用契约面）。机型实现必须给出**完整且恰好**这些顶层键。
STATE_KEYS = (
    "simulation",
    "time_s",
    "base_position_m",
    "base_quaternion_wxyz",
    "base_linear_velocity_mps",
    "base_angular_velocity_rad_s",
    "joint_positions_rad",
    "joint_velocities_rad_s",
    "joint_torque_nm",
    "imu",
    "emergency_stop",
    "control_source",
)
IMU_KEYS = ("quaternion_wxyz", "angular_velocity_rad_s", "linear_acceleration_mps2")

#: 速度指令字段（规范名，平台无关）。未知字段必须拒绝，防止厂家语义渗入通用层。
VELOCITY_FIELDS = ("vx_mps", "vy_mps", "wz_rad_s")

#: 厂家内部标识的识别式：出现在**通用契约面**（能力名/公开方法名/参数名/反馈顶层键）即失败。
VENDOR_TOKEN_PATTERNS = (
    re.compile(r"(?i)\b(?:fl|fr|rl|rr)_[a-z0-9_]+"),
    re.compile(r"(?i)dds"),
    re.compile(r"(?i)crc"),
    re.compile(r"(?i)domain_id"),
    re.compile(r"(?i)can_id"),
    re.compile(r"(?i)serial_number"),
)


class QuadrupedError(ValueError):
    """四足适配层错误基类（带稳定的错误码，便于上层映射与中文诊断）。"""

    code = "IRAF-QUADRUPED-ERROR"

    def __init__(self, message):
        super().__init__(message)
        self.code = type(self).code


class DeclarationError(QuadrupedError):
    """声明缺失/自相矛盾（fail-closed，禁止默认值兜底）。"""

    code = "IRAF-QUADRUPED-DECLARATION-INVALID"


class CapabilityContractError(QuadrupedError):
    """能力契约破裂：声明了不存在或未实现的能力。"""

    code = "IRAF-QUADRUPED-CAPABILITY-UNAVAILABLE"


class UnsupportedCapabilityError(QuadrupedError):
    """能力未在本后端实现：显式拒绝，不返回伪造成功。"""

    code = "IRAF-QUADRUPED-CAPABILITY-NOT-IMPLEMENTED"


class ModelUnavailableError(QuadrupedError):
    """模型/关键帧/执行器引用不可用。"""

    code = "IRAF-QUADRUPED-MODEL-UNAVAILABLE"


class CommandRejectedError(QuadrupedError):
    """指令被拒绝：未知关节名、越界、非法数值、非法时长。"""

    code = "IRAF-QUADRUPED-COMMAND-REJECTED"


class ControlAuthorityError(QuadrupedError):
    """控制权不成立：无租约、旧 fencing token、租约过期或被他人接管。"""

    code = "IRAF-QUADRUPED-NO-CONTROL-AUTHORITY"


class TerminalExecutionError(QuadrupedError):
    """终态执行的控制请求：必须拒绝，终态不得回写为成功。"""

    code = "IRAF-QUADRUPED-TERMINAL-EXECUTION"


class EmergencyStopLatchedError(QuadrupedError):
    """仍处于急停闭锁：授权复位前不得重新调度运动。"""

    code = "IRAF-QUADRUPED-SAFETY-LATCHED"


def vendor_tokens(text):
    """返回文本里命中的厂家内部标识（用于通用契约面的可移植性门禁）。"""
    hits = []
    for pattern in VENDOR_TOKEN_PATTERNS:
        hit = pattern.search(str(text))
        if hit:
            hits.append(hit.group(0))
    return hits


def assert_portable_surface(kind, names):
    """通用契约面不得出现厂家内部标识；命中即显式失败（不是打印警告）。"""
    offenders = {}
    for name in names:
        hits = vendor_tokens(name)
        if hits:
            offenders[str(name)] = hits
    if offenders:
        raise DeclarationError(
            "%s 含厂家内部标识（通用契约必须平台无关）: %s" % (kind, offenders)
        )
    return True


def validate_state(state):
    """标准反馈的键集合必须与规范完全一致：多键/少键都是契约破裂。"""
    missing = [key for key in STATE_KEYS if key not in state]
    extra = sorted(set(state) - set(STATE_KEYS))
    if missing or extra:
        raise DeclarationError("反馈键集合不符合通用契约：缺少 %s，多出 %s" % (missing, extra))
    imu = state["imu"] or {}
    imu_missing = [key for key in IMU_KEYS if key not in imu]
    if imu_missing:
        raise DeclarationError("反馈 imu 段缺少键: %s" % imu_missing)
    assert_portable_surface("反馈顶层键", list(state))
    return True


def pd_torque_raw(q, dq, q_des, kp, kd, tau_ff):
    """力矩型 PD + 前馈的**未截断**命令（纯函数）。

    抽出来是为了让「命令力矩 vs 模型 ctrlrange 的缺口」可被测到：`pd_torque` 返回的是**已截断**
    的值，采样拿到它就只能知道"饱和了"，不知道"缺多少"。两处必须用同一段算术 ——
    `pd_torque` 调用本函数后再截断，任何一处改动都会同时生效（不存在第二份公式）。
    """
    return kp * (q_des - q) - kd * dq + tau_ff


def pd_torque(q, dq, q_des, kp, kd, tau_ff, lower, upper):
    """力矩型 PD + 前馈，并按模型 ctrlrange 截断。纯函数，便于逐项断言。

    语义与验收路径 `loopback.compute_ctrl` 必须逐位一致（单测交叉比对）；重复出现
    在这里是因为验收 harness 与运行期后端是两条调用链，靠测试钉住一致性而不是复制注释。
    """
    ctrl = pd_torque_raw(q, dq, q_des, kp, kd, tau_ff)
    saturated = np.logical_or(ctrl < lower, ctrl > upper)
    return np.clip(ctrl, lower, upper), saturated


def gravity_bias_torque(model, data, mujoco, dofs):
    """重力前馈：qvel 置零后取 `qfrc_bias`（与臂侧后端、验收路径同一做法）。

    注意这是**自由浮动基**下的偏置，不含地面约束反力，因此它只消除稳态下垂，
    站立载荷由 PD 承担；不得把它表述为支撑力矩。
    """
    qvel = data.qvel.copy()
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    tau = np.array([data.qfrc_bias[int(dof)] for dof in dofs], dtype=float)
    data.qvel[:] = qvel
    mujoco.mj_forward(model, data)
    return tau


def substeps_per_control(timestep_s, control_hz):
    """控制周期与物理步长的整除校验：不整除即声明非法，不四舍五入。"""
    if control_hz <= 0:
        raise DeclarationError("控制频率必须为正数，实际: %s" % control_hz)
    if timestep_s <= 0:
        raise DeclarationError("模型 timestep 必须为正数，实际: %s" % timestep_s)
    exact = 1.0 / (float(control_hz) * float(timestep_s))
    rounded = int(round(exact))
    if rounded < 1 or abs(exact - rounded) > 1e-9:
        raise DeclarationError(
            "控制频率 %g Hz 与模型 timestep %g s 不整除：一个控制周期 %g 个物理步"
            % (control_hz, timestep_s, exact)
        )
    return rounded


class ExecutionLedger:
    """执行台账：单调序号 + 状态迁移 + 终态不可回写（铁律 2.4 / 1.6）。

    序号单调递增且不重用；终态写入后再写任何状态都显式失败，避免「终态被执行回写成成功」。
    """

    def __init__(self):
        self._sequence = 0
        self._executions = {}

    def begin(self, capability, fencing_token):
        if capability not in CAPABILITIES:
            raise DeclarationError("未知能力，不在规范词表内: %s" % capability)
        self._sequence += 1
        execution_id = "exec-%04d" % self._sequence
        self._executions[execution_id] = {
            "execution_id": execution_id,
            "capability": str(capability),
            "state": ACTIVE_STATE,
            "sequence": self._sequence,
            "fencing_token": int(fencing_token),
            "terminal_reason": None,
        }
        return execution_id

    def entry(self, execution_id):
        entry = self._executions.get(str(execution_id))
        if entry is None:
            raise CommandRejectedError("未登记的执行 id（不得对未知执行下达控制指令）: %s" % execution_id)
        return entry

    def assert_active(self, execution_id):
        entry = self.entry(execution_id)
        if entry["state"] in TERMINAL_STATES:
            raise TerminalExecutionError(
                "执行 %s 已处于终态 %s：终态不得被回写，控制请求必须拒绝"
                % (entry["execution_id"], entry["state"])
            )
        return entry

    def finish(self, execution_id, state, reason=None):
        if state not in TERMINAL_STATES:
            raise DeclarationError("finish 只接受终态，实际: %s" % state)
        entry = self.entry(execution_id)
        if entry["state"] in TERMINAL_STATES:
            raise TerminalExecutionError(
                "执行 %s 已是终态 %s：不得再次写入 %s（终态不可回写）"
                % (entry["execution_id"], entry["state"], state)
            )
        entry["state"] = str(state)
        entry["terminal_reason"] = None if reason is None else str(reason)
        return entry

    def snapshot(self):
        return [dict(entry) for entry in self._executions.values()]


class EmergencyStopLatch:
    """安全闭锁：急停后必须先由控制器确认 safe state 并授权复位（铁律 1.9 / 1.6）。"""

    def __init__(self):
        self._engaged = False
        self._reason = None
        self._sequence = 0

    @property
    def engaged(self):
        return self._engaged

    def engage(self, reason):
        text = str(reason or "").strip()
        if not text:
            raise DeclarationError("急停必须带非空原因（安全事件要可追溯）")
        self._engaged = True
        self._reason = text
        self._sequence += 1
        return self.snapshot()

    def assert_clear(self):
        if self._engaged:
            raise EmergencyStopLatchedError(
                "仍处于急停闭锁（原因：%s）：授权复位前不得重新调度运动" % self._reason
            )
        return True

    def clear(self, authorization):
        if not self._engaged:
            raise CommandRejectedError("当前未处于急停闭锁，无需复位")
        text = str(authorization or "").strip()
        if not text:
            raise ControlAuthorityError("复位必须带授权依据（控制器确认 safe state 后的授权引用）")
        self._engaged = False
        self._reason = None
        self._authorization = text
        return self.snapshot()

    def snapshot(self):
        return {
            "engaged": bool(self._engaged),
            "reason": self._reason,
            "sequence": int(self._sequence),
        }


def verify_capabilities(profile_capabilities, adapter_class):
    """`declared ⊆ implemented` 能力契约（装配期可复用的纯函数）。

    与 `iraf_adapters.factory.verify_backend_contract` 的分工：
    factory 校验「声明的能力在类上有对应方法」（防止悬空能力）；
    本函数校验「声明的能力在**该后端**上真的已实现」（防止方法存在但只会抛
    `UnsupportedCapabilityError` 的能力被声明 —— 例如首期没有步态控制器的 Go2 不得声明 `locomote`）。
    """
    declared = frozenset(str(item) for item in (profile_capabilities or ()))
    implemented = frozenset(str(item) for item in getattr(adapter_class, "IMPLEMENTED_CAPABILITIES", ()))
    unknown = sorted(declared - set(CAPABILITIES))
    missing = sorted(declared - implemented)
    if unknown:
        raise CapabilityContractError("Profile 声明了规范词表外的能力（悬空能力）: %s" % unknown)
    if missing:
        raise CapabilityContractError(
            "Profile 声明了 %s 未实现的能力: %s（已实现: %s）"
            % (getattr(adapter_class, "__name__", adapter_class), missing, sorted(implemented))
        )
    return {
        "schema_version": "iraf.quadruped-capability-contract/v1",
        "adapter": "%s:%s" % (adapter_class.__module__, adapter_class.__name__),
        "vocabulary": list(CAPABILITIES),
        "declared_capabilities": sorted(declared),
        "implemented_capabilities": sorted(implemented),
        # 实现了但未声明：只记录（与 factory 的 one-way 语义一致），不拦装配。
        "undeclared_implemented_capabilities": sorted(implemented - declared),
        "passed": True,
    }


class QuadrupedAdapter:
    """四足后端的通用契约面；机型实现继承它并声明 `IMPLEMENTED_CAPABILITIES`。"""

    CAPABILITIES = CAPABILITIES
    MOTION_CAPABILITIES = MOTION_CAPABILITIES
    #: 该后端**真正实现**的能力；空集合表示「只有契约面，没有实现」。
    IMPLEMENTED_CAPABILITIES = frozenset()
    #: 力矩上限的唯一来源（本战役只允许 model，禁止在配置里另写一套安全边界）。
    TORQUE_LIMIT_SOURCES = ("model",)

    def __init__(self, declaration, profile, authority):
        if authority is None:
            raise ControlAuthorityError("必须提供 ControlAuthorityManager：控制权只能来自受信租约")
        if profile is None:
            raise DeclarationError("必须提供 RobotProfile（身份只能来自声明）")
        self.declaration = dict(declaration or {})
        self.profile = profile
        self.authority = authority
        self.ledger = ExecutionLedger()
        self.estop = EmergencyStopLatch()
        self._control_source = {"owner": None, "fencing_token": None, "execution_id": None}
        if not bool(self.declaration.get("simulation", False)):
            raise DeclarationError(
                "四足后端本轮只交付仿真形态：声明必须 simulation=true；"
                "真机接入须另立 ADR 与 HIL 证据（不得把仿真结论冒充真机能力）"
            )
        source = str(((self.declaration.get("control") or {}).get("torque_limit_source")) or "")
        if source not in self.TORQUE_LIMIT_SOURCES:
            raise DeclarationError(
                "control.torque_limit_source 必须显式声明为 %s，实际: %r"
                % (list(self.TORQUE_LIMIT_SOURCES), source)
            )

    # ---- 能力面：未实现的能力必须显式拒绝，不做任何伪造成功 ----
    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        raise UnsupportedCapabilityError(self._unsupported("stand"))

    def stop(self, lease, execution_id=None):
        raise UnsupportedCapabilityError(self._unsupported("stop"))

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        raise UnsupportedCapabilityError(self._unsupported("locomote"))

    def read_state(self):
        raise UnsupportedCapabilityError(self._unsupported("read_state"))

    def emergency_stop(self, reason):
        """急停：**不要求租约**（安全优先，铁律 1.6），并进入授权复位闭锁。

        只做「零力矩释放 + 记录」，不推进物理：报告里显式写明 `stepped: false`，
        避免把「已下发零力矩」读成「已完成安全停机」。
        """
        snapshot = self.estop.engage(reason)
        released = self._apply_zero_torque()
        return {
            "simulation": bool(self.declaration.get("simulation", False)),
            "capability": "emergency_stop",
            "lease_required": False,
            "torque_released": bool(released),
            "stepped": False,
            "emergency_stop": snapshot,
            "evidence_scope": "adapter_state_only",
        }

    def emergency_stop_confirmed(self):
        return {
            "simulation": bool(self.declaration.get("simulation", False)),
            "confirmed": bool(self.estop.engaged),
            "emergency_stop": self.estop.snapshot(),
        }

    def clear_emergency_stop(self, lease, authorization):
        """授权复位：必须带有效租约与授权依据；复位本身不重新调度任何运动。"""
        self.require_lease(lease, "emergency_stop")
        snapshot = self.estop.clear(authorization)
        return {
            "simulation": bool(self.declaration.get("simulation", False)),
            "cleared": True,
            "authorization": str(authorization),
            "fencing_token": int(lease.fencing_token),
            "emergency_stop": snapshot,
        }

    def describe(self):
        """契约摘要：能力词表、已实现集合、声明与实现的差集（供审计与预检读取）。"""
        declared = sorted(str(item) for item in (getattr(self.profile, "capabilities", None) or ()))
        implemented = sorted(self.IMPLEMENTED_CAPABILITIES)
        return {
            "schema_version": "iraf.quadruped-adapter/v1",
            "adapter": "%s:%s" % (type(self).__module__, type(self).__name__),
            "robot": str(getattr(self.profile, "name", "")),
            "profile_version": str(getattr(self.profile, "version", "")),
            "simulation": bool(self.declaration.get("simulation", False)),
            "capability_vocabulary": list(CAPABILITIES),
            "declared_capabilities": declared,
            "implemented_capabilities": implemented,
            "declared_but_not_implemented": sorted(set(declared) - set(implemented)),
            "implemented_but_not_declared": sorted(set(implemented) - set(declared)),
            "state_keys": list(STATE_KEYS),
            "velocity_fields": list(VELOCITY_FIELDS),
            "control_source": dict(self._control_source),
            "emergency_stop": self.estop.snapshot(),
        }

    # ---- 契约守卫（子类在每次运动调用前必须走这些路径）----
    @staticmethod
    def _unsupported(capability):
        return "能力 %s 未在本后端实现：显式拒绝（不返回伪造成功）" % capability

    def require_capability(self, capability):
        if capability not in CAPABILITIES:
            raise DeclarationError("未知能力，不在规范词表内: %s" % capability)
        if capability not in self.IMPLEMENTED_CAPABILITIES:
            raise UnsupportedCapabilityError(self._unsupported(capability))
        return True

    def require_lease(self, lease, capability):
        if lease is None:
            raise ControlAuthorityError(
                "能力 %s 需要控制权租约：无租约不得驱动执行器" % capability
            )
        resource = str(getattr(lease, "resource", ""))
        if not resource:
            raise ControlAuthorityError("租约缺少资源标识，无法证明控制权归属")
        try:
            self.authority.validate(lease)
        except LeaseConflict as exc:
            raise ControlAuthorityError(
                "控制权租约无效（旧 fencing token / 已过期 / 已被他人接管）: %s" % exc
            )
        return lease

    def require_motion_allowed(self, capability, lease):
        """运动三步守卫：能力已实现 → 无安全闭锁 → 持有效租约。"""
        self.require_capability(capability)
        self.estop.assert_clear()
        self.require_lease(lease, capability)
        return True

    def begin_execution(self, capability, lease):
        execution_id = self.ledger.begin(capability, lease.fencing_token)
        self._control_source = {
            "owner": str(getattr(lease, "owner", "")),
            "fencing_token": int(lease.fencing_token),
            "execution_id": execution_id,
        }
        return execution_id

    def require_active_execution(self, execution_id, lease, capability=None):
        """同一执行内的后续控制请求：执行必须仍在活动态、且租约仍是同一 token。

        终态执行（`SUCCEEDED` / `STOPPED` / `SAFETY_STOP` …）的控制请求在这里被拒绝：
        终态不得被回写为成功，也不得继续驱动执行器（铁律 1.6 / 1.12）。
        """
        # 先判「执行是否存在 / 能力是否对得上」（结构性错误，与状态无关，报错更精确），
        # 再判终态与 token；三条检查都在下发任何控制量之前完成。
        entry = self.ledger.entry(execution_id)
        if capability is not None and entry["capability"] != str(capability):
            raise CommandRejectedError(
                "执行 %s 属于能力 %s，不能用能力 %s 续跑"
                % (execution_id, entry["capability"], capability)
            )
        entry = self.ledger.assert_active(execution_id)
        self.require_lease(lease, entry["capability"])
        if int(lease.fencing_token) != int(entry["fencing_token"]):
            raise ControlAuthorityError(
                "执行 %s 由 fencing token %d 建立，当前租约 token %d：旧 token 不得驱动该执行"
                % (execution_id, entry["fencing_token"], int(lease.fencing_token))
            )
        return entry

    def resolve_targets(self, targets=None):
        """关节目标位形：默认取 Profile 的 `spec.home`（单一事实来源），显式目标只能收紧。"""
        joints = [str(item) for item in (getattr(self.profile, "joints", None) or ())]
        if not joints:
            raise DeclarationError("Profile 未声明关节，无法解析目标位形")
        if targets is None:
            home = getattr(self.profile, "home", None) or {}
            missing = [name for name in joints if name not in home]
            if missing:
                raise DeclarationError(
                    "Profile 未声明 spec.home（或缺少关节 %s），且调用未显式给出目标位形："
                    "禁止默认值兜底" % missing
                )
            return {name: float(home[name]) for name in joints}, "profile_home"
        if not isinstance(targets, dict) or not targets:
            raise CommandRejectedError("targets 必须是非空对象（关节名 -> 目标角度）")
        unknown = sorted(str(name) for name in targets if str(name) not in joints)
        if unknown:
            raise CommandRejectedError("未知关节名（身份只能来自 Profile）: %s" % unknown)
        limits = dict(getattr(self.profile, "joint_limits", None) or {})
        resolved = {}
        for name, value in targets.items():
            name = str(name)
            try:
                number = float(value)
            except (TypeError, ValueError):
                raise CommandRejectedError("关节 %s 的目标不是数值: %r" % (name, value))
            if not math.isfinite(number):
                raise CommandRejectedError("关节 %s 的目标不是有限数值: %r" % (name, value))
            lower, upper = limits[name]
            if number < float(lower) or number > float(upper):
                raise CommandRejectedError(
                    "关节 %s 目标 %r 越出声明限位 [%r, %r]：调用只能收紧、不能放宽"
                    % (name, number, float(lower), float(upper))
                )
            resolved[name] = number
        # 部分目标：未指定的关节必须回落到 Profile 的 `spec.home`（声明来源），
        # **不得**回落到"当前位置"——那是隐式默认值，会让"保持姿态"与"目标姿态"混淆。
        missing = [name for name in joints if name not in resolved]
        if missing:
            home = getattr(self.profile, "home", None) or {}
            absent = [name for name in missing if name not in home]
            if absent:
                raise DeclarationError(
                    "部分目标缺少关节 %s，且 Profile 的 spec.home 也未声明它们："
                    "禁止用当前位置兜底" % absent
                )
            for name in missing:
                resolved[name] = float(home[name])
            return resolved, "explicit_partial"
        return resolved, "explicit"

    def resolve_velocity(self, velocity):
        """速度指令只接受规范字段；未知字段显式拒绝（厂家语义不得渗入通用契约）。"""
        if not isinstance(velocity, dict) or not velocity:
            raise CommandRejectedError("速度指令必须是非空对象: %r" % (velocity,))
        unknown = sorted(str(key) for key in velocity if str(key) not in VELOCITY_FIELDS)
        if unknown:
            raise CommandRejectedError("速度指令含未知字段: %s（规范字段 %s）" % (unknown, list(VELOCITY_FIELDS)))
        missing = [field for field in VELOCITY_FIELDS if field not in velocity]
        if missing:
            raise CommandRejectedError("速度指令缺少字段: %s" % missing)
        parsed = {}
        for field in VELOCITY_FIELDS:
            try:
                number = float(velocity[field])
            except (TypeError, ValueError):
                raise CommandRejectedError("速度字段 %s 不是数值: %r" % (field, velocity[field]))
            if not math.isfinite(number):
                raise CommandRejectedError("速度字段 %s 不是有限数值: %r" % (field, velocity[field]))
            parsed[field] = number
        return parsed

    @staticmethod
    def resolve_duration_ms(duration_ms, declared_seconds):
        if duration_ms is None:
            seconds = float(declared_seconds)
        else:
            try:
                seconds = float(duration_ms) / 1000.0
            except (TypeError, ValueError):
                raise CommandRejectedError("duration_ms 必须是数值: %r" % (duration_ms,))
        if not math.isfinite(seconds) or seconds <= 0:
            raise CommandRejectedError("时长必须为正的有限数值，实际: %r" % (seconds,))
        return seconds

    def _apply_zero_torque(self):
        raise UnsupportedCapabilityError(
            "后端未实现零力矩释放（_apply_zero_torque）：拒绝伪造安全动作"
        )
