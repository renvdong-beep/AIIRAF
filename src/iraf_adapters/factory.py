"""从部署配置装配 Capability Backend，并在装配期完成契约校验。

契约先于实现（AGENTS.md 铁律 1）与失败即显式（铁律 5）要求：
Profile 声明的 capabilities 必须与 Backend 实际实现的方法一致，
且不一致必须在**装配期**就暴露，而不是等某个任务跑到一半才由
PolicyGateway 报 "RobotProfile 缺少能力" 或 Provider 报
"Backend 未实现 xxx，拒绝伪造成功"。

本模块因此做两层校验：
1. capabilities 声明的每一项都必须能在 Backend 上找到对应实现；
2. 反向检查：Backend 实现了运动能力但 Profile 未声明时给出显式告警级错误，
   避免"声明缺项导致任务被策略静默拒绝"这种难以定位的问题。

校验不改变既有调用签名，load_backend 仍是 (entrypoint, config, profile,
authority) -> backend。
"""
import importlib

#: capability 名 -> Backend 上必须存在的方法名。
#: 只声明"能力名到方法的映射"，不假设任何具体机型的实现细节，
#: 因此新机器人只需实现同名方法即可复用本校验。
#: 该表覆盖 profiles/ 中实际声明过的全部能力，新增能力时同步登记，
#: 未登记的能力会被显式拦下（避免"策略通过但无人实现"的悬空能力）。
CAPABILITY_METHODS = {
    "move_joint": "move_joint",
    "pick_object": "pick_object",
    "visual_pick": "visual_pick",
    "calibrate_grasp": "calibrate_grasp",
    "calibrate_camera_to_base": "calibrate_camera_to_base",
    "stop": "stop",
    "step": "step",
}

#: 运动能力清单：用于反向检查"实现了但未声明"的高风险不一致。
#: 只包含会驱动机械臂的能力；标定类能力不产生物理动作，故不在此列。
MOTION_CAPABILITIES = ("move_joint", "pick_object", "visual_pick", "stop")


class BackendContractError(ValueError):
    """装配期契约校验失败。"""


def _resolve_class(entrypoint):
    if not entrypoint or ":" not in entrypoint:
        raise ValueError("IRAF_BACKEND_ENTRYPOINT 无效")
    module_name, class_name = entrypoint.split(":", 1)
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise BackendContractError(
            "无法导入 Backend 模块: " + str(module_name)
        ) from exc
    try:
        return getattr(module, class_name)
    except AttributeError as exc:
        raise BackendContractError(
            "Backend 模块缺少类: " + str(class_name)
        ) from exc


def _declared_capabilities(profile):
    """读取 Profile 声明的能力集合；缺失时视为空集（由调用方决定语义）。"""
    raw = getattr(profile, "capabilities", None)
    if raw is None:
        return frozenset()
    return frozenset(str(item) for item in raw)


def verify_backend_contract(backend_class, profile):
    """交叉核对 Profile.capabilities 与 Backend 实现，返回校验报告。

    校验对象是 **类** 而非实例：这样问题在实例化之前就暴露，
    且对实现了 __getattr__ 的代理型 backend 也能给出明确结论。
    """
    # from_config 是最基本的装配契约，先于能力核对检查，
    # 这样"根本不是合法 Backend"的问题不会被能力漂移的消息掩盖。
    if not callable(getattr(backend_class, "from_config", None)):
        raise ValueError("Backend 必须实现 from_config")

    declared = _declared_capabilities(profile)
    missing = []
    for capability in sorted(declared):
        method = CAPABILITY_METHODS.get(capability)
        if method is None:
            # Profile 声明了本模块不认识的 capability：不静默放过，
            # 因为它会导致"策略通过了但没人实现"的悬空能力。
            missing.append((capability, "<未登记的 capability>"))
            continue
        if not callable(getattr(backend_class, method, None)):
            missing.append((capability, method))
    if missing:
        raise BackendContractError(
            "Backend 未实现 RobotProfile 声明的能力: "
            + ", ".join("%s(需 %s)" % (capability, method) for capability, method in missing)
        )

    # 反向检查：实现了运动能力却未声明，属于配置漂移，必须显式暴露。
    undeclared = []
    for capability in MOTION_CAPABILITIES:
        method = CAPABILITY_METHODS.get(capability)
        if method is None or capability in declared:
            continue
        if callable(getattr(backend_class, method, None)):
            undeclared.append(capability)
    if undeclared:
        raise BackendContractError(
            "Backend 实现了未声明的运动能力，会导致策略静默拒绝任务: "
            + ", ".join(sorted(undeclared))
        )

    return {
        "schema_version": "iraf.backend-contract/v1",
        "backend": backend_class.__module__ + ":" + backend_class.__name__,
        "profile_name": str(getattr(profile, "name", "")),
        "declared_capabilities": sorted(declared),
        "verified_methods": {
            capability: CAPABILITY_METHODS[capability]
            for capability in sorted(declared)
            if capability in CAPABILITY_METHODS
        },
        "undeclared_motion_capabilities": [],
        "passed": True,
    }


def load_backend(entrypoint, config, profile, authority):
    """装配 Backend；装配期即完成契约校验，失败不返回半成品。"""
    backend_class = _resolve_class(entrypoint)
    report = verify_backend_contract(backend_class, profile)
    backend = backend_class.from_config(config, profile, authority)
    # 把报告挂到实例上，便于运行期审计与验收脚本读取；
    # 不参与任何运动链路，仅作为装配证据。
    try:
        backend.backend_contract = report
    except (AttributeError, TypeError):
        # 实现方可能禁止动态属性；此时契约校验已完成，不影响装配结果。
        pass
    return backend
