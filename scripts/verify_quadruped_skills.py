"""四足技能验收入口（步骤 17）：`--config <机型声明>` → 报告 + 退出码。

链路（不绕过任何一层）：TaskFlow → SkillRuntime → PolicyGateway/ControlAuthority →
Provider（`iraf_skills.quadruped`）→ 适配器（`UnitreeGo2Adapter`）→ MuJoCo。
本文件只是入口层：解析参数、装配运行时、逐用例执行、映射退出码；判据实现放在
`iraf_skills.quadruped`（可导入、可逐项断言）。

数字与路径全部来自声明（铁律 5.3）：
- `config/go2_loopback.yaml`：机型/模型/Profile/安全策略/报告路径/拒绝用例数下限/标称序列；
- `profiles/safety/quadruped_lab.yaml`：放行清单 + 四足速度与工作空间上限；
- `profiles/unitree_go2_mujoco.yaml`：关节身份、限位、站立参考位形与能力声明。
本文件不含任何阈值/路径默认值。

用法::

    PYTHONPATH=src python3 scripts/verify_quadruped_skills.py --config config/go2_loopback.yaml

退出码：

- `0` 全部用例通过（成功路径 SUCCEEDED + 全部期望拒绝均命中预期错误码）
- `1` 用法错误（缺参数、声明文件不存在）
- `2` 声明非法（缺必需键、安全策略缺 quadruped_limits、Profile/安全策略非法）
- `3` 引用完整性失败（技能目录缺失、模型未生成、Profile 不存在）
- `4` 后端装配失败（能力契约/模型编译/关键帧/绑定）
- `5` 判据未通过（报告 `failed_checks` 逐条列出）

诚实边界：全部结论属于**仿真**（报告 `simulation: true`）；标称序列里**没有行走**
（决策 4.B 首期无步态控制器，locomote 只交付显式拒绝路径，不得用 mock 步态补进成功路径）；
目标端/真机验收一律 DEFERRED（板卡不在场），本入口不得被表述为真机能力验收。
"""

import argparse
import json
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import RegistryError, SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402
from iraf_skills import quadruped as quadruped_skills  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_BACKEND = 4
EXIT_CRITERIA = 5

#: 机型后端入口从 factory 的登记表取（禁止在本脚本硬编码模块路径）。
ROBOT_BACKEND = "unitree_go2_mujoco"

#: 技能层声明的必需键（缺键 -> 退出码 2）。
REQUIRED_SKILL_KEYS = (
    "skills.safety_policy",
    "skills.report",
    "skills.minimum_rejection_cases",
    "skills.nominal_sequence",
    "skills.refused_capability",
)

SUBMITTER_ROLES = frozenset({"task.submit", "task.read"})

#: 区分"未给出上下文"与"显式给出 None（未认证）"——否则未认证用例会被默认上下文掩盖。
_UNSET = object()


def _dig(document, dotted, label="声明"):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            raise quadruped_contract.DeclarationError("%s 缺少声明键: %s" % (label, dotted))
        node = node[part]
    return node


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def _rel(path, root):
    try:
        return Path(path).resolve().relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return str(path)


def _request(profile, safety, skill, parameters, resource_id, case, *, deadline_offset_ms=60000,
             idempotency_key=None, version="1.0.0"):
    correlation = "go2-skills-" + case
    return {
        "request_id": correlation,
        "idempotency_key": idempotency_key or (correlation + "-" + uuid.uuid4().hex),
        "correlation_id": correlation,
        "skill": skill,
        "skill_version_constraint": version,
        "parameters": parameters,
        "deadline_unix_ms": int(time.time() * 1000) + int(deadline_offset_ms),
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": resource_id,
        "controller": correlation,
    }


def _context(subject, roles=None):
    return AuthenticatedContext(str(subject), frozenset(roles or SUBMITTER_ROLES), "local")


def _case_record(case, skill, kind, result):
    return {
        "case": case,
        "skill": skill,
        "kind": kind,
        "status": str(result.get("status", "")),
        "error_code": str(result.get("error_code", "")),
        "reason": str(result.get("reason", "")),
        "execution_id": str(result.get("execution_id", "")),
    }


def _run_success(runtime, profile, safety, resource_id, case, skill, parameters, backend=None,
                 workspace_check=False, limits=None):
    record = {"case": case, "skill": skill, "kind": "success"}
    before = backend.read_state() if (workspace_check and backend is not None) else None
    started = time.perf_counter()
    result = runtime.execute(
        _request(profile, safety, skill, parameters, resource_id, case), _context(case)
    )
    # 墙钟时长要记录：技能租约 TTL = Skill 声明的 timeoutSeconds，必须覆盖一次执行的墙钟耗时，
    # 否则租约会在执行中过期（释放租约时抛 LeaseConflict）。这条数字是"租约够不够"的证据。
    record["wall_seconds"] = time.perf_counter() - started
    record.update(_case_record(case, skill, "success", result))
    evidence = (result.get("result") or {}).get("evidence") or {}
    if result.get("status") == "SUCCEEDED":
        record["provider"] = (result.get("result") or {}).get("skill")
        record["evidence_digest_keys"] = sorted(evidence)
        if workspace_check:
            workspace = quadruped_skills.check_workspace(before, evidence["final_state"], limits)
            record["workspace"] = workspace
            record["checks"] = workspace["checks"]
            record["passed"] = bool(workspace["passed"])
        else:
            record["passed"] = True
    else:
        record["passed"] = False
    return record


def _rejection_cases(runtime, profile, safety, resource_id, limits, declaration):
    """逐条拒绝路径：每条都必须给出**错误码 + 非空原因**，且原因要能指认拒绝维度。"""
    records = []
    refused = str(declaration["skills"]["refused_capability"])

    def run(case, skill, parameters, expected_code, reason_contains, *, context=_UNSET,
            deadline_offset_ms=60000, pipeline="runtime", subject=None, idempotency_key=None):
        case_context = _context(subject or case) if context is _UNSET else context
        if pipeline == "pre_dispatch_limit":
            # 调度前门禁：速度上限来自安全策略声明，超限即拒绝并落执行记录。
            request = _request(profile, safety, skill, parameters, resource_id, case,
                               idempotency_key=idempotency_key)
            try:
                quadruped_skills.enforce_velocity_limits(parameters["velocity"], limits)
                record = {"case": case, "skill": skill, "kind": "expected_rejection",
                          "status": "SUCCEEDED", "error_code": "", "reason": "",
                          "detail": "声明上限内的速度指令不应被门禁拒绝（正向对照）"}
            except quadruped_skills.SkillContractError as exc:
                result = runtime.record_pre_dispatch_failure(
                    request, case_context, exc.code, str(exc),
                    metadata={"resolved_skill": skill},
                )
                record = _case_record(case, skill, "expected_rejection", result)
                record["pipeline"] = "pre_dispatch_limit"
        else:
            result = runtime.execute(
                _request(profile, safety, skill, parameters, resource_id, case,
                         deadline_offset_ms=deadline_offset_ms, idempotency_key=idempotency_key),
                case_context,
            )
            record = _case_record(case, skill, "expected_rejection", result)
        record["expected_error_code"] = expected_code
        record["expected_reason_contains"] = reason_contains
        record["passed"] = bool(
            record["error_code"] == expected_code
            and reason_contains in record["reason"]
            and record["reason"].strip() != ""
        )
        records.append(record)
        return record

    run("reject-deadline-expired", "stand", {}, "IRAF-DEADLINE-EXCEEDED", "截止时间",
        deadline_offset_ms=-1000)
    run("reject-rbac-no-submit-role", "stand", {}, "IRAF-POLICY-DENIED", "权限",
        context=_context("go2-skills-rbac", {"task.read"}))
    run("reject-unauthenticated-context", "stand", {}, "IRAF-UNAUTHENTICATED",
        "missing authenticated context", context=None)
    run("reject-capability-not-declared", refused,
        {"velocity": {"vx_mps": 0.2, "vy_mps": 0.0, "wz_rad_s": 0.0}},
        "IRAF-SKILL-PROVIDER-UNAVAILABLE", "缺少能力")
    run("reject-velocity-limit-exceeded", refused,
        {"velocity": {"vx_mps": 5.0, "vy_mps": 0.0, "wz_rad_s": 0.0}},
        quadruped_skills.COMMAND_REJECTED_CODE, "超过声明上限", pipeline="pre_dispatch_limit")
    run("reject-stand-target-out-of-limit", "stand", {"joint_targets": {"FL_calf_joint": 0.0}},
        "IRAF-EXECUTION-FAILED", "越出声明限位")

    # 过期/重复状态：同一 idempotency_key 映射到不同请求，必须拒绝而不是改正后重跑。
    # 幂等键按 subject 作用域（store 的 UNIQUE(subject, idempotency_key)），因此两次请求必须
    # 用同一 subject——否则第二次请求会被当成全新执行，用例就变成"恒通过"的假用例。
    key = "go2-skills-stale-" + uuid.uuid4().hex
    stale_subject = "go2-skills-stale"
    preparation = runtime.execute(
        _request(profile, safety, refused, {"velocity": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}},
                 resource_id, "reject-stale-idempotency-key-seed", idempotency_key=key),
        _context(stale_subject),
    )
    record = run("reject-stale-idempotency-key", "stand", {}, "IRAF-IDEMPOTENCY-CONFLICT",
                 "same idempotency key", subject=stale_subject, idempotency_key=key)
    record["preparation"] = _case_record("reject-stale-idempotency-key-seed", refused, "seed", preparation)
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=None, help="四足机型声明（config/go2_loopback.yaml）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    args = parser.parse_args(argv)

    if args.config is None:
        print("用法错误：必须给出 --config <声明文件>", file=sys.stderr)
        return EXIT_USAGE
    root = args.root or unitree_go2.repo_root()
    config_path = _resolve(root, args.config)
    if not config_path.is_file():
        print("用法错误：声明文件不存在: %s" % config_path, file=sys.stderr)
        return EXIT_USAGE

    try:
        declaration, _ = unitree_go2.load_declaration(config_path)
        for key in REQUIRED_SKILL_KEYS:
            _dig(declaration, key)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    skills_declaration = declaration["skills"]
    profile_path = _resolve(root, _dig(declaration, "robot.profile"))
    safety_path = _resolve(root, skills_declaration["safety_policy"])
    report_path = _resolve(root, args.report or skills_declaration["report"])

    try:
        profile = load_robot_profile(profile_path)
        safety = load_safety_policy(safety_path)
    except ProfileError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    try:
        limits = quadruped_skills.load_quadruped_limits(safety_path)
    except quadruped_skills.SkillContractError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    skill_root = Path(root) / "skills"
    if not skill_root.is_dir():
        print("引用完整性失败：技能目录不存在: %s" % skill_root, file=sys.stderr)
        return EXIT_REFERENCE
    try:
        registry = SkillRegistry().load_directory(skill_root)
    except RegistryError as exc:
        print("引用完整性失败：技能清单无法加载: %s" % exc, file=sys.stderr)
        return EXIT_REFERENCE

    authority = ControlAuthorityManager()
    try:
        backend = load_backend(
            KNOWN_BACKENDS[ROBOT_BACKEND], str(config_path), profile, authority
        )
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（模型/关键帧不可用）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except (quadruped_contract.DeclarationError, ProfileError, ValueError) as exc:
        print("后端装配失败（声明/契约）：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND
    except Exception as exc:  # 模型编译等：显式失败，不返回半成品
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    runtime = SkillRuntime(
        profile, safety, backend, registry, authority, SqliteExecutionStore(":memory:")
    )
    resource_id = profile.name + "-mujoco"

    # 证据里的路径一律仓库相对（步骤 06 的纪律）：describe() 给的是本机绝对路径，
    # 直接写进报告会让证据带机器相关噪声。
    backend_summary = backend.describe()
    backend_summary["model"]["path"] = _rel(getattr(backend, "model_path", ""), root)

    cases = []
    for skill in [str(item) for item in skills_declaration["nominal_sequence"]]:
        cases.append(
            _run_success(
                runtime, profile, safety, resource_id, "%s-success" % skill, skill, {},
                backend=backend, workspace_check=(skill == "stand"), limits=limits,
            )
        )
    rejections = _rejection_cases(runtime, profile, safety, resource_id, limits, declaration)
    cases.extend(rejections)

    failed_checks = [
        "%s（期望 %s，实际 %s：%s）" % (
            item["case"],
            item.get("expected_error_code", "SUCCEEDED"),
            item.get("error_code") or item.get("status"),
            item.get("reason") or item.get("detail", ""),
        )
        for item in cases
        if not item.get("passed")
    ]
    minimum = int(skills_declaration["minimum_rejection_cases"])
    if len(rejections) < minimum:
        failed_checks.append(
            "拒绝用例数 %d 少于声明下限 %d" % (len(rejections), minimum)
        )

    report = {
        "schema_version": "iraf.quadruped-skills-acceptance/v1",
        "simulation": True,
        "config": _rel(config_path, root),
        "robot": str(declaration["robot"]["id"]),
        "profile": {
            "path": _rel(profile_path, root),
            "name": profile.name,
            "version": profile.version,
            "digest": profile.digest,
            "capabilities": sorted(str(item) for item in profile.capabilities),
        },
        "safety_policy": {
            "path": _rel(safety_path, root),
            "name": safety.name,
            "version": safety.version,
            "digest": safety.digest,
            "simulation_only": bool(safety.simulation_only),
            "allowed_skills": sorted(str(item) for item in safety.allowed_skills),
        },
        "limits": limits,
        "nominal_sequence": [str(item) for item in skills_declaration["nominal_sequence"]],
        "refused_capability": str(skills_declaration["refused_capability"]),
        "backend": {
            "entrypoint": KNOWN_BACKENDS[ROBOT_BACKEND],
            "model": _rel(getattr(backend, "model_path", ""), root),
            "capabilities": backend_summary,
        },
        "cases": cases,
        "rejection_cases": len(rejections),
        "minimum_rejection_cases": minimum,
        "failed_checks": failed_checks,
        "not_proved": [
            "行走（locomote）未实现：首期没有步态控制器，本轮只交付显式拒绝路径；"
            "步态/导航/停靠能力未验收（AGENTS.md 6.8 禁止用低层模型证据替代）。",
            "locomote 输出 schema 的成功路径未被触发（能力被拒绝），该 schema 仅为契约声明。",
            "目标端/真机验收 DEFERRED：板卡不在场，本报告全部结论为 simulation=true 的仿真证据。",
        ],
        "passed": not failed_checks,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {
        "passed": report["passed"],
        "report_path": str(args.report or skills_declaration["report"]),
        "skills": [
            {"case": item["case"], "kind": item["kind"], "status": item.get("status"),
             "error_code": item.get("error_code"), "passed": item.get("passed")}
            for item in cases
        ],
        "rejection_cases": len(rejections),
        "minimum_rejection_cases": minimum,
        "failed_checks": failed_checks,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return EXIT_OK if report["passed"] else EXIT_CRITERIA


if __name__ == "__main__":
    raise SystemExit(main())
