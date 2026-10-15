#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安装形态声明校验（只读）：config/install/install_profiles.yaml ↔ 其 schema ↔ 仓库事实。

为什么要有这个入口（本项目反复踩过的三类坑）：
  ① 声明改了却不生效/改错了段（同名键出现在多个段）⇒ 这里用 `--profile <id>` 打印**解析后的生效值**；
  ② 同一事实两处（端口同时写在 config/install 与 deploy/iraf-runtime.env）⇒ 这里做**交叉校验**，
     不一致即 fail-closed 并打印两处的名字与取值；
  ③ 版本号被复制到多处 ⇒ 这里按 `version_ref`（pyproject.toml#project.version）实读比对，拒绝硬编码。

边界：本脚本**只读**，不写文件、不装任何东西；它不构成安装证据。

用法：
    python3 scripts/install_profile_check.py                     # 校验全部角色
    python3 scripts/install_profile_check.py --profile edge-intel # 额外打印该角色的生效值（含默认值合并）
    python3 scripts/install_profile_check.py --json               # 只输出 JSON 报告

退出码（与 docs/iraf-multiplatform-sdk-design.md §5 统一约定一致）：
    0 校验通过   1 参数错误   2 声明/预检失败（schema 不符、语义冲突、交叉校验不一致）
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - 环境缺依赖时给出可操作中文原因
    print("[install-profile-check][错误] 缺少 PyYAML：请先 pip install PyYAML", file=sys.stderr)
    raise SystemExit(2)

try:
    from jsonschema import Draft7Validator
except ImportError:  # pragma: no cover
    print("[install-profile-check][错误] 缺少 jsonschema：请先 pip install jsonschema", file=sys.stderr)
    raise SystemExit(2)

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2

REPO_ROOT = Path(__file__).resolve().parents[1]
DECLARATION = Path("config/install/install_profiles.yaml")
SCHEMA = Path("config/install/install_profiles.schema.json")
RUNTIME_ENV = Path("deploy/iraf-runtime.env")

#: 形态定义（写死在**校验脚本**里，不是写死在安装脚本里）：形态 A 是单机 Ubuntu 22.04，
#: 边缘角色含"真机档 edge-intel / edge-motion"与"仿真档 edge-intel-sim"（默认仿真、场景由 AgentOS 语义触发）；
#: 形态 B 的仿真侧同样是 x86 Ubuntu 22.04，边缘两角色必须是 aarch64。
X86_FORM_IDS = ("single-ubuntu2204", "sim-x86")
EDGE_FORM_IDS = ("edge-intel", "edge-motion", "edge-intel-sim")
REQUIRED_UBUNTU_VERSION = "22.04"

#: 交付单元模板**必须**包含的占位符：机器相关的路径/解释器一律由安装器按声明渲染，
#: 不得写死在模板里（AGENTS 5.3）。缺一即拒绝。
REQUIRED_UNIT_TOKENS = ("@PREFIX@", "@CURRENT_LINK@", "@PYTHON_BIN@", "@CONFIG_DIR@", "@LOG_DIR@")

#: 模板里**不允许**出现的开发机痕迹（实测踩到：deploy/iraf-runtime*.service 写死
#: /home/coretek、User=coretek、miniconda 解释器 ⇒ 装到别的机器必然错，却被当作交付单元声明）。
UNIT_TEMPLATE_DENYLIST = ("/home/", "miniconda", "User=coretek")


class CheckError(Exception):
    """声明不成立（退出码 2）。"""


def load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise CheckError("缺少文件：%s" % path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise CheckError("%s 不是合法 YAML：%s" % (path, exc))
    if not isinstance(data, dict):
        raise CheckError("%s 的根结构必须是映射" % path)
    return data


def validate_schema(declaration: dict, schema: dict) -> list:
    validator = Draft7Validator(schema)
    errors = []
    for error in sorted(validator.iter_errors(declaration), key=lambda item: list(item.absolute_path)):
        where = "/".join(str(part) for part in error.absolute_path) or "<根>"
        errors.append("%s：%s" % (where, error.message))
    return errors


def expand(path_value: str) -> str:
    """唯一的路径展开规则：前导 `~` 展开为 $HOME；其余原样（见声明文件头说明）。"""
    if path_value.startswith("~/"):
        return str(Path.home() / path_value[2:])
    return path_value


def resolve(profile: dict, defaults: dict) -> dict:
    """把 defaults 与单角色覆盖合并，并把 `{...}` 占位符解析成生效值。

    只允许两类展开（声明文件头写明的唯一规则）：
      · 前导 `~` → $HOME
      · `{prefix}` / `{config_dir}` / `{log_dir}` / `{data_dir}` / `{version}` → 对应生效值
    `--profile <id>` 会打印这里的输出，用来防“改了声明却改错段/没生效”。
    """
    effective = {}
    for key in ("prefix", "config_dir", "log_dir", "data_dir", "install_method",
                "systemd_scope", "dependency_source"):
        effective[key] = profile.get(key, defaults.get(key))
    context = {
        "prefix": expand(str(effective["prefix"])),
        "config_dir": expand(str(effective["config_dir"])),
        "log_dir": expand(str(effective["log_dir"])),
        "data_dir": expand(str(effective["data_dir"])),
        "version": "<version>",
    }

    def substitute(text) -> str:
        return re.sub(r"\{([a-z_]+)\}",
                      lambda match: context.get(match.group(1), match.group(0)), str(text))

    effective["prefix"] = context["prefix"]
    effective["config_dir"] = context["config_dir"]
    effective["log_dir"] = context["log_dir"]
    effective["data_dir"] = context["data_dir"]
    effective["current_link"] = defaults["current_link"]
    effective["version_dir"] = substitute(defaults["version_dir_layout"])
    effective["file_record"] = defaults["file_record"]
    effective["uninstall_keep"] = [substitute(item) for item in defaults["uninstall"]["keep"]]
    effective["residue_scan_roots"] = [substitute(item) for item in profile["residue"]["scan_roots"]]
    effective["residue_units"] = list(profile["residue"]["units"])
    effective["residue_ports"] = list(profile["residue"]["ports"])
    effective["residue_processes"] = list(profile["residue"]["processes"])
    effective["install_boundaries"] = defaults["install_boundaries"]
    return effective


def read_toml_field(repo_root: Path, ref: str) -> str:
    """读 `pyproject.toml#project.version` 这类引用；不引入额外依赖（configparser 关闭插值）。"""
    file_part, _, key_path = ref.partition("#")
    path = repo_root / file_part
    if not path.is_file():
        raise CheckError("版本引用指向的文件不存在：%s" % file_part)
    text = path.read_text(encoding="utf-8")
    field = key_path.split(".")[-1]
    match = re.search(r'^\s*%s\s*=\s*"([^"]+)"' % re.escape(field), text, flags=re.MULTILINE)
    if not match:
        raise CheckError("%s 里找不到 %s（版本单一来源失效）" % (file_part, key_path))
    return match.group(1).strip()


def read_runtime_env_ports(repo_root: Path) -> dict:
    """实读 deploy/iraf-runtime.env 里的端口声明（交叉校验用；只读）。"""
    path = repo_root / RUNTIME_ENV
    if not path.is_file():
        raise CheckError("缺少 %s（端口事实来源之一，无法交叉校验）" % RUNTIME_ENV)
    ports = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("IRAF_RUNTIME_PORT=") or line.startswith("IRAF_GRPC_PORT="):
            name, _, value = line.partition("=")
            value = value.strip()
            if not value.isdigit():
                raise CheckError("%s 的 %s 不是整数：%r" % (RUNTIME_ENV, name, value))
            ports[name] = int(value)
    if not ports:
        raise CheckError("%s 里没有 IRAF_RUNTIME_PORT / IRAF_GRPC_PORT 声明" % RUNTIME_ENV)
    return ports


def check_semantics(declaration: dict, repo_root: Path) -> tuple:
    """返回 (checks, notes)：checks 是 (检查名, 是否通过, 说明) 列表。"""
    checks = []
    notes = []
    components_enum = set(declaration["components_enum"])
    profiles = declaration["profiles"]
    defaults = declaration["defaults"]

    def add(name, ok, detail):
        checks.append({"name": name, "passed": bool(ok), "detail": detail})

    ids = [str(item["id"]) for item in profiles]
    expected_ids = set(X86_FORM_IDS) | set(EDGE_FORM_IDS)
    add("角色 id 唯一", len(ids) == len(set(ids)), "profiles=%s" % ids)
    add("正式角色齐备（含板卡仿真档）", set(ids) == expected_ids,
        "缺少=%s 多余=%s" % (sorted(expected_ids - set(ids)), sorted(set(ids) - expected_ids)))

    port_owner = {}
    for profile in profiles:
        pid = str(profile["id"])
        components = list(profile["components"])
        unknown = [item for item in components if item not in components_enum]
        add("%s：组件在受控枚举内" % pid, not unknown, "未登记=%s" % unknown)
        forbidden = list(profile.get("forbidden_components") or [])
        overlap = [item for item in components if item in forbidden]
        add("%s：禁用组件未同时被安装" % pid, not overlap, "同时出现在 components 与 forbidden_components=%s" % overlap)
        add("%s：禁用组件在受控枚举内" % pid, not [i for i in forbidden if i not in components_enum],
            "未登记=%s" % [i for i in forbidden if i not in components_enum])

        target = profile["target"]
        if pid in X86_FORM_IDS:
            ok = (target["arch"] == "x86_64" and target["os"] == "ubuntu"
                  and target["os_version"] == REQUIRED_UBUNTU_VERSION)
            add("%s：x86 形态必须是 Ubuntu %s x86_64" % (pid, REQUIRED_UBUNTU_VERSION), ok,
                "实读 os=%s os_version=%s arch=%s" % (target["os"], target["os_version"], target["arch"]))
        if pid in EDGE_FORM_IDS:
            add("%s：边缘形态必须是 aarch64" % pid, target["arch"] == "aarch64",
                "实读 arch=%s" % target["arch"])
            boundary = profile.get("safety_boundary") or {}
            add("%s：已声明安全边界不合并（铁律 6.11）" % pid, boundary.get("merge_allowed") is False,
                "safety_boundary=%s" % (boundary or "<缺失>"))
            notes.append("%s：target.os=%s / kernel=%s / python_tag=%s ⇒ 未采集前必须回填签名 "
                         "HyperProfile/Release manifest（AGENTS 6.10），否则安装只能 --allow-unverified"
                         % (pid, target["os"], target.get("kernel", "unverified"),
                            target.get("python_tag", "unverified")))

        # 仿真/真机边界（AGENTS 1.7）：按**档位**判，而不是按"是不是边缘"判。
        #  · 真机档（simulation_only: false）：不得装仿真 Provider —— 仿真不得进真机路径
        #  · 仿真档（simulation_only: true）：必须装仿真 Provider —— 否则声明了仿真却没有仿真后端
        simulation_only = bool(profile.get("simulation_only"))
        if simulation_only:
            add("%s：仿真档已安装仿真 Provider（adapters_sim）" % pid, "adapters_sim" in components,
                "simulation_only=true components=%s" % components)
        else:
            add("%s：真机档未安装仿真 Provider（adapters_sim）" % pid, "adapters_sim" not in components,
                "simulation_only=false components=%s" % components)

        units = [str(item["name"]) + ".service" for item in profile["services"]]
        residue = profile["residue"]
        add("%s：residue.units 与服务清单一致" % pid, sorted(residue["units"]) == sorted(units),
            "residue=%s services=%s" % (sorted(residue["units"]), sorted(units)))
        service_ports = sorted({int(p) for item in profile["services"] for p in (item.get("ports") or [])})
        add("%s：residue.ports 与服务端口一致" % pid, sorted(residue["ports"]) == service_ports,
            "residue=%s services=%s" % (sorted(residue["ports"]), service_ports))
        declared_ports = [int(p) for item in profile["services"] for p in (item.get("ports") or [])]
        add("%s：同一角色内端口不重复" % pid, len(declared_ports) == len(set(declared_ports)),
            "ports=%s" % declared_ports)
        for port in service_ports:
            port_owner.setdefault(port, []).append(pid)

        for item in profile["services"]:
            template = repo_root / str(item["template"])
            add("%s：unit 模板存在（%s）" % (pid, item["template"]), template.is_file(),
                str(template))
            if template.is_file():
                text = template.read_text(encoding="utf-8", errors="replace")
                missing = [token for token in REQUIRED_UNIT_TOKENS if token not in text]
                add("%s：模板 %s 含全部必需占位符" % (pid, item["name"]), not missing,
                    "缺失占位符=%s（必需：%s）" % (missing, list(REQUIRED_UNIT_TOKENS)))
                # 只检查**生效行**（注释里提到开发机不算违规；真正危险的是 ExecStart/Environment 里的硬编码）
                active = [line.strip() for line in text.splitlines()
                          if line.strip() and not line.lstrip().startswith(("#", ";"))]
                traces = sorted({token for line in active for token in UNIT_TEMPLATE_DENYLIST
                                 if token in line})
                add("%s：模板 %s 的生效行无开发机硬编码" % (pid, item["name"]), not traces,
                    "命中开发机痕迹=%s（交付单元不得写死开发机路径/用户/解释器）" % traces)

        effective = resolve(profile, defaults)
        missing_effective = [key for key in ("prefix", "config_dir", "log_dir", "data_dir",
                                            "install_method", "systemd_scope", "dependency_source")
                             if not str(effective.get(key) or "").strip()]
        add("%s：解析后关键字段齐备（defaults 继承也算）" % pid, not missing_effective,
            "缺失=%s ｜ 生效=%s" % (missing_effective, {k: effective.get(k) for k in
                                                      ("install_method", "systemd_scope")}))
        resolved_paths = {key: effective[key] for key in ("prefix", "config_dir", "log_dir", "data_dir")}
        bad_paths = {key: value for key, value in resolved_paths.items()
                     if not str(value).startswith("/")}
        add("%s：安装路径（展开后）都是绝对路径" % pid, not bad_paths,
            "非绝对=%s ｜ 生效=%s" % (bad_paths, resolved_paths))
        keep = list(effective["uninstall_keep"])
        add("%s：卸载保留项不含安装前缀本身" % pid,
            all(not str(item).startswith(effective["prefix"]) for item in keep),
            "keep=%s prefix=%s" % (keep, effective["prefix"]))

    # 版本单一来源（实读 pyproject.toml）
    version = read_toml_field(repo_root, declaration["sdk"]["version_ref"])
    dist = read_toml_field(repo_root, declaration["sdk"]["dist_name_ref"])
    add("版本来自单一来源（%s）" % declaration["sdk"]["version_ref"], bool(version), "实读 version=%s" % version)
    add("发行包名来自单一来源", bool(dist), "实读 name=%s" % dist)

    # 端口交叉校验：声明 vs deploy/iraf-runtime.env（同一事实两处即拒绝）
    env_ports = read_runtime_env_ports(repo_root)
    declared = sorted(port_owner)
    env_values = sorted(env_ports.values())
    add("端口：声明与 deploy/iraf-runtime.env 一致", declared == env_values,
        "声明=%s（%s）｜env=%s（%s）" % (declared, port_owner, env_values, env_ports))
    shared = {port: owners for port, owners in port_owner.items() if len(owners) > 1}
    if shared:
        notes.append("端口 %s 被多个角色声明 ⇒ 这些角色**不得装在同一台机器**上"
                     "（形态 B 的两域分属不同 VM；若强行同机，安装脚本会因端口占用失败，"
                     "这也是“合并不合边界”在端口层的体现）" % shared)

    boundaries = defaults["install_boundaries"]
    add("安装边界：不写系统 site-packages / 不用系统包管理器 / 不碰 AgentOS 文件",
        boundaries["write_system_site_packages"] is False
        and boundaries["use_system_package_manager"] is False
        and boundaries["touch_agentos_files"] is False,
        "install_boundaries=%s" % boundaries)
    return checks, notes


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="校验 IRAF 安装形态声明（只读）")
    parser.add_argument("--profile", help="额外打印该角色的生效值（默认值合并后的解析结果）")
    parser.add_argument("--json", action="store_true", help="只输出 JSON 报告")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT, help="仓库根（调试用）")
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    try:
        declaration = load_yaml(repo_root / DECLARATION)
        schema = json.loads((repo_root / SCHEMA).read_text(encoding="utf-8"))
    except (CheckError, json.JSONDecodeError) as exc:
        print("[install-profile-check][错误] %s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    schema_errors = validate_schema(declaration, schema)
    report = {"declaration": str(DECLARATION), "schema_errors": schema_errors,
              "checks": [], "notes": [], "resolved": None}
    if schema_errors:
        report["passed"] = False
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2))
        else:
            print("[install-profile-check] schema 校验失败（%d 项）：" % len(schema_errors))
            for item in schema_errors:
                print("  - %s" % item)
        return EXIT_DECLARATION

    try:
        checks, notes = check_semantics(declaration, repo_root)
    except CheckError as exc:
        # 语义检查自身发现"事实不成立"（如版本单一来源失效、交叉校验文件缺失）：
        # 必须给出可操作的中文原因与退出码 2，不得抛裸异常（否则 CI 只看到 traceback）。
        print("[install-profile-check][错误] %s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    report["checks"] = checks
    report["notes"] = notes
    failed = [item for item in checks if not item["passed"]]
    report["passed"] = not failed

    if args.profile:
        wanted = [item for item in declaration["profiles"] if str(item["id"]) == args.profile]
        if not wanted:
            print("[install-profile-check][错误] 未知角色 id：%s（可选：%s）"
                  % (args.profile, ", ".join(str(i["id"]) for i in declaration["profiles"])),
                  file=sys.stderr)
            return EXIT_USAGE
        report["resolved"] = {"id": args.profile,
                              "effective": resolve(wanted[0], declaration["defaults"])}

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        print("[install-profile-check] 声明：%s" % DECLARATION)
        for item in checks:
            print("  %s %s —— %s" % ("✓" if item["passed"] else "✗", item["name"], item["detail"]))
        for item in notes:
            print("  ! %s" % item)
        if report["resolved"]:
            print("\n[生效值] 角色 %s（默认值已合并；改声明后请以此处为准）" % args.profile)
            for key, value in report["resolved"]["effective"].items():
                print("  %-22s %s" % (key, value))
        print("\n结果：%s（%d 项检查，%d 项未通过）"
              % ("通过" if report["passed"] else "未通过", len(checks), len(failed)))
    return EXIT_OK if report["passed"] else EXIT_DECLARATION


if __name__ == "__main__":
    raise SystemExit(main())
