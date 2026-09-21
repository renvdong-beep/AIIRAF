#!/usr/bin/env python3
"""wheel 标签匹配与候选择优的**单一实现**（抓取侧与依赖解析侧共用）。

================================ 为什么要有这个模块 ================================
同一个问题曾经有两套实现：
  * `deploy/sdk/check_wheel_tags.py`（抓取侧）自带正则解析 wheel 文件名 + 自写
    `python_tag_compatible` / `platform_tag_compatible`；
  * `deploy/sdk/lib_wheelhouse_deps.py`（依赖解析侧）用 `packaging`。
两者对"压缩标签"的处理不同，实测分歧：`glfw-2.10.2-py2.py3-none-manylinux_2_28_aarch64.whl`
在 `py2.py3` 下等价于 `py3-none-<plat>`，pip 与 `packaging` 都认为 cp310 可装，而抓取侧的自写
规则要求"非纯 Python wheel 必须显式列出 cp310/py310" ⇒ 把它拒掉，实际抓到的是更旧的
`glfw-2.10.0-py2.py27…py314-none-…whl`。结果是"人工确认的候选清单"与"实际抓取结果"不是同一套数据。

本模块统一由 `packaging` 决定"某个 wheel 能否装到声明的目标上"，并提供统一的择优先后规则；
两处调用方都不再自己写标签判断（AGENTS.md 2.11 / 5.3：不允许多处相同实现）。

约定（与产品契约一致，只严不松）：
  1. 声明的 `reject_platform_tags` 仍然**优先**生效（它是声明层的额外禁令，例如 win_*/macosx）；
  2. 是否可装由 `packaging.tags` 的可接受标签集判定（与 pip 同源）；
  3. 平台标签兼容阶梯：声明 `manylinux_2_28_aarch64` 时接受更低 glibc 的 manylinux 标签
     （2_27 … 2_17、manylinux2014）—— 这在 glibc ≥ 2.28 的宿主上是真的可安装；
  4. 择优：先按版本（PEP 440），正式版优先于预发布；同版本内 精确平台 > 兼容 manylinux > 纯 Python；
     声明了 `pins`（`==版本`）时先按 pin 过滤。
"""

from __future__ import annotations

import dataclasses
import re

try:  # packaging 是硬依赖：标签语义必须与 pip 同源，不能手写正则近似
    from packaging.tags import Tag, compatible_tags, cpython_tags
    from packaging.utils import InvalidWheelFilename, parse_wheel_filename as _pkg_parse
    from packaging.version import InvalidVersion, Version
except ImportError as exc:  # pragma: no cover
    raise SystemExit(f"[预检失败] 缺少 packaging 模块：{exc}；请安装 python3-packaging 后重试")


class WheelSpecError(Exception):
    """显式失败：调用方必须转换为非零退出码，不得吞掉。"""


# kind 的偏好次序（数字越小越优先）
KIND_RANK = {"exact": 0, "compatible": 1, "pure_python": 2}


def canonical_name(name: str) -> str:
    """PEP 503 归一化名称（索引目录名与 pins 查表都用它）。"""
    return re.sub(r"[-_.]+", "-", str(name)).lower()


def python_version_tuple(python_tag: str) -> tuple[int, int]:
    """`cp310` → (3, 10)；不能推导即显式失败（不猜）。"""
    match = re.fullmatch(r"cp(\d)(\d+)", str(python_tag or ""))
    if not match:
        raise WheelSpecError(
            f"python_tag={python_tag!r} 无法推导解释器版本（只支持 cpXY，例如 cp310）"
        )
    return int(match.group(1)), int(match.group(2))


def arch_of_platform_tag(platform_tag: str) -> str:
    """从平台标签里取出架构后缀（`manylinux_2_28_aarch64` → `aarch64`）；取不到就原样返回。"""
    match = re.fullmatch(r"manylinux(?:_\d+_\d+)?_([a-z0-9_]+)", str(platform_tag or ""))
    if match:
        return match.group(1)
    match = re.fullmatch(r"(?:musllinux_\d+_\d+|linux)_([a-z0-9_]+)", str(platform_tag or ""))
    if match:
        return match.group(1)
    return str(platform_tag or "")


def compatible_platform_tags(platform_tag: str) -> list[str]:
    """声明平台的兼容阶梯（精确标签在最前）。

    在 glibc ≥ 2.28 的系统上，pip 同样接受用更低 glibc 编译的 manylinux wheel
    （2_27 / 2_17 / manylinux2014 …）。只放"精确等于声明标签"会漏掉真实存在的包
    ——实测：rpds-py 只发 manylinux_2_17_aarch64，按精确匹配会解析不出来。
    """
    match = re.fullmatch(r"manylinux_(\d+)_(\d+)_(\w+)", str(platform_tag or ""))
    if not match:
        return [platform_tag]
    major, minor, arch = int(match.group(1)), int(match.group(2)), match.group(3)
    ladder = []
    if major == 2:
        for candidate in range(minor, 16, -1):
            if candidate >= 17:
                ladder.append(f"manylinux_2_{candidate}_{arch}")
        if minor >= 17:
            ladder.append(f"manylinux2014_{arch}")   # manylinux_2_17 的别名
    return ladder or [platform_tag]


def acceptable_tags(python_tag: str, platform_tag: str) -> frozenset:
    """目标可接受标签集（与 pip 同源：cpython_tags ∪ compatible_tags）。"""
    version = python_version_tuple(python_tag)
    tags = set()
    for platform in compatible_platform_tags(platform_tag):
        tags |= set(cpython_tags(python_version=version, platforms=[platform]))
        tags |= set(compatible_tags(python_version=version, platforms=[platform]))
    if not tags:
        raise WheelSpecError(f"无法为 python_tag={python_tag} / platform_tag={platform_tag} 生成可接受标签集")
    return frozenset(tags)


@dataclasses.dataclass(frozen=True)
class WheelInfo:
    filename: str
    name: str            # 归一化名称
    version: str
    python_tags: tuple
    abi_tags: tuple
    platform_tags: tuple
    tags: frozenset


def parse_wheel(filename: str) -> WheelInfo:
    """解析 wheel 文件名（`packaging` 解析，压缩标签会被展开）。"""
    if not str(filename).endswith(".whl"):
        raise WheelSpecError(f"不是 wheel 文件：{filename}（离线 wheelhouse 只接受 .whl）")
    try:
        name, version, _build, tags = _pkg_parse(filename)
    except (InvalidWheelFilename, ValueError) as exc:
        raise WheelSpecError(f"wheel 文件名无法解析：{filename}（{exc}）") from exc
    return WheelInfo(
        filename=filename,
        name=canonical_name(str(name)),
        version=str(version),
        python_tags=tuple(sorted({t.interpreter for t in tags})),
        abi_tags=tuple(sorted({t.abi for t in tags})),
        platform_tags=tuple(sorted({t.platform for t in tags})),
        tags=frozenset(tags),
    )


def classify(filename: str, python_tag: str, platform_tag: str,
             reject_platform_tags: tuple = ()) -> dict:
    """判定单个 wheel 能否用于声明目标；返回可审计的 dict（供 CLI/单测使用）。

    决策顺序：① 声明拒绝标签 → 拒绝；② packaging 标签集求交 → 接受并给出 kind；③ 否则拒绝。
    kind：exact（与声明平台标签精确一致）/ compatible（兼容阶梯上的更低 glibc/别名）/
          pure_python（platform=any 且 abi=none）。
    """
    try:
        info = parse_wheel(filename)
    except WheelSpecError as exc:
        return {"filename": filename, "accepted": False, "kind": "", "reason": str(exc),
                "name": "", "version": "", "python_tags": [], "abi_tags": [], "platform_tags": []}

    result = {
        "filename": filename,
        "name": info.name,
        "version": info.version,
        "python_tags": list(info.python_tags),
        "abi_tags": list(info.abi_tags),
        "platform_tags": list(info.platform_tags),
        "accepted": False,
        "kind": "",
        "reason": "",
    }

    joined = " ".join(info.platform_tags)
    for rejected in reject_platform_tags or ():
        if not rejected:
            continue
        if any(rejected == tag or rejected in tag for tag in info.platform_tags):
            result["reason"] = f"命中声明拒绝的平台标签 {rejected}（文件平台标签：{joined}）"
            return result

    accepted = acceptable_tags(python_tag, platform_tag)
    if not (info.tags & accepted):
        # 诊断要能定位到"解释器/ABI"还是"平台"，否则使用者只能看到笼统的"不匹配"。
        # 注意按 (interpreter, abi) 组合判定：`cp39` 只以 abi3 形式出现在 cp310 的可接受集里，
        # 因此 `cp39-cp39-<plat>` 属于解释器/ABI 不匹配，而不是平台问题。
        ok_pairs = {(t.interpreter, t.abi) for t in accepted}
        ok_platforms = {t.platform for t in accepted}
        file_pairs = {(i, a) for i in info.python_tags for a in info.abi_tags}
        if not (file_pairs & ok_pairs):
            result["reason"] = (
                f"解释器标签不匹配：声明 {python_tag}（或 py3 纯 Python），"
                f"文件为 {'/'.join(info.python_tags)}-{'/'.join(info.abi_tags)}"
            )
        elif not (set(info.platform_tags) & ok_platforms):
            result["reason"] = (
                f"平台标签与声明不符：声明 {platform_tag}（架构 {arch_of_platform_tag(platform_tag)}），"
                f"文件为 {joined}"
            )
        else:
            result["reason"] = (
                f"标签组合与声明目标不匹配：声明 {python_tag}/{platform_tag}，"
                f"文件为 {'/'.join(info.python_tags)}-{'/'.join(info.abi_tags)}-{joined}"
            )
        return result

    ladder = compatible_platform_tags(platform_tag)
    if platform_tag in info.platform_tags:
        result.update(accepted=True, kind="exact",
                      reason=f"平台标签与声明一致（{platform_tag}）")
    elif info.platform_tags == ("any",) and "none" in info.abi_tags:
        result.update(accepted=True, kind="pure_python",
                      reason="纯 Python wheel（platform=any、abi=none），与架构无关")
    else:
        hit = next((tag for tag in info.platform_tags if tag in ladder), "")
        result.update(accepted=True, kind="compatible",
                      reason=f"{hit or joined} 属声明 {platform_tag} 的兼容阶梯（更低 glibc 要求），同架构可安装")
    return result


def version_key(version: str) -> Version:
    """PEP 440 排序键（不手写分段比较：实测手写会把 4.0.0a5 排到 3.1.10 之前）。"""
    try:
        return Version(version)
    except InvalidVersion as exc:
        raise WheelSpecError(f"版本号无法按 PEP 440 解析：{version!r}（{exc}）") from exc


def is_prerelease(version: str) -> bool:
    return version_key(version).is_prerelease
