"""组装 UR5e + Robotiq 2F-85 抓取场景（机器人无关的组装思路）。

设计要点（全部来自实测踩坑，不是设计偏好）：

1. **来源**：两个源模型都来自 MuJoCo Menagerie（官方开源），本脚本只做组合。
2. **挂载**：夹爪挂在 UR5e 的 `attachment_site`（pos="0 0.1 0"
   quat="-1 1 0 0"），沿用官方给出的法兰坐标系，不手调欧拉角。
3. **类名去冲突**：2F-85 自带 `default class="2f85"`，UR5e 已有自己的
   default 树；直接拼接会报 "repeated default class name"，因此把夹爪的
   类统一改名加前缀，并**同时改写 class= 与 childclass=**。
4. **元素名去冲突**：`base` / `visual` / `collision` 这类通用名两侧都在用，
   MuJoCo 要求同类元素名全局唯一，因此给夹爪具名元素加前缀并同步改写引用。

**两个必须显式处理、否则会静默出错的坑（实测代价很大）：**

坑 A：**网格前缀必须改写，不能只改元素名。**
  UR5e 与 2F-85 **都有** `base.stl`。只给 body/joint 改名而不改
  `<mesh file="base.stl">`，夹爪会去加载 UR5e 的 `base.stl`，
  于是夹爪基座变成了机械臂基座（实测网格顶点数 10899 → 6532）。
  修法：把夹爪的 `<mesh>` 资产 file 也加前缀，并把磁盘文件按前缀复制，
  让两个 `base.stl` 各自独立。

坑 B：**mesh 单位缩放必须在每个引用点显式声明，不能只靠 default 类继承。**
  官方 2F-85 把 `scale="0.001"` 写在 `<default class="2f85"><mesh .../></default>`
  里。该 default 子树搬进 UR5e 的 default 树后，会被 UR5e 自己的
  `class="ur5e"` 顶层 mesh 默认值（无 scale ≡ 1.0）覆盖，导致网格保持
  STL 原始毫米单位：实测 `base_mount` 网格 span 从 0.075m 变成 75m，
  质量从 0.150kg 变成 1.5e8kg（∝ 体积 ∝ span³）。
  质量爆炸会让夹爪的 tendon 完全驱不动（equality 被 1e8 倍惯量锁死、
  driver 被拖到限位外、pad 间隙几乎不变）。
  修法：在每个 `<mesh>` 资产上**显式写 scale**，并在编译后断言网格
  span 与质量落在官方量级，不通过就报错，绝不静默放行。

产物：
  <output_dir>/ur5e_2f85.xml                  组合后的 MJCF
  <output_dir>/ur5e_2f85-source-lock.json     来源与哈希（可审计）
"""

import argparse
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

MENAGERIE = Path("/home/coretek/MuJoCoBin/mujoco_menagerie")
UR5E_DIR = MENAGERIE / "universal_robots_ur5e"
GRIPPER_DIR = MENAGERIE / "robotiq_2f85"

#: 夹爪 default 类统一改名，避免与机械臂（或未来其它组件）的类名冲突。
CLASS_PREFIX = "rq2f85_"
OLD_ROOT_CLASS = "2f85"

#: 官方 tendon 执行器名（改名前）。
GRIPPER_ACTUATOR_NAME = "fingers_actuator"
#: 官方 pinch site 名（改名前）。
PINCH_SITE_NAME = "pinch"

#: 官方 2F-85 执行器参数（原样保留）。
GRIPPER_CTRLRANGE = (0.0, 255.0)
GRIPPER_GAINPRM = "0.3137255 0 0"
GRIPPER_BIASPRM = "0 -100 -10"
#: 官方 forcerange。实测在该力限下官方模型也能完成 84.8mm 全行程夹合，
#: 因此默认沿用官方值；需要提高时必须走 --forcerange 并记录证据。
OFFICIAL_FORCERANGE = 5.0

#: 夹爪网格缩放（毫米 → 米）。必须在每个 mesh 引用点显式声明（坑 B）。
GRIPPER_MESH_SCALE = 0.001

#: 臂关节速率阻尼；官方为 0，会导致无 ctrl 时 QACC 发散。
DEFAULT_ARM_DAMPING = 2.0

#: 质量合理性区间：官方 2F-85 单个体质量都在 0.001~0.8kg 量级。
#: 上界给到 5kg 是为了留余量，同时能拦住 1e6 级别的单位错误。
GRIPPER_BODY_MASS_MAX_KG = 5.0
#: 夹爪网格 span 的合理上界（米）。官方最大 span 约 0.095m。
GRIPPER_MESH_SPAN_MAX_M = 0.2


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path):
    root = ET.parse(str(path)).getroot()
    for element in root.iter():
        if isinstance(element.tag, str) and "}" in element.tag:
            element.tag = element.tag.split("}", 1)[1]
    return root


def _rename_classes(root, old, new):
    """把 default 定义的类名与所有 class= 引用统一重命名。

    必须同时改 class= 与 childclass=：MuJoCo 里 body/geom 用 childclass
    指定"子元素继承的默认类"，只改 class 会留下悬空引用
    （实测报 "unknown default childclass"）。
    """
    mapping = {}
    for element in root.iter("default"):
        name = element.get("class")
        if not name:
            continue
        mapping[name] = new if name == old else CLASS_PREFIX + name
    if not mapping:
        raise ValueError("未找到任何 default 类，拒绝组装")
    if new not in mapping.values():
        raise ValueError("根类未被正确改名: " + old)
    for element in root.iter():
        for attribute in ("class", "childclass"):
            name = element.get(attribute)
            if name and name in mapping:
                element.set(attribute, mapping[name])
    return mapping


def _prefix_element_names(subtree, prefix, tags):
    """给子树内所有具名元素加前缀，返回旧名→新名映射。"""
    renamed = {}
    for element in subtree.iter():
        if element.tag not in tags:
            continue
        name = element.get("name")
        if name:
            renamed[name] = prefix + name
            element.set("name", prefix + name)
    return renamed


def _prefix_mesh_assets(gripper_asset, prefix, assets_dir, source_assets_dir):
    """给夹爪网格资产加前缀：改 file 名 + 改 mesh name + 按前缀复制磁盘文件。

    **这是坑 A 的修法。** UR5e 与 2F-85 都有 `base.stl`：
    只改 XML 里 mesh 元素的 name（甚至不改）而共用同名 file 时，
    两个模型会指向同一份网格，夹爪基座会变成机械臂基座。

    必须同时改 **file 与 name** 并复制磁盘文件：
    - `file` 决定加载哪份 STL，两份同名文件必须各存一份；
    - `name` 是 `<geom mesh="...">` 的解引用目标；只改 file 而不改 name，
      会出现"UR5e 的 geom 引用 base_mount，但夹爪的网格也叫 base_mount"，
      编译报 "mesh 'base_mount' not found in geom 29"（实测）。

    返回 {原文件名: 新文件名} 映射（供审计）。
    """
    mapping = {}
    if gripper_asset is None:
        raise ValueError("夹爪模型缺少 asset 段")
    for mesh in gripper_asset.findall("mesh"):
        filename = mesh.get("file")
        if not filename:
            continue
        new_name = prefix + filename
        mapping[filename] = new_name
        mesh.set("file", new_name)
        # name 缺省时等于 file（MuJoCo 语义）；两种写法都要覆盖，
        # 否则 geom 的 mesh= 引用会指向夹爪侧的旧名字。
        mesh.set("name", new_name)
        source = source_assets_dir / filename
        if not source.is_file():
            raise FileNotFoundError("夹爪网格缺失: " + str(source))
        shutil.copyfile(source, assets_dir / new_name)
    return mapping


def _rewrite_mesh_references(gripper_root, mesh_file_map, prefix):
    """把夹爪 geom 的 `mesh=` 引用改写到加前缀后的网格名。

    `_prefix_mesh_assets` 已经把网格资产改名成 `<prefix><file>`，
    但 geom 上的 `mesh=` 不会自动跟着变（它不是 `name` 属性，
    不在 `_prefix_element_names` 的覆盖范围内）。漏掉这一步会在编译期报
    "mesh 'base_mount' not found in geom 29"（实测）。

    兼容两种写法：官方 mesh 元素只写 `file`（name 缺省等于 file），
    所以 `mesh=` 里既可能是裸文件名 `base_mount`，也可能是带扩展名的
    `base_mount.stl`；两种都映射到同一个新名字。
    """
    resolved = {}
    for filename, new_name in mesh_file_map.items():
        resolved[filename] = new_name
        resolved[Path(filename).stem] = new_name
    rewritten = {}
    for geom in gripper_root.iter("geom"):
        reference = geom.get("mesh")
        if not reference:
            continue
        new_reference = resolved.get(reference)
        if new_reference is None:
            new_reference = prefix + reference
        geom.set("mesh", new_reference)
        rewritten[reference] = new_reference
    return rewritten


def _force_mesh_scale(gripper_asset):
    """在每个夹爪 mesh 资产上显式写 scale（坑 B 的修法）。

    不能依赖 default 类继承：`class="2f85"` 的 `<mesh scale=.../>` 一旦
    搬进 UR5e 的 default 树就会被 UR5e 的 mesh 默认值覆盖，缩放失效。
    显式写在元素属性上优先级最高，不受类继承影响。
    """
    count = 0
    if gripper_asset is None:
        raise ValueError("夹爪模型缺少 asset 段")
    for mesh in gripper_asset.findall("mesh"):
        mesh.set("scale", "%.9f %.9f %.9f" % (
            GRIPPER_MESH_SCALE, GRIPPER_MESH_SCALE, GRIPPER_MESH_SCALE
        ))
        count += 1
    if count == 0:
        raise ValueError("夹爪 asset 段没有任何 mesh，拒绝组装")
    return count


def _add_pinch_alias(gripper_root, renamed_prefix):
    """在加前缀的 pinch site 之外，补一个原名别名 site，二者并存。

    site 名全局唯一，原名可能与机械臂侧冲突（这正是加前缀的原因），
    因此在夹爪自己的 body 上同时挂两个几何一致的 site：一个带前缀
    （内部引用用），一个原名（对外标准名）。site 不参与碰撞。
    """
    pinched = [
        element for element in gripper_root.iter("site")
        if element.get("name") == renamed_prefix + PINCH_SITE_NAME
    ]
    if len(pinched) != 1:
        raise ValueError("未找到唯一的夹爪 pinch site（加前缀后）: %d" % len(pinched))
    source = pinched[0]
    parent = None
    for candidate in gripper_root.iter("body"):
        if source in list(candidate):
            parent = candidate
            break
    if parent is None:
        raise ValueError("pinch site 的父 body 未找到")
    if any(
        element.get("name") == PINCH_SITE_NAME
        for element in gripper_root.iter("site")
    ):
        raise ValueError("已存在原名 pinch site，拒绝重复添加")
    alias = ET.Element("site")
    for key, value in source.attrib.items():
        if key == "name":
            continue
        alias.set(key, value)
    alias.set("name", PINCH_SITE_NAME)
    parent.append(alias)
    return {
        "alias": PINCH_SITE_NAME,
        "kept": renamed_prefix + PINCH_SITE_NAME,
        "reason": "保留官方标准名，供外部工具按名引用",
    }


def _apply_arm_damping(arm_root, arm_joint_names, damping):
    """给臂关节补速率阻尼，消除无 ctrl 时的 QACC 发散。

    官方 UR5e 臂关节 damping=0，纯靠执行器 PD 维持。一旦某次 mj_step
    落在"没有写入 ctrl"的窗口（后端刚构造、两次技能调用之间），
    就会出现 Nan/Inf QACC。加一点被动阻尼后模型任何时刻都是耗散的。
    """
    if damping < 0:
        raise ValueError("arm damping 不能为负")
    joint_elements = {
        element.get("name"): element
        for element in arm_root.iter("joint")
        if element.get("name")
    }
    applied = {}
    for name in arm_joint_names:
        element = joint_elements.get(name)
        if element is None:
            raise ValueError("机械臂缺少关节: " + name)
        element.set("damping", "%.6f" % damping)
        applied[name] = damping
    return applied


def _verify_gripper_physics(model, prefix, verbose=True):
    """编译后断言夹爪的网格尺度与质量落在官方量级，不通过即失败。

    **这是防止坑 B 静默复发的关键门禁。**
    历史上正是缺了这一步，才让"网格保持毫米单位 → 质量 1.5e8kg →
    夹爪完全驱不动"的错误一路走到抓取验收才暴露。
    """
    problems = []
    mesh_spans = {}
    for index in range(model.nmesh):
        name = mujoco_name(model, "mesh", index)
        if not name or not name.startswith(prefix):
            continue
        vertnum = int(model.mesh_vertnum[index])
        start = int(model.mesh_vertadr[index])
        if vertnum <= 0:
            continue
        verts = model.mesh_vert[start:start + vertnum]
        span = float((verts.max(axis=0) - verts.min(axis=0)).max())
        mesh_spans[name] = round(span, 6)
        if span > GRIPPER_MESH_SPAN_MAX_M:
            problems.append(
                "网格 %s 尺度异常: max span=%.6fm > %.3fm（疑似 mesh scale 未生效，"
                "网格仍是毫米单位）" % (name, span, GRIPPER_MESH_SPAN_MAX_M)
            )
    if not mesh_spans:
        problems.append("未找到任何带前缀的夹爪网格，前缀改写可能未生效")

    body_masses = {}
    for index in range(model.nbody):
        name = mujoco_name(model, "body", index)
        if not name or not name.startswith(prefix):
            continue
        mass = float(model.body_mass[index])
        body_masses[name] = round(mass, 9)
        if mass > GRIPPER_BODY_MASS_MAX_KG:
            problems.append(
                "夹爪 body %s 质量异常: %.6fkg > %.3fkg（疑似网格单位错误导致"
                "体积放大）" % (name, mass, GRIPPER_BODY_MASS_MAX_KG)
            )
    if not body_masses:
        problems.append("未找到任何带前缀的夹爪 body，前缀改写可能未生效")

    if problems:
        raise ValueError(
            "夹爪物理参数自检未通过（拒绝产出不可信的模型）:\n  - "
            + "\n  - ".join(problems)
        )
    if verbose:
        print("gripper physics self-check OK: meshes=%d bodies=%d" % (
            len(mesh_spans), len(body_masses)
        ))
        print("  mesh max span: %.6fm (limit %.3fm)" % (
            max(mesh_spans.values()), GRIPPER_MESH_SPAN_MAX_M
        ))
        print("  body max mass: %.6fkg (limit %.3fkg)" % (
            max(body_masses.values()), GRIPPER_BODY_MASS_MAX_KG
        ))
    return {"mesh_spans_m": mesh_spans, "body_masses_kg": body_masses}


def mujoco_name(model, kind, index):
    import mujoco

    obj = {
        "mesh": mujoco.mjtObj.mjOBJ_MESH,
        "body": mujoco.mjtObj.mjOBJ_BODY,
        "geom": mujoco.mjtObj.mjOBJ_GEOM,
    }[kind]
    return mujoco.mj_id2name(model, obj, index)


def build(
    output_dir,
    arm_pos=None,
    mount_quat=None,
    forcerange=OFFICIAL_FORCERANGE,
    arm_damping=DEFAULT_ARM_DAMPING,
    verbose=True,
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    assets_dir = output_dir / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)

    for source in (UR5E_DIR, GRIPPER_DIR):
        if not source.is_dir():
            raise FileNotFoundError("缺少 Menagerie 源模型: " + str(source))

    arm = _load(UR5E_DIR / "ur5e.xml")
    gripper = _load(GRIPPER_DIR / "2f85.xml")

    # --- 1) 类名去冲突 ---
    class_mapping = _rename_classes(
        gripper, OLD_ROOT_CLASS, CLASS_PREFIX + OLD_ROOT_CLASS
    )

    # --- 2) 元素名去冲突 ---
    name_map = _prefix_element_names(
        gripper,
        CLASS_PREFIX,
        {
            "body", "joint", "geom", "site", "tendon", "fixed", "spatial",
            "actuator", "general", "material",
        },
    )
    if GRIPPER_ACTUATOR_NAME not in name_map:
        raise ValueError("夹爪执行器未被改名，去冲突逻辑可能已失效")
    reference_attrs = (
        "site", "body1", "body2", "tendon", "joint", "joint1", "joint2", "material",
    )
    for element in gripper.iter():
        for attribute in reference_attrs:
            value = element.get(attribute)
            if value and value in name_map:
                element.set(attribute, name_map[value])

    # --- 3) 网格资产加前缀 + 按前缀复制磁盘文件（坑 A）---
    gripper_asset = gripper.find("asset")
    mesh_file_map = _prefix_mesh_assets(
        gripper_asset, CLASS_PREFIX, assets_dir, GRIPPER_DIR / "assets"
    )

    # --- 3b) geom 的 mesh= 引用同步改写 ---
    mesh_ref_map = _rewrite_mesh_references(gripper, mesh_file_map, CLASS_PREFIX)

    # --- 4) 网格缩放显式声明（坑 B）---
    scaled_meshes = _force_mesh_scale(gripper_asset)

    # --- 5) 保留 pinch 的官方标准名 ---
    pinch_alias_report = _add_pinch_alias(gripper, CLASS_PREFIX)

    # --- 6) 夹爪 tendon 执行器参数（默认沿用官方值）---
    actuator_element = None
    for element in gripper.iter():
        if element.tag in ("actuator", "general", "position", "motor"):
            if element.get("name") == CLASS_PREFIX + GRIPPER_ACTUATOR_NAME:
                actuator_element = element
                break
    if actuator_element is None:
        raise ValueError("夹爪缺少 tendon 执行器，拒绝组装")
    official_force = [
        float(v) for v in (actuator_element.get("forcerange") or "").split()
    ]
    if len(official_force) != 2:
        raise ValueError(
            "夹爪执行器缺少可解析的 forcerange: "
            + str(actuator_element.get("forcerange"))
        )
    actuator_element.set(
        "forcerange", "%.6f %.6f" % (-forcerange, forcerange)
    )
    for key, value in (
        ("ctrlrange", " ".join("%.1f" % v for v in GRIPPER_CTRLRANGE)),
        ("gainprm", GRIPPER_GAINPRM),
        ("biasprm", GRIPPER_BIASPRM),
    ):
        actuator_element.set(key, value)
    force_report = {
        "forcerange": [-forcerange, forcerange],
        "official_forcerange": official_force,
        "changed": abs(forcerange - abs(official_force[0])) > 1e-9,
    }

    # --- 7) 臂关节补阻尼 ---
    arm_joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    damping_report = _apply_arm_damping(arm, arm_joint_names, arm_damping)

    # --- 8) 机械臂网格与材质搬进统一 assets/ ---
    asset_root = arm.find("asset")
    if asset_root is None:
        asset_root = ET.SubElement(arm, "asset")
    existing_meshes = {
        mesh.get("file") for mesh in asset_root.findall("mesh") if mesh.get("file")
    }
    copied = set()
    for source in sorted((UR5E_DIR / "assets").glob("*")):
        if not source.is_file():
            continue
        shutil.copyfile(source, assets_dir / source.name)
        copied.add(source.name)
    for filename in sorted(copied):
        if filename not in existing_meshes:
            ET.SubElement(asset_root, "mesh", file=filename)
            existing_meshes.add(filename)

    # 夹爪网格资产搬进机械臂的 asset 段（file/scale 已在步骤 3/4 处理）。
    for mesh in list(gripper_asset.findall("mesh")):
        new_mesh = ET.SubElement(asset_root, "mesh")
        for key, value in mesh.attrib.items():
            new_mesh.set(key, value)
        existing_meshes.add(new_mesh.get("file"))

    # 材质：夹爪 geom 引用 material="gray" 等，只搬网格会报
    # "material 'gray' not found"。名字已由 name_map 统一加前缀。
    existing_materials = {
        material.get("name") for material in asset_root.findall("material")
    }
    for material in gripper_asset.findall("material"):
        name = material.get("name")
        if not name or name in existing_materials:
            continue
        new_material = ET.SubElement(asset_root, "material", name=name)
        for key, value in material.attrib.items():
            if key != "name":
                new_material.set(key, value)
        existing_materials.add(name)

    compiler = arm.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(arm, "compiler")
    compiler.set("meshdir", "assets")
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")

    # --- 9) 搬运夹爪 default 子树 ---
    arm_default = arm.find("default")
    if arm_default is None:
        raise ValueError("机械臂模型缺少 default 段")
    gripper_default = gripper.find("default")
    if gripper_default is None:
        raise ValueError("夹爪模型缺少 default 段")
    root_class = CLASS_PREFIX + OLD_ROOT_CLASS
    moved = 0
    for child in list(gripper_default):
        if child.tag == "default" and child.get("class") == root_class:
            arm_default.append(child)
            moved += 1
    if moved != 1:
        raise ValueError("夹爪根 default 类数量异常: %d" % moved)
    seen = {}
    for element in arm.iter("default"):
        name = element.get("class")
        if name:
            seen[name] = seen.get(name, 0) + 1
    dupes = sorted(name for name, count in seen.items() if count > 1)
    if dupes:
        raise ValueError("拼接后仍存在重复 default 类: " + ", ".join(dupes))

    # --- 10) 搬运执行器 ---
    arm_actuator = arm.find("actuator")
    if arm_actuator is None:
        arm_actuator = ET.SubElement(arm, "actuator")
    gripper_actuator = gripper.find("actuator")
    if gripper_actuator is None:
        raise ValueError("夹爪模型缺少 actuator 段")
    actuator_names = []
    for actuator in list(gripper_actuator):
        actuator_names.append(actuator.get("name"))
        arm_actuator.append(actuator)

    # --- 11) 搬运 tendon / equality ---
    for section in ("tendon", "equality"):
        block = gripper.find(section)
        if block is None:
            continue
        target = arm.find(section)
        if target is None:
            target = ET.SubElement(arm, section)
        for child in list(block):
            target.append(child)
    # contact 段的 exclude 也要搬：装爪的 4 杆机构靠它避免自碰撞。
    contact_block = gripper.find("contact")
    if contact_block is not None:
        target = arm.find("contact")
        if target is None:
            target = ET.SubElement(arm, "contact")
        for child in list(contact_block):
            target.append(child)

    # --- 12) 挂载夹爪到法兰 ---
    worldbody = arm.find("worldbody")
    if worldbody is None:
        raise ValueError("机械臂模型缺少 worldbody")
    wrist3 = None
    for body in worldbody.iter("body"):
        if body.get("name") == "wrist_3_link":
            wrist3 = body
            break
    if wrist3 is None:
        raise ValueError("UR5e 缺少 wrist_3_link，无法挂载夹爪")
    flange = None
    for site in wrist3.findall("site"):
        if site.get("name") == "attachment_site":
            flange = site
            break
    if flange is None:
        raise ValueError("UR5e 缺少 attachment_site，无法确定法兰坐标系")

    gripper_worldbody = gripper.find("worldbody")
    if gripper_worldbody is None:
        raise ValueError("夹爪模型缺少 worldbody")
    gripper_body = gripper_worldbody.find("body")
    if gripper_body is None:
        raise ValueError("夹爪模型缺少根 body")
    gripper_body.set("pos", flange.get("pos", "0 0 0"))
    gripper_body.set("quat", mount_quat or flange.get("quat", "1 0 0 0"))
    wrist3.append(gripper_body)

    base = worldbody.find("body")
    if arm_pos is not None and base is not None:
        base.set("pos", " ".join("%.9f" % float(v) for v in arm_pos))

    # --- 13) 关键帧维度补齐 ---
    # UR5e 的 home 关键帧只覆盖 6 个臂关节，组合后 qpos 维度 = 6 + 夹爪自由度，
    # 直接保留会报 "invalid qpos size"。先把 keyframe 摘下来编译一次拿到
    # 真实 nq/nu，再按维度补零放回（不依赖对夹爪自由度的估算）。
    import mujoco as _mujoco

    output = output_dir / "ur5e_2f85.xml"
    keyframe = arm.find("keyframe")
    saved_keys = []
    if keyframe is not None:
        saved_keys = [copy.deepcopy(key) for key in list(keyframe)]
        arm.remove(keyframe)

    ET.indent(arm, space="    ")
    ET.ElementTree(arm).write(output, encoding="unicode", xml_declaration=True)
    probe = _mujoco.MjModel.from_xml_path(str(output.resolve()))
    nq, nu = int(probe.nq), int(probe.nu)

    if saved_keys:
        keyframe = ET.SubElement(arm, "keyframe")
        for key in saved_keys:
            qpos = (key.get("qpos") or "").split()
            ctrl = (key.get("ctrl") or "").split()
            if qpos:
                if len(qpos) > nq:
                    raise ValueError(
                        "关键帧 qpos 超出模型维度: %d > %d" % (len(qpos), nq)
                    )
                key.set("qpos", " ".join(qpos + ["0"] * (nq - len(qpos))))
            if ctrl:
                if len(ctrl) > nu:
                    raise ValueError(
                        "关键帧 ctrl 超出模型维度: %d > %d" % (len(ctrl), nu)
                    )
                key.set("ctrl", " ".join(ctrl + ["0"] * (nu - len(ctrl))))
            keyframe.append(key)
        ET.indent(arm, space="    ")
        ET.ElementTree(arm).write(output, encoding="unicode", xml_declaration=True)
        probe = _mujoco.MjModel.from_xml_path(str(output.resolve()))

    # --- 14) 物理自检门禁（防止坑 A/B 静默复发）---
    physics_check = _verify_gripper_physics(probe, CLASS_PREFIX, verbose=verbose)

    # --- 15) 网格引用自检：夹爪网格文件必须与机械臂的不重叠 ---
    gripper_mesh_files = {name for name in mesh_file_map.values()}
    arm_mesh_files = {mesh.get("file") for mesh in asset_root.findall("mesh")}
    overlap = sorted(gripper_mesh_files & copied)
    if overlap:
        raise ValueError(
            "夹爪与机械臂共用了同名网格文件（会被互相覆盖）: "
            + ", ".join(overlap)
        )

    lock = {
        "schema_version": "iraf.ur5e-2f85-source-lock/v1",
        "sources": [
            {
                "path": str(UR5E_DIR / "ur5e.xml"),
                "sha256": sha256(UR5E_DIR / "ur5e.xml"),
            },
            {
                "path": str(GRIPPER_DIR / "2f85.xml"),
                "sha256": sha256(GRIPPER_DIR / "2f85.xml"),
            },
        ],
        "arm_mesh_files": sorted(copied),
        "gripper_mesh_files": mesh_file_map,
        "gripper_mesh_references": mesh_ref_map,
        "gripper_mesh_count": len(mesh_file_map),
        "gripper_meshes_scaled": scaled_meshes,
        "gripper_mesh_scale": GRIPPER_MESH_SCALE,
        "class_renamed": class_mapping,
        "elements_renamed": len(name_map),
        "gripper_actuators": actuator_names,
        "flange_site": {"pos": flange.get("pos"), "quat": flange.get("quat")},
        "output": str(output),
        "physics_self_check": physics_check,
        "overrides": {
            "gripper_force_limit": force_report,
            "pinch_site_alias": pinch_alias_report,
            "arm_joint_damping": damping_report,
        },
    }
    (output_dir / "ur5e_2f85-source-lock.json").write_text(
        json.dumps(lock, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if verbose:
        print("WROTE", output, output.stat().st_size, "bytes")
        print("arm meshes:", len(copied), "gripper meshes:", len(mesh_file_map))
        print("gripper actuators:", actuator_names)
        print("overrides:", json.dumps(lock["overrides"], ensure_ascii=False))
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("build/models/ur5e_2f85")
    )
    parser.add_argument("--arm-pos", default=None, help="机械臂基座世界坐标 'x y z'")
    parser.add_argument(
        "--forcerange",
        type=float,
        default=OFFICIAL_FORCERANGE,
        help="夹爪 tendon 执行器力上限（官方 %.1fN）" % OFFICIAL_FORCERANGE,
    )
    parser.add_argument(
        "--arm-damping",
        type=float,
        default=DEFAULT_ARM_DAMPING,
        help="臂关节速率阻尼（官方为 0，无 ctrl 时会发散）",
    )
    args = parser.parse_args(argv)
    arm_pos = [float(v) for v in args.arm_pos.split()] if args.arm_pos else None
    output = build(
        args.output_dir,
        arm_pos=arm_pos,
        forcerange=args.forcerange,
        arm_damping=args.arm_damping,
    )

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(output.resolve()))
    print(
        "LOADED nbody=%d njnt=%d ngeom=%d nu=%d nq=%d"
        % (model.nbody, model.njnt, model.ngeom, model.nu, model.nq)
    )
    print("joints:", [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
    ])
    print("actuators:", [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    ])
    print("sites:", [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        for i in range(model.nsite)
    ])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
