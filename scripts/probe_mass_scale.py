"""定位夹爪质量被放大 1e9 倍的原因：mesh scale 被重复应用。

实测（probe_gripper_diff.py）：
  base_mount    官方 0.150003 kg  →  组装 150002855.16 kg
  silicone_pad  官方 0.001303 kg  →  组装 1302811.84 kg
两处都精确放大约 1e9 倍。

1e9 = (1e3)^3，而官方 2f85.xml 的根 default 类里有：

    <default class="2f85">
      <mesh scale="0.001 0.001 0.001"/>
      ...
    </default>

mesh scale=0.001 让 STL 的毫米单位变成米。质量 ∝ 体积 ∝ scale^3，
所以"scale 被应用了两次"（等效 scale=1e-6）会让质量放大 (1e6/1e-3)^... 
—— 更直接地说，只要**质量被按未缩放体积计算**，就会差 1e9 倍。

本脚本检查：
1. 官方模型里 base_mount 的 body_mass 与 geom 尺寸，和 STL 原始单位的关系；
2. 组装模型的对应值；
3. 组装 XML 里 mesh/geom 元素上是否残留/重复了 scale；
4. 把所有带 scale 的元素打印出来，定位重复点。
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def dump_xml_scales(path, label):
    tree = ET.parse(str(path))
    root = tree.getroot()
    print("\n=== %s 的 scale 声明 ===" % label)
    rows = []
    for element in root.iter():
        if not isinstance(element.tag, str):
            continue
        scale = element.get("scale")
        if scale is None:
            continue
        rows.append(
            {
                "tag": element.tag,
                "name": element.get("name"),
                "class": element.get("class"),
                "file": element.get("file"),
                "scale": scale,
            }
        )
        print("  <%s name=%s class=%s file=%s scale=%s>"
              % (element.tag, element.get("name"), element.get("class"),
                 element.get("file"), scale))
    # default 段里的 mesh scale（不带 name 的那种）
    for default in root.iter("default"):
        for mesh in default.findall("mesh"):
            print("  <default class=%s><mesh scale=%s/>"
                  % (default.get("class"), mesh.get("scale")))
            rows.append(
                {
                    "tag": "default/mesh",
                    "class": default.get("class"),
                    "scale": mesh.get("scale"),
                }
            )
    return rows


def dump_masses(path, label):
    model = mujoco.MjModel.from_xml_path(str(path))
    print("\n=== %s 的 body 质量（夹爪部分）===" % label)
    rows = {}
    for index in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
        if not any(key in name for key in ("base", "driver", "coupler", "follower",
                                           "spring_link", "pad")):
            continue
        rows[name] = {
            "mass": round(float(model.body_mass[index]), 9),
            "inertia": [round(float(v), 12) for v in model.body_inertia[index]],
            "ipos": [round(float(v), 6) for v in model.body_ipos[index]],
        }
        print("  %-34s mass=%.9f inertia=%s"
              % (name, float(model.body_mass[index]),
                 [round(float(v), 12) for v in model.body_inertia[index]]))
    return rows


def dump_mesh_scale(path, label):
    """读 mesh 的编译结果：mesh_vert 的实际尺度决定质量。"""
    model = mujoco.MjModel.from_xml_path(str(path))
    print("\n=== %s 的 mesh 实际尺寸（编译后顶点包围盒）===" % label)
    rows = {}
    for index in range(model.nmesh):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, index) or ""
        if not any(key in name for key in ("base", "driver", "coupler", "follower",
                                           "spring_link", "pad")):
            continue
        count = int(model.mesh_vertnum[index])
        start = int(model.mesh_vertadr[index])
        if count <= 0:
            continue
        verts = model.mesh_vert[start:start + count]
        span = (verts.max(axis=0) - verts.min(axis=0))
        rows[name] = [round(float(v), 6) for v in span]
        print("  %-24s verts=%-6d span=%s"
              % (name, count, [round(float(v), 6) for v in span]))
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--official",
        default="/home/coretek/MuJoCoBin/mujoco_menagerie/robotiq_2f85/2f85.xml",
    )
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--output", type=Path, default=Path("build/calibration/ur5-mass-scale.json")
    )
    args = parser.parse_args()

    official = Path(args.official).resolve()
    assembly = Path(args.assembly).resolve()

    report = {
        "schema_version": "iraf.ur5-mass-scale/v1",
        "official_xml_scales": dump_xml_scales(official, "官方 2f85.xml"),
        "assembly_xml_scales": dump_xml_scales(assembly, "组装 ur5e_2f85.xml"),
        "official_masses": dump_masses(official, "官方"),
        "assembly_masses": dump_masses(assembly, "组装"),
        "official_meshes": dump_mesh_scale(official, "官方"),
        "assembly_meshes": dump_mesh_scale(assembly, "组装"),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
