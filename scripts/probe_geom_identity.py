"""列出场景中未命名 geom 的归属（排查接触体身份）。"""

import json
from pathlib import Path

import mujoco

root = Path(__file__).resolve().parents[1]
model = mujoco.MjModel.from_xml_path(str(root / "build/models/ur5-pick-scene.xml"))

print("geom 总数: %d" % model.ngeom)
for gid in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    body_id = int(model.geom_bodyid[gid])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    if name:
        continue
    print(
        "  #%-3d body=%-28s type=%-8s size=%s contype=%d conaffinity=%d rgroups=%d"
        % (
            gid,
            body_name,
            int(model.geom_type[gid]),
            [round(float(v), 5) for v in model.geom_size[gid]],
            int(model.geom_contype[gid]),
            int(model.geom_conaffinity[gid]),
            int(model.geom_rgba[gid][3] > 0),
        )
    )
print()
for gid in (26, 27, 28):
    if gid < model.ngeom:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        body_id = int(model.geom_bodyid[gid])
        print(
            "geom#%d name=%s body=%s type=%s size=%s"
            % (
                gid,
                name,
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
                int(model.geom_type[gid]),
                [round(float(v), 5) for v in model.geom_size[gid]],
            )
        )
print()
print("关键 geom id:")
for target in (
    "box_01_geom",
    "workbench",
    "rq2f85_left_pad1",
    "rq2f85_left_pad2",
    "rq2f85_right_pad1",
    "rq2f85_right_pad2",
):
    print("  %-24s %d" % (target, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, target)))
