#!/usr/bin/env python3
"""按**机型声明的自由相机**渲一帧离屏图，用来核对取景（是否把两台臂 + 狗都收进画面）。

为什么需要（2026-09-29）：使用者反馈"看不全其他机械臂"。取景是**声明 → 图像**的映射，
不能靠猜；这个脚本把声明的四个量（lookat/distance/azimuth/elevation）直接渲成一帧 PNG，
人（或 vision）看图核对。

用法：
  PYTHONPATH=src python3 scripts/probe_render_declared_camera.py \
      --declaration config/go2_joint.yaml --model build/scenes/handoff_lab/handoff_lab_joint.xml \
      --out build/diagnostics/declared-camera-frame.png
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--declaration", default="config/go2_joint.yaml")
    parser.add_argument("--model", default="build/scenes/handoff_lab/handoff_lab_joint.xml")
    parser.add_argument("--key", type=int, default=0)
    parser.add_argument("--out", default="build/diagnostics/declared-camera-frame.png")
    args = parser.parse_args()

    declaration = yaml.safe_load((REPO / args.declaration).read_text(encoding="utf-8"))
    render = declaration.get("render") or {}
    camera = render.get("camera")
    width, height = int(render.get("width_px")), int(render.get("height_px"))
    model = mujoco.MjModel.from_xml_path(str(REPO / args.model))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, args.key)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=height, width=width)
    if isinstance(camera, dict):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [float(v) for v in camera["lookat_m"]]
        cam.distance = float(camera["distance_m"])
        cam.azimuth = float(camera["azimuth_deg"])
        cam.elevation = float(camera["elevation_deg"])
        renderer.update_scene(data, camera=cam)
        spec = {"kind": "free", "camera": camera}
    else:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, str(camera))
        if cam_id < 0:
            print(json.dumps({"error": "声明的相机 %r 不在模型里" % camera}, ensure_ascii=False))
            return 2
        renderer.update_scene(data, camera=cam_id)
        spec = {"kind": "fixed", "camera": str(camera)}
    frame = np.asarray(renderer.render())
    out = REPO / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
    except ImportError:
        import imageio.v2 as imageio
        imageio.imwrite(str(out), frame)
    else:
        Image.fromarray(frame).save(out)

    # 客观量：非背景像素占比（证明"确实渲出了东西"）+ 相机世界位置（便于复算取景）。
    # 取景是否"看得见两台臂"由**看图**判定（本脚本把图落盘，供人/vision 核对），不用启发式猜。
    if isinstance(camera, dict):
        el = np.radians(float(camera["elevation_deg"]))
        az = np.radians(float(camera["azimuth_deg"]))
        lookat = np.asarray(camera["lookat_m"], dtype=float)
        cam_pos = lookat + float(camera["distance_m"]) * np.array(
            [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), -np.sin(el)])
    else:
        cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=float)
    interesting = {}
    for name in ("piper_link1", "ur5e_base", "base_link", "tray_01", "box_01"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            interesting[name] = [round(float(v), 4) for v in data.xpos[bid]]
    print(json.dumps({"camera_spec": spec, "camera_world_pos_m": [round(float(v), 4) for v in cam_pos],
                      "bodies_world_pos_m": interesting, "frame_png": str(out.relative_to(REPO)),
                      "nonblack_fraction": round(float((frame.reshape(-1, 3).max(axis=1) > 8).mean()), 4)},
                     ensure_ascii=False, indent=1))

    # ---- 客观取景核算：把关键 body 投到像面，判"在不在画面里、留边多少"
    # 为什么不用"看图"：本机 vision 通道不可用（403），而"看不看得见"是可算的：
    #   相机位置（自由相机约定，MuJoCo）：pos = lookat + d·(cos e·cos a, cos e·sin a, −sin e)
    #   相机基：forward = normalize(lookat − pos)、right = normalize(cross(forward, (0,0,1)))、
    #            up = cross(right, forward)
    #   投影像素：px = (0.5 + 0.5·x_c /(z_c·tan(fovy/2)·aspect))·W，py = (0.5 − 0.5·y_c /(z_c·tan(fovy/2)))·H
    # 自校验：用同一个算式投**场景自带固定相机**的已知目标（方块 box_01 是 overhead_camera 的注视点），
    #         若它落在像面中心附近 ⇒ 算式与约定一致，才能用同一算式判自由相机的取景。
    def _project(pos, lookat, fovy_deg, w, h, point):
        pos = np.asarray(pos, dtype=float)
        forward = np.asarray(lookat, dtype=float) - pos
        forward = forward / np.linalg.norm(forward)
        right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
        norm = np.linalg.norm(right)
        right = right / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])
        up = np.cross(right, forward)
        v = np.asarray(point, dtype=float) - pos
        z_c = float(np.dot(v, forward))
        if z_c <= 1e-6:
            return None
        half = np.tan(np.radians(fovy_deg) / 2.0)
        x_ndc = float(np.dot(v, right)) / (z_c * half * (w / h))
        y_ndc = float(np.dot(v, up)) / (z_c * half)
        return ((0.5 + 0.5 * x_ndc) * w, (0.5 - 0.5 * y_ndc) * h)

    fovy = float(model.vis.global_.fovy)
    points = {name: [float(v) for v in data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)]]
              for name in ("piper_link1", "ur5e_base", "base_link", "tray_01", "box_01")
              if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0}
    if isinstance(camera, dict):
        pix = {name: _project(cam_pos, camera["lookat_m"], fovy, width, height, point)
               for name, point in points.items()}
        inside = {name: (None if value is None else
                         bool(0 <= value[0] <= width and 0 <= value[1] <= height))
                  for name, value in pix.items()}
        print(json.dumps({
            "fovy_deg": fovy,
            "projected_px": {k: (None if v is None else [round(v[0], 1), round(v[1], 1)])
                             for k, v in pix.items()},
            "inside_frame": inside,
            "verdict_all_robots_visible": bool(all(v for v in inside.values() if v is not None)),
        }, ensure_ascii=False, indent=1))
    # 自校验：固定相机的注视点应落在像面中心附近
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overhead_camera")
    if cam_id >= 0:
        check = _project(data.cam_xpos[cam_id], points["box_01"], fovy, width, height, points["box_01"])
        print("投影自校验（场景固定相机 overhead_camera 投它自己的注视点 box_01）：%s（应接近画面中心 %s）"
              % (None if check is None else [round(check[0], 1), round(check[1], 1)],
                 [width / 2, height / 2]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
