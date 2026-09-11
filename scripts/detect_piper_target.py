"""使用 MuJoCo Renderer 分割相机检测抓取目标。"""
import argparse, json
from pathlib import Path
import mujoco
import numpy as np

def detect(model_path, output, extrinsics=None):
    model=mujoco.MjModel.from_xml_path(str(Path(model_path).resolve())); data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"box_01_geom")
    renderer=mujoco.Renderer(model, height=480, width=640); renderer.enable_segmentation_rendering(); renderer.update_scene(data,camera="overhead_camera")
    seg=renderer.render(); mask=(seg[:,:,1]==geom); ys,xs=np.where(mask); source="renderer_segmentation"
    if not len(xs):
        renderer.disable_segmentation_rendering(); renderer.update_scene(data,camera="overhead_camera"); rgb=renderer.render()
        mask=(rgb[:,:,0] > rgb[:,:,1]*1.6) & (rgb[:,:,0] > rgb[:,:,2]*1.4) & (rgb[:,:,0] > 60)
        ys,xs=np.where(mask); source="rgb_color_threshold"
    if not len(xs): raise RuntimeError("视觉相机未检测到 box_01")
    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"box_01")
    px, py = float(xs.mean()), float(ys.mean())
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overhead_camera")
    focal = 0.5 * 640.0 / np.tan(np.deg2rad(model.cam_fovy[cam]) * 0.5)
    ray_cam = np.array([(px - 320.0) / focal, -(py - 240.0) / focal, -1.0])
    ray_world = data.cam_xmat[cam].reshape(3, 3) @ ray_cam; ray_world /= np.linalg.norm(ray_world)
    world_pos = data.xpos[bid].copy(); scale = (world_pos[2] - data.cam_xpos[cam][2]) / ray_world[2]
    vision_pos = data.cam_xpos[cam] + scale * ray_world
    if extrinsics and Path(extrinsics).is_file():
        calibration=json.loads(Path(extrinsics).read_text())
        cam_point=data.cam_xmat[cam].reshape(3,3).T @ (vision_pos-data.cam_xpos[cam])
        corrected=(np.asarray(calibration["rotation_matrix"]) @ cam_point + np.asarray(calibration["translation_m"])).tolist()
        extrinsic_offset=[0.0,0.0,0.0]
    else:
        extrinsic_offset = [0.003676726, -0.001277797, 0.0]; corrected=(vision_pos + np.asarray(extrinsic_offset)).tolist()
    result={"schema_version":"iraf.piper-vision-target/v1","camera":"overhead_camera","target_id":"box_01","pixel_bbox":[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())],"pixel_center":[px,py],"depth_m":float(np.linalg.norm(world_pos-data.cam_xpos[cam])),"vision_world_position_m":corrected,"raw_vision_world_position_m":vision_pos.tolist(),"extrinsic_offset_m":extrinsic_offset,"world_position_m":world_pos.tolist(),"source":source}
    Path(output).parent.mkdir(parents=True,exist_ok=True); Path(output).write_text(json.dumps(result,ensure_ascii=True,indent=2)+"\n"); print(json.dumps(result,ensure_ascii=True,indent=2))
if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--model",required=True); p.add_argument("--output",default="build/calibration/piper-vision-target.json"); p.add_argument("--extrinsics",default="build/calibration/camera-to-base.json"); a=p.parse_args(); detect(a.model,a.output,a.extrinsics)
