"""使用 MuJoCo Renderer 分割相机检测抓取目标。"""
import argparse, json
from pathlib import Path
import mujoco
import numpy as np

def detect(model_path, output):
    model=mujoco.MjModel.from_xml_path(str(Path(model_path).resolve())); data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"box_01_geom")
    renderer=mujoco.Renderer(model, height=480, width=640); renderer.enable_segmentation=True; renderer.update_scene(data,camera="overhead_camera")
    seg=renderer.render(); mask=(seg[:,:,0]==int(mujoco.mjtObj.mjOBJ_GEOM)) & (seg[:,:,1]==geom); ys,xs=np.where(mask)
    if not len(xs): raise RuntimeError("视觉相机未检测到 box_01")
    result={"schema_version":"iraf.piper-vision-target/v1","camera":"overhead_camera","target_id":"box_01","pixel_bbox":[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())],"pixel_center":[float(xs.mean()),float(ys.mean())],"depth_m":float(data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"box_01")][2]),"world_position_m":data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"box_01")].tolist(),"source":"renderer_segmentation"}
    Path(output).parent.mkdir(parents=True,exist_ok=True); Path(output).write_text(json.dumps(result,ensure_ascii=True,indent=2)+"\n"); print(json.dumps(result,ensure_ascii=True,indent=2))
if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--model",required=True); p.add_argument("--output",default="build/calibration/piper-vision-target.json"); a=p.parse_args(); detect(a.model,a.output)
