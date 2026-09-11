"""从 MuJoCo 正上方相机自动保存 Piper 场景截图。"""
import argparse
from pathlib import Path
import mujoco
from PIL import Image

def capture(model_path, output):
    model=mujoco.MjModel.from_xml_path(str(Path(model_path).resolve())); data=mujoco.MjData(model); mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=480,width=640); renderer.update_scene(data,camera="overhead_camera")
    Image.fromarray(renderer.render()).save(output); print(output)

if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--model",required=True); p.add_argument("--output",default="build/calibration/piper-top.png"); a=p.parse_args(); Path(a.output).parent.mkdir(parents=True,exist_ok=True); capture(a.model,a.output)
