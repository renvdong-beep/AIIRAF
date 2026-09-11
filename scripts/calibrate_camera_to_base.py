"""使用对应点拟合相机到 Piper 基座的刚体变换。"""
import argparse, json
from pathlib import Path
import numpy as np

def calibrate(points, output):
    cam=np.asarray([p["camera_m"] for p in points],dtype=float); base=np.asarray([p["base_m"] for p in points],dtype=float)
    if len(points)<4: raise ValueError("至少需要 4 组不共面的对应点")
    ac=cam-cam.mean(0); ab=base-base.mean(0); u,_,vt=np.linalg.svd(ac.T@ab); r=vt.T@u.T
    if np.linalg.det(r)<0: vt[-1]*=-1; r=vt.T@u.T
    t=base.mean(0)-r@cam.mean(0); pred=(r@cam.T).T+t; err=np.linalg.norm(pred-base,axis=1)
    result={"schema_version":"iraf.camera-to-base/v1","translation_m":t.tolist(),"rotation_matrix":r.tolist(),"mean_error_m":float(err.mean()),"max_error_m":float(err.max()),"point_count":len(points),"passed":bool(err.max()<0.01)}
    Path(output).parent.mkdir(parents=True,exist_ok=True); Path(output).write_text(json.dumps(result,ensure_ascii=True,indent=2)+"\n"); print(json.dumps(result,ensure_ascii=True,indent=2))
if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--points",required=True); p.add_argument("--output",default="build/calibration/camera-to-base.json"); a=p.parse_args(); calibrate(json.loads(Path(a.points).read_text()),a.output)
