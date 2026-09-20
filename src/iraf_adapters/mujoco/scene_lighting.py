"""按配置声明向 MJCF 注入光源（纯视觉层，不参与物理）。

为什么单独成模块：两个场景生成器都需要它，而"默认不注入光源、只靠 MuJoCo 头灯"
会让离屏相机图接近全黑（实测均值 16~24/255、98% 像素≈近黑），
颜色分割虽然仍能工作（按色调判据），但人眼与证据截图都不可读。

声明（`config` 的 `scene.lights`，**缺省不注入**以保持既有行为）：

```yaml
scene:
  lights:
    - name: key                      # 主光
      directional: true              # 方向光：只用 dir，pos 忽略
      dir: [-0.3, 0.5, -1.0]         # 光照传播方向（从光源指向场景）
      diffuse: [1.0, 0.98, 0.95]
      specular: [0.4, 0.4, 0.4]
      castshadow: true
```

校验：dir 不得为零向量；diffuse/specular 必须非负有限数；
`directional` 缺省 true（方向光不依赖机型工作空间位置，跨机型可移植）。
"""

import math
import xml.etree.ElementTree as ET


class SceneLightingError(ValueError):
    """光源声明非法（显式失败，不静默跳过）。"""


def _vector(value, label, allow_zero=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise SceneLightingError("%s 必须是 3 个数值" % label)
    values = [float(item) for item in value]
    if not all(math.isfinite(item) for item in values):
        raise SceneLightingError("%s 必须是有限数" % label)
    if not allow_zero and max(abs(item) for item in values) < 1e-9:
        raise SceneLightingError("%s 不能为零向量" % label)
    return values


def inject_lights(world, lights):
    """把声明的光源注入 `<worldbody>`，返回注入的光源名列表。

    未声明（None 或空列表）时不注入任何光源：保持既有场景行为不变，
    是否要"亮起来"由配置决定，不由代码隐含决定。
    """
    if lights is None:
        return []
    if not isinstance(lights, (list, tuple)):
        raise SceneLightingError("scene.lights 必须是列表")
    injected = []
    for index, item in enumerate(lights):
        if not isinstance(item, dict):
            raise SceneLightingError("scene.lights[%d] 必须是对象" % index)
        unknown = sorted(set(item) - {"name", "dir", "diffuse", "specular", "castshadow", "directional", "pos"})
        if unknown:
            raise SceneLightingError(
                "scene.lights[%d] 含未知字段: %s" % (index, unknown)
            )
        name = str(item.get("name") or ("light_%d" % (index + 1)))
        attributes = {
            "name": name,
            "dir": "%.6f %.6f %.6f" % tuple(_vector(item.get("dir"), "lights[%d].dir" % index)),
            "diffuse": "%.4f %.4f %.4f" % tuple(
                _vector(item.get("diffuse", [0.8, 0.8, 0.8]), "lights[%d].diffuse" % index, allow_zero=True)
            ),
            "specular": "%.4f %.4f %.4f" % tuple(
                _vector(item.get("specular", [0.2, 0.2, 0.2]), "lights[%d].specular" % index, allow_zero=True)
            ),
            # 方向光不依赖机型工作空间位置，跨机型可移植；点光需 pos。
            "directional": "true" if bool(item.get("directional", True)) else "false",
        }
        if attributes["directional"] == "false":
            attributes["pos"] = "%.6f %.6f %.6f" % tuple(
                _vector(item.get("pos", [0.0, 0.0, 1.0]), "lights[%d].pos" % index, allow_zero=True)
            )
        if bool(item.get("castshadow", False)):
            attributes["castshadow"] = "true"
        ET.SubElement(world, "light", attributes)
        injected.append(name)
    return injected
