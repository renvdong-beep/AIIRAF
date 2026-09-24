"""带连续步进、指标和受控故障注入的 MuJoCo 3 后端。"""

from pathlib import Path
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import mujoco
import numpy as np
import yaml

from iraf_skills.common.trajectory import quintic_position

#: 伺服前馈（重力静差补偿）的单关节上限，单位 rad。
#: 用途是拦截"单位/符号写错"这类配置错误（例如误把力矩 N·m 填进来、
#: 或把方向写反），而不是限制正常取值：实测纯 PD 的 UR5e 需要最大 0.017 rad。
#: 前馈量级必须远小于关节行程，否则它就不再是"补偿"而是另一条动作指令。
GRAVITY_FEEDFORWARD_LIMIT_RAD = 0.1

#: 夹爪几何相关的必需字段：涉及"抓取点在哪、指腹在哪"的判定都必须显式声明。
#: **刻意不提供任何机型默认值**：早期实现把缺省值指向 Piper 的
#: `link6/link7/link8` 与 `piper_left_finger/piper_right_finger`，
#: 结果换构型后要么静默用了错误几何、要么在运行时抛"缺少 body: link6"，
#: 两种都难以定位。缺失即显式失败，并由场景生成器负责把配置里的声明写进 report。
GRIPPER_GEOMETRY_FIELDS = (
    "wrist_body",
    "left_finger_body",
    "right_finger_body",
    "left_finger_geom",
    "right_finger_geom",
)


#: 视觉证据的刷新策略：always（每次抓取前刷新，默认）/ on_missing（仅证据缺失时）
#: / never（只读现有证据）。
VISION_REFRESH_MODES = ("always", "on_missing", "never")

#: 视觉检测器命令允许的占位符。写错占位符＝配置错误，必须在执行前显式失败，
#: 否则会出现"命令跑了但参数没传进去"这类难查的静默故障。
VISION_COMMAND_PLACEHOLDERS = ("python", "model", "evidence", "config", "target_id", "calibration")

#: 生成的检测器输入配置落盘位置（属构建产物，按仓库约定写在 build/ 下）。
DEFAULT_DETECTION_CONFIG_OUTPUT = "build/calibration/detection-config.json"

#: 刷新策略的调试用环境变量覆盖（不是契约，契约见 vision.refresh）。
VISION_REFRESH_ENV = "IRAF_REFRESH_VISION"


def _project_root():
    """仓库根目录（本文件位于 <root>/src/iraf_adapters/mujoco/）。"""
    return Path(__file__).resolve().parents[3]


def _resolve_project_path(value):
    """解析配置里的路径：绝对路径原样，相对路径相对仓库根目录。"""
    path = Path(str(value))
    return path if path.is_absolute() else _project_root() / path


def _render_detector_command(command, values):
    """把视觉检测器命令里的占位符替换为实际值。

    未知占位符显式失败：`{modle}` 这类拼写错误若被静默忽略，
    检测器会带着默认参数跑出一个"看起来成功"的错误证据。
    """
    rendered = []
    for arg in command:
        for name in re.findall(r"\{([^{}]*)\}", arg):
            if name not in values:
                raise ValueError(
                    "视觉检测器命令含未知占位符 {%s}（可用: %s）"
                    % (name, sorted(values))
                )
        rendered.append(arg.format(**values))
    return rendered


def _parse_vision_config(config):
    """解析后端配置的可选 `vision` 段（声明式视觉 Provider）。

    背景：早期实现把证据文件路径与检测器脚本路径**写死在后端里**
    （build/calibration/piper-vision-target.json、scripts/detect_piper_target*.py），
    违反铁律 6.5（只依赖能力，不依赖具体模型名/设备路径）：换机器人后要么
    静默复用了别机型的检测器，要么报错指向一个与调用方无关的路径。

    现在后端只认声明，且**任何字段都不是必填**：

        vision:
          evidence_file: <证据文件路径>          # 可选
          refresh: always | on_missing | never  # 可选，默认 always
          detector:                             # 可选；不声明＝不刷新，只读证据
            command: [argv, ...]                # 支持占位符 {python}/{model}/{evidence}/{config}/{target_id}
            config_file: <检测器参数真源>        # 可选；其 depth 段覆盖内置默认值
            config_output: <生成物落盘位置>      # 可选

    未声明 `vision` 段时，`visual_pick` 必须由请求显式给出 `vision_file`，否则显式失败。
    """
    if config is None:
        return None
    if not isinstance(config, dict):
        raise ValueError("vision 配置必须是对象")
    unknown = sorted(set(config) - {"evidence_file", "refresh", "detector"})
    if unknown:
        raise ValueError("vision 含未知字段: " + str(unknown))

    evidence_file = config.get("evidence_file")
    if evidence_file is not None and (
        not isinstance(evidence_file, str) or not evidence_file
    ):
        raise ValueError("vision.evidence_file 必须是非空字符串")

    refresh = str(config.get("refresh", "always"))
    if refresh not in VISION_REFRESH_MODES:
        raise ValueError(
            "vision.refresh 必须是 %s 之一，实际: %s"
            % (list(VISION_REFRESH_MODES), refresh)
        )

    detector_raw = config.get("detector")
    detector = None
    if detector_raw is not None:
        if not isinstance(detector_raw, dict):
            raise ValueError("vision.detector 必须是对象")
        unknown = sorted(
            set(detector_raw)
            - {"command", "config_file", "config_output", "calibration_file"}
        )
        if unknown:
            raise ValueError("vision.detector 含未知字段: " + str(unknown))
        command = detector_raw.get("command")
        if not isinstance(command, (list, tuple)) or not command:
            raise ValueError("vision.detector.command 必须是非空命令数组")
        command = [str(item) for item in command]
        if not all(item for item in command):
            raise ValueError("vision.detector.command 元素不能为空字符串")
        allowed = set(VISION_COMMAND_PLACEHOLDERS)
        for arg in command:
            for name in re.findall(r"\{([^{}]*)\}", arg):
                if name not in allowed:
                    raise ValueError(
                        "vision.detector.command 含未知占位符 {%s}（可用: %s）"
                        % (name, sorted(allowed))
                    )
        config_file = detector_raw.get("config_file")
        if config_file is not None and (
            not isinstance(config_file, str) or not config_file
        ):
            raise ValueError("vision.detector.config_file 必须是非空字符串")
        config_output = detector_raw.get("config_output")
        if config_output is not None and (
            not isinstance(config_output, str) or not config_output
        ):
            raise ValueError("vision.detector.config_output 必须是非空字符串")
        calibration_file = detector_raw.get("calibration_file")
        if calibration_file is not None and (
            not isinstance(calibration_file, str) or not calibration_file
        ):
            raise ValueError("vision.detector.calibration_file 必须是非空字符串")
        # 相机标定**必须按机型声明**：标定文件里是"这台相机相对这个基座"的外参，
        # 复用另一台机器人的标定会得到"看起来合理但实际错误"的坐标
        # （实测风险：默认路径 build/calibration/camera_to_base.json 曾是 Piper 的）。
        if any("{calibration}" in arg for arg in command) and not calibration_file:
            raise ValueError(
                "vision.detector.command 使用 {calibration} 时必须声明 "
                "vision.detector.calibration_file（相机标定按机型独立）"
            )
        detector = {
            "command": command,
            "config_file": config_file,
            "config_output": config_output or DEFAULT_DETECTION_CONFIG_OUTPUT,
            "calibration_file": calibration_file,
        }
    return {"evidence_file": evidence_file, "refresh": refresh, "detector": detector}


def _declared_gripper_geometry(gripper):
    """取出夹爪几何声明的必需字段；缺失即显式失败（不提供机型默认值）。"""
    missing = [key for key in GRIPPER_GEOMETRY_FIELDS if not gripper.get(key)]
    if missing:
        raise ValueError(
            "夹爪几何声明不完整，缺少字段: %s（必须由 profile/config 声明，"
            "不允许隐式默认值）" % missing
        )
    return {key: str(gripper[key]) for key in GRIPPER_GEOMETRY_FIELDS}


def _parse_name_map(name_map, model):
    """解析并**校验**「声明名 → 模型名」映射（联合模型专有）；返回 (正向表, 反向表)。

    为什么需要（A 方案，2026-09-24）：联合模型把附加本体整体加前缀（`piper_joint1`），
    而 Profile / 技能参数口径是**声明名**（`joint1`）。若不映射，臂后端在联合模型上
    直接报 `找不到关节或执行器: joint1`（实测）——"Profile 名 == 模型名"这条隐含前提
    第一次被跨本体场景打破。映射表由构建器**机械生成**（联合报告 `manipulation.name_map`），
    本函数只做校验与查表，不生成、不猜。

    校验（全部 fail-closed，装配期就失败而不是等第一次运动）：
      * 键值必须都是非空字符串，且 `声明名 != 模型名`（同值映射是声明错误，不是"无需映射"）；
      * **声明名不得已存在于模型里**（否则解析有歧义：到底指主本体的对象还是附加本体的？）；
      * **模型名必须存在于模型里**（joint/body/geom/site/actuator 之一）；
      * 反向表不得出现多对一（两个声明名指向同一模型名 ⇒ 回写 `last_positions` 时会互相覆盖）。
    未声明映射（None / 空表）时返回两张空表：所有解析原样返回，单本体路径行为逐位不变。
    """
    if name_map is None:
        return {}, {}
    if not isinstance(name_map, dict):
        raise ValueError("name_map 必须是对象（声明名 → 模型名）；实际 %r" % (name_map,))
    if not name_map:
        return {}, {}

    def spot(name):
        """名字在模型里的种类（取第一个命中）；不存在返回 None。"""
        text = str(name)
        for label in ("joint", "body", "geom", "site", "actuator"):
            kind = getattr(mujoco.mjtObj, "mjOBJ_%s" % label.upper())
            if mujoco.mj_name2id(model, kind, text) >= 0:
                return label
        return None

    forward = {}
    reverse = {}
    for declared, model_name in name_map.items():
        key = str(declared)
        value = str(model_name)
        if not key or not value:
            raise ValueError("name_map 的键值必须是非空字符串：%r -> %r" % (declared, model_name))
        if key == value:
            raise ValueError(
                "name_map 出现同值映射（%s -> %s）：同值不需要映射，出现即声明错误" % (key, value))
        existing = spot(key)
        if existing is not None:
            raise ValueError(
                "name_map 的声明名 %s 已存在于模型里（作为 %s）：解析会有歧义 —— 是主本体的对象"
                "还是附加本体的？拒绝装配，不静默选一个" % (key, existing))
        found = spot(value)
        if found is None:
            raise ValueError(
                "name_map 指向的模型名 %s 不存在（joint/body/geom/site/actuator 都没有）："
                "映射已过期或报告与模型不匹配" % value)
        if value in reverse:
            raise ValueError(
                "name_map 多对一：%s 与 %s 都指向 %s ⇒ 关节状态回写会互相覆盖"
                % (reverse[value], key, value))
        forward[key] = value
        reverse[value] = key
    return forward, reverse


class MujocoBackend:
    @classmethod
    def from_config(cls, config, profile, authority):
        fault_injection_enabled = config.get("fault_injection_enabled", False)
        if not isinstance(fault_injection_enabled, bool):
            raise ValueError("fault_injection_enabled 必须是布尔值")
        return cls(
            config["model_path"],
            profile,
            authority,
            fault_injection_enabled=fault_injection_enabled,
            manipulation_config=config.get("manipulation"),
            vision_config=config.get("vision"),
            realtime=bool(config.get("realtime", False)),
            display_size=(
                int(config.get("display_width", 640)),
                int(config.get("display_height", 480)),
            ),
            # 声明名 → 模型名（联合模型专有；单本体产物不带此键 ⇒ 行为逐位不变）
            name_map=config.get("name_map"),
        )

    def __init__(
        self,
        model_path,
        profile,
        authority,
        fault_injection_enabled=False,
        manipulation_config=None,
        vision_config=None,
        realtime=False,
        display_size=(640, 480),
        name_map=None,
    ):
        self.profile = profile
        self.authority = authority
        self._model_path = str(Path(model_path).resolve())
        self.model = mujoco.MjModel.from_xml_path(self._model_path)
        self.data = mujoco.MjData(self.model)
        # **按模型自带的关键帧初始化位形**（存在时）。
        # 为什么必须做：MjData 的默认 qpos 是**全零**，而全零位形对多数
        # 6 轴臂意味着"手臂竖直向上 / 夹爪水平伸出"。pick_object 的
        # HOME_HOLD 段是从**当前位形**插值到 HOME 的：
        # 从全零位插到"夹爪朝下"的 HOME，中途 pad 会降到台面以下
        # （实测 UR5e：t=0.33 处 pad_z=-0.0047m），沿途把方块顶飞
        # （实测方块被推到 z=5.3m，对齐门禁随即报 59m 偏差）。
        # MJCF 的 keyframe[0] 是模型作者声明的"合理初始位形"
        # （UR5e 官方 home / 我们的场景会把它写成抓取起始位形），
        # 用它初始化即可消除跨台面扫掠。
        # 无 keyframe 时保持全零，行为与改动前一致（Piper 不受影响）。
        # 用 getattr 探测而非直接取属性：单测会用 SimpleNamespace 替身，
        # 没有 nkey 字段（直接取会在装配期抛 AttributeError）。
        key_count = int(getattr(self.model, "nkey", 0) or 0)
        if key_count > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self._actuators = {
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            ): index
            for index in range(self.model.nu)
        }
        self.last_positions = {joint: 0.0 for joint in profile.joints}
        # 声明名 → 模型名映射（A 方案）：联合模型（狗 + 臂同一个 MJCF）里附加本体的对象一律带前缀
        # （`piper_joint1`、`piper_link6`），而 Profile / 技能参数口径是**声明名**（`joint1`）。
        # 单本体产物不带 name_map ⇒ 两张表都空 ⇒ 所有解析原样返回，行为与改动前逐位一致。
        self._name_map, self._declaration_by_model = _parse_name_map(name_map, self.model)
        self.stopped = False
        self._cancel_event = threading.Event()
        self._data_lock = threading.RLock()
        self._loop_lock = threading.RLock()
        self._metrics_lock = threading.RLock()
        self._loop_stop = threading.Event()
        self._loop_thread = None
        self._continuous_mode = False
        self._fault_injection_enabled = bool(fault_injection_enabled)
        self._fault_kind = None
        self._fault_delay_seconds = 0.0
        self._fault_remaining = 0
        self._manipulation = self._parse_manipulation_config(manipulation_config)
        # 视觉 Provider 声明（可选）：证据文件、刷新策略、检测器命令。
        # 后端不提供任何机型默认路径，未声明即"只能读请求显式给出的证据"。
        self._vision = _parse_vision_config(vision_config)
        self._realtime = bool(realtime)
        self._reset_metrics()
        # 离屏显示通道：惰性创建，供 render_frames 使用（与物理步进解耦）。
        width, height = display_size
        if int(width) < 1 or int(height) < 1:
            raise ValueError("display_size 必须是正整数对")
        self._display_size = (int(width), int(height))
        self._display_renderer_cache = None

    def runtime_inventory(self):
        return {
            "safety": {"estop": False},
            "manipulation": {
                "target_visible": bool(self._manipulation["targets"])
            },
            "mode": "simulation",
        }

    def calibrate_grasp(self, config=None, lease=None):
        """读取当前模型的腕部和真实指尖 geom，生成可审计标定证据。"""
        self.authority.validate(lease)
        gripper_cfg = (self._manipulation.get("gripper") or {})
        names = _declared_gripper_geometry(gripper_cfg)
        wrist = self._body_id(names["wrist_body"])

        def geom(name, label):
            """取声明的指腹接触面 geom；不存在即显式失败。

            不再回退到 body 原点：2F-85 的 pad body 原点在铰链附近，与 pad box
            中心相差约 2cm，回退会让标定结果悄悄偏掉。
            """
            ident = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if ident < 0:
                raise ValueError("夹爪 %s 声明的 geom 不存在: %s" % (label, name))
            return self.data.geom_xpos[ident].copy()

        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            w = self.data.xpos[wrist].copy()
            l = geom(names["left_finger_geom"], "左指")
            r = geom(names["right_finger_geom"], "右指")
        center = (l + r) / 2.0; offset = center - w; norm = max(float((offset @ offset) ** 0.5), 1e-9)
        return {"wrist_position_m": w.tolist(), "left_finger_position_m": l.tolist(), "right_finger_position_m": r.tolist(), "grasp_center_m": center.tolist(), "finger_separation_m": float(((l-r) @ (l-r)) ** 0.5), "wrist_body": names["wrist_body"], "tcp_offset_from_wrist_m": offset.tolist(), "approach_axis_world": (offset / norm).tolist(), "recommended_pregrasp_offset_m": 0.04}

    def _calibrate_intrinsics(self, samples, image_size, rotation, translation):
        """标定针孔内参：principal_point_px 与 focal_px（Levenberg-Marquardt）。

        残差为每个样本的 (预测像素 - 观测像素)。初值由 fovy 与图像中心给出，
        迭代中若出现非物理解（焦距非正）则拒绝该步，保证结果可审计。
        """
        rotation_matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
        offset = np.asarray(translation, dtype=float).reshape(3)
        points = np.zeros((len(samples), 3), dtype=float)
        pixels = np.zeros((len(samples), 2), dtype=float)
        for index, sample in enumerate(samples):
            pixel = sample.get("pixel")
            base = sample.get("base_m")
            if not isinstance(pixel, (list, tuple)) or len(pixel) != 2:
                raise ValueError("内参标定 pixel 必须是 2 个数值")
            if not isinstance(base, (list, tuple)) or len(base) != 3:
                raise ValueError("内参标定 base_m 必须是 3 个数值")
            base_point = np.asarray([float(value) for value in base], dtype=float)
            pixel_point = np.asarray([float(pixel[0]), float(pixel[1])], dtype=float)
            if not np.isfinite(base_point).all() or not np.isfinite(pixel_point).all():
                raise ValueError("内参标定必须是有限数值")
            # 相机坐标系：p_base = R @ p_cam + t  =>  p_cam = R.T @ (p_base - t)
            points[index] = rotation_matrix.T @ (base_point - offset)
            pixels[index] = pixel_point
        # MuJoCo 相机前方为 -Z，统一取正深度。
        depths = -points[:, 2]
        if float(depths.min()) <= 1e-6:
            raise ValueError("内参标定样本必须位于相机前方")
        lateral = float(np.abs(points[:, :2]).max())
        if lateral < 1e-4:
            raise ValueError("内参标定样本横向散布过小，无法约束主点: %.9f" % lateral)

        width, height = float(image_size[0]), float(image_size[1])

        def residuals(params):
            focal, cu, cv = (float(value) for value in params)
            if focal <= focal_floor:
                return None
            # 经验证：u 随相机系 X 同向（u = f*X/d + cu），v 随 Y 反向（v = -f*Y/d + cv）。
            predicted_u = focal * points[:, 0] / depths + cu
            predicted_v = -focal * points[:, 1] / depths + cv
            return np.concatenate((predicted_u - pixels[:, 0], predicted_v - pixels[:, 1]))

        # 焦距物理下界：不允许小于图像短边的 10%，避免滑向焦距趋零的退化解。
        focal_floor = 0.1 * min(width, height)
        # 初值：焦距由垂直视场角推算，主点取图像中心。
        params = np.array([0.5 * height / 0.8097, width / 2.0, height / 2.0], dtype=float)
        base_residual = residuals(params)
        if base_residual is None:
            raise ValueError("内参标定初值无效")
        cost = float(base_residual @ base_residual)
        damping = 1e-3
        for _ in range(200):
            jacobian = np.zeros((base_residual.size, 3), dtype=float)
            for column in range(3):
                # 用相对步长做数值差分：焦距量级约 300，绝对步长 1e-6 会让
                # Jacobian 全为零，优化器就会滑向焦距趋零的退化解。
                step_size = max(abs(params[column]) * 1e-6, 1e-6)
                probe = params.copy()
                probe[column] += step_size
                probe_residual = residuals(probe)
                if probe_residual is None:
                    jacobian[:, column] = 0.0
                    continue
                jacobian[:, column] = (probe_residual - base_residual) / step_size
            current = residuals(params)
            normal = jacobian.T @ jacobian + damping * np.eye(3)
            gradient = jacobian.T @ current
            try:
                delta = np.linalg.solve(normal, -gradient)
            except np.linalg.LinAlgError:
                break
            candidate = params + delta
            candidate_residual = residuals(candidate)
            if candidate_residual is None:
                damping *= 10.0
                if damping > 1e12:
                    break
                continue
            candidate_cost = float(candidate_residual @ candidate_residual)
            if candidate_cost < cost:
                params = candidate
                base_residual = candidate_residual
                cost = candidate_cost
                damping = max(damping * 0.3, 1e-9)
            else:
                damping *= 10.0
                if damping > 1e12:
                    break

        final_residual = residuals(params)
        if final_residual is None:
            raise ValueError("内参标定未收敛到物理解")
        focal, cu, cv = (float(value) for value in params)
        if not math.isfinite(focal) or focal <= focal_floor:
            raise ValueError(
                "内参标定焦距越界: focal=%.6f floor=%.6f" % (focal, focal_floor)
            )
        # 只在参数变化已不可见时接受结果，避免把未收敛的解当成标定值。
        convergence_error = float(np.sqrt((final_residual**2).mean()))
        return {
            "focal_px": focal,
            "principal_point_px": [cu, cv],
            "image_size_px": [int(image_size[0]), int(image_size[1])],
            "residual_px_rms": convergence_error,
            "sample_count": len(samples),
        }

    def calibrate_camera_to_base(self, inputs, lease):
        """用对应点对拟合相机外参与内参，输出可审计误差。"""
        self.authority.validate(lease)
        config = inputs if isinstance(inputs, dict) else {"pairs": inputs}
        pairs = config.get("pairs")
        if pairs is None and isinstance(inputs, (list, tuple)):
            pairs = inputs
        if not isinstance(pairs, (list, tuple)) or len(pairs) < 4:
            raise ValueError("相机标定至少需要 4 组对应点")
        camera_points = np.zeros((len(pairs), 3), dtype=float)
        base_points = np.zeros((len(pairs), 3), dtype=float)
        for index, pair in enumerate(pairs):
            if not isinstance(pair, dict):
                raise ValueError("相机标定对应点必须是对象: 下标 " + str(index))
            camera = pair.get("camera_m")
            base = pair.get("base_m")
            for label, value in (("camera_m", camera), ("base_m", base)):
                if not isinstance(value, (list, tuple)) or len(value) != 3:
                    raise ValueError("相机标定 %s 必须是 3 个数值: 下标 %d" % (label, index))
            camera_points[index] = [float(value) for value in camera]
            base_points[index] = [float(value) for value in base]
        if not np.isfinite(camera_points).all() or not np.isfinite(base_points).all():
            raise ValueError("相机标定对应点必须是有限数值")

        max_iterations = 64
        tolerance = 1e-12
        rotation = np.eye(3)
        translation = np.zeros(3)
        for _ in range(max_iterations):
            base_centroid = base_points.mean(axis=0)
            # 先固定当前 R 求最优平移，再对去中心点做 SVD 求最优旋转（Kabsch 迭代）。
            moved = camera_points @ rotation.T + translation
            translation = translation + (base_centroid - moved.mean(axis=0))
            centered_camera = (camera_points @ rotation.T + translation) - base_centroid
            centered_base = base_points - base_centroid
            covariance = centered_camera.T @ centered_base
            u, _, vt = np.linalg.svd(covariance)
            correction = vt.T @ u.T
            if np.linalg.det(correction) < 0:
                vt[-1, :] *= -1.0
                correction = vt.T @ u.T
            rotation = correction @ rotation
            residuals = camera_points @ rotation.T + translation - base_points
            if float(np.abs(residuals).max()) <= tolerance:
                break

        residual_vectors = camera_points @ rotation.T + translation - base_points
        distances = np.sqrt((residual_vectors**2).sum(axis=1))
        mean_error = float(distances.mean())
        max_error = float(distances.max())
        tolerance_m = float(config.get("tolerance_m", 0.005))
        if not math.isfinite(tolerance_m) or tolerance_m <= 0:
            raise ValueError("相机标定 tolerance_m 必须是正有限数")

        result = {
            "translation_m": [float(value) for value in translation],
            "rotation_matrix": [[float(value) for value in row] for row in rotation],
            "mean_error_m": mean_error,
            "max_error_m": max_error,
            "passed": bool(max_error <= tolerance_m),
            "point_count": len(pairs),
            "tolerance_m": tolerance_m,
        }

        samples = config.get("intrinsics_samples")
        if samples:
            image_size = config.get("image_size_px") or [640, 480]
            result["intrinsics"] = self._calibrate_intrinsics(
                samples, image_size, rotation, translation
            )
        return result

    def visual_pick(self, inputs, lease):
        """按视觉证据驱动抓取。

        证据来源与刷新方式**全部由配置声明**（见 `_parse_vision_config`），
        后端不含任何机型专有路径：未声明 `vision` 时只能读请求里的 `vision_file`。

        证据文件格式兼容两种：
        - 单目标格式：顶层 vision_world_position_m + target_id，姿态默认单位四元数；
        - 多目标格式（检测器输出，schema `iraf.vision-targets/v1`）：
          targets 数组，按 target_id 选取，携带 6DoF 姿态与顶面法向。
        两种格式按**结构**区分（是否有 targets 数组），不看 schema 字符串，
        因此历史证据文件（Piper 命名时期）与新证据文件都能读。
        """
        target_id = inputs["target_id"]
        path = self._resolve_vision_evidence(inputs)
        # 只判定一次：on_missing 模式下刷新成功后文件已存在，再判一次会得到相反结果。
        refreshed = self._should_refresh_vision(path)
        if refreshed:
            self._run_vision_detector(target_id, path)
        if not path.is_file():
            raise RuntimeError("视觉目标证据不存在: " + str(path))
        data = json.loads(path.read_text())

        entry, position, quaternion_wxyz, source_label = self._select_vision_entry(
            data, target_id
        )
        frame_id = entry.get("frame_id", data.get("frame_id", "world"))
        if frame_id != "world":
            raise RuntimeError("视觉目标必须已转换到 world 坐标系")

        orientation = {
            "w": quaternion_wxyz[0],
            "x": quaternion_wxyz[1],
            "y": quaternion_wxyz[2],
            "z": quaternion_wxyz[3],
        }
        result = self.pick_object(
            target_id,
            {
                "frame_id": frame_id,
                "position": dict(zip(("x", "y", "z"), position)),
                "orientation": orientation,
            },
            int(inputs.get("duration_ms", 10000)),
            lease,
        )
        result.setdefault("evidence", {}).update(
            {
                "vision": {
                    "source": source_label,
                    "schema_version": data.get("schema_version", "unknown"),
                    "frame_id": frame_id,
                    "pixel_center": entry.get("pixel_center"),
                    "pixel_bbox": entry.get("pixel_bbox"),
                    "depth_m": entry.get("depth_m"),
                    "vision_world_position_m": position,
                    "vision_quaternion_wxyz": quaternion_wxyz,
                    "vision_normal": entry.get("normal"),
                    "vision_size_m": entry.get("size_m"),
                    "vision_residual_m": entry.get("residual_m"),
                    "evidence_file": str(path),
                    # 留证：证据是自己刷新的还是外部提供的，以及声明式的刷新策略。
                    "refreshed": bool(refreshed),
                    "refresh_mode": (self._vision or {}).get("refresh", "always")
                    if self._vision
                    else "undeclared",
                }
            }
        )
        return result

    def _resolve_vision_evidence(self, inputs):
        """确定视觉证据文件路径：请求显式给出优先，其次配置声明。

        两者都没有时显式失败 —— 不再回退到某个机型的固定路径。
        """
        declared = (self._vision or {}).get("evidence_file")
        value = inputs.get("vision_file") or declared
        if not value:
            raise RuntimeError(
                "未提供视觉证据路径：请在请求参数里给 vision_file，"
                "或在后端配置的 vision.evidence_file 中声明（后端不提供机型默认路径）"
            )
        return _resolve_project_path(value)

    def _should_refresh_vision(self, path):
        """是否在抓取前刷新视觉证据。

        优先级：环境变量 `IRAF_REFRESH_VISION`（0/1，调试用显式覆盖）
        ＞ 配置声明 `vision.refresh`（always / on_missing / never）。
        未声明 detector 命令时一律不刷新。
        """
        detector = (self._vision or {}).get("detector")
        if not detector:
            return False
        override = os.environ.get(VISION_REFRESH_ENV)
        if override is not None:
            if override not in ("0", "1"):
                raise ValueError(
                    "%s 只能是 0 或 1，实际: %s" % (VISION_REFRESH_ENV, override)
                )
            return override == "1"
        mode = (self._vision or {}).get("refresh", "always")
        if mode == "always":
            return True
        if mode == "never":
            return False
        return not path.is_file()

    def _run_vision_detector(self, target_id, evidence_path):
        """执行声明式检测器命令刷新视觉证据；失败即显式失败（不沿用旧证据）。"""
        detector = (self._vision or {}).get("detector") or {}
        config_path = self._detection_config_path(detector)
        values = {
            "python": sys.executable,
            "model": self._model_path,
            "evidence": str(evidence_path),
            "config": str(config_path),
            "target_id": str(target_id),
            # 标定文件按机型声明；未声明时为空串（命令里用到它会在装配期就失败）
            "calibration": str(_resolve_project_path(detector["calibration_file"]))
            if detector.get("calibration_file")
            else "",
        }
        command = _render_detector_command(detector["command"], values)
        try:
            subprocess.run(command, check=True, cwd=str(_project_root()))
        except (OSError, subprocess.CalledProcessError) as exc:
            raise RuntimeError(
                "视觉检测器执行失败: %s（命令: %s）" % (exc, command)
            ) from exc

    def _detection_config_path(self, detector_cfg=None):
        """生成检测器输入配置，返回其路径。

        检测器需要知道"场景里有哪些目标、各自什么颜色"才能按颜色分割：
        这些是**场景事实**，来自场景生成器写下的旁挂报告（<scene>.json），
        因此单目标与多目标场景共用同一条检测链路，不写死在配置里。

        参数真源（depth 段）可由 `vision.detector.config_file` 声明；
        未声明则使用内置默认值。声明的文件不存在或格式不对**显式失败**，
        不再静默忽略（静默忽略会让"改了参数却没生效"无法察觉）。

        配置写成 .json 而非 .yaml，避免本机 DLP 对 yaml 文件的加密干扰。
        """
        detector_cfg = detector_cfg or {}
        scene_path = Path(self._model_path)
        report_path = scene_path.with_suffix(".json")
        if not report_path.is_file():
            raise RuntimeError(
                "场景旁挂报告不存在，无法生成检测配置: " + str(report_path)
            )
        report = json.loads(report_path.read_text(encoding="utf-8"))
        half_size = float(report.get("target_half_size_m", 0.025))
        entries = report.get("targets")
        if not entries:
            # 兼容早期场景报告：只有单个 target_id。
            entries = [{"id": report.get("target_id", "box_01")}]

        targets = []
        for item in entries:
            rgba = item.get("rgba")
            if rgba is None:
                rgba = [0.82, 0.22, 0.12, 1.0]
            targets.append({"id": str(item["id"]), "rgba": [float(v) for v in rgba]})

        config = {
            "target": {
                "half_size_m": half_size,
                "rgba": targets[0]["rgba"],
            },
            "targets": targets,
            "depth": {
                "outlier_mad_scale": 3.0,
                "ransac_iterations": 200,
                "ransac_inlier_m": 0.002,
                "size_tolerance_m": 0.006,
                "max_points_per_target": 5000,
                "min_points_per_target": 100,
                "color_tolerance": 0.18,
            },
        }
        # 参数真源：声明了就用声明的，保证参数只有一处真源；未声明用内置默认值。
        config_file = detector_cfg.get("config_file")
        if config_file:
            shared = _resolve_project_path(config_file)
            if not shared.is_file():
                raise RuntimeError(
                    "vision.detector.config_file 不存在: " + str(shared)
                )
            shared_config = yaml.safe_load(shared.read_text(encoding="utf-8"))
            if not isinstance(shared_config, dict):
                raise ValueError(
                    "vision.detector.config_file 必须是对象: " + str(shared)
                )
            if isinstance(shared_config.get("depth"), dict):
                merged = dict(config["depth"])
                merged.update(
                    {
                        key: value
                        for key, value in shared_config["depth"].items()
                        if key != "symmetry_disambiguation"
                    }
                )
                config["depth"] = merged

        output = _resolve_project_path(
            detector_cfg.get("config_output") or DEFAULT_DETECTION_CONFIG_OUTPUT
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(config, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
        )
        return output

    @staticmethod
    def _select_vision_entry(data, target_id):
        """从视觉证据中取出指定目标的记录。

        返回 (entry, position, quaternion_wxyz, source_label)。
        多目标格式按 id 精确匹配；缺失时显式失败，绝不回退到"第一个目标"，
        否则"按 ID 抓取指定目标"的验收会退化成"抓到任意目标"。
        """
        targets = data.get("targets")
        if isinstance(targets, list) and targets:
            matched = [item for item in targets if item.get("id") == target_id]
            if not matched:
                available = ", ".join(str(item.get("id")) for item in targets)
                raise RuntimeError(
                    f"视觉证据中没有请求的目标 {target_id}；可选: {available}"
                )
            entry = matched[0]
            position = entry.get("position_m")
            if not isinstance(position, list) or len(position) != 3:
                raise RuntimeError(f"目标 {target_id} 的视觉坐标无效")
            quaternion = entry.get("quaternion_wxyz")
            if not isinstance(quaternion, list) or len(quaternion) != 4:
                raise RuntimeError(f"目标 {target_id} 的视觉姿态无效")
            norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
            if norm < 1e-9:
                raise RuntimeError(f"目标 {target_id} 的视觉姿态四元数为零")
            unit = [float(value) / norm for value in quaternion]
            return entry, [float(value) for value in position], unit, entry.get(
                "source", "pose_estimation"
            )

        # 单目标旧格式
        position = data.get("vision_world_position_m")
        if not isinstance(position, list) or len(position) != 3:
            raise RuntimeError("视觉目标坐标无效")
        if data.get("target_id") != target_id:
            raise RuntimeError("视觉目标 ID 与请求不一致")
        return data, [float(value) for value in position], [1.0, 0.0, 0.0, 0.0], data.get(
            "source", "unknown"
        )

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        """闭合双指并以目标和两侧手指的真实接触作为抓取确认。"""
        self.authority.validate(lease)
        target = self._manipulation["targets"].get(target_id)
        gripper = self._manipulation["gripper"]
        if target is None:
            raise ValueError("MuJoCo 场景中没有受控目标: " + str(target_id))
        if gripper is None:
            raise RuntimeError("MuJoCo Backend 未配置夹爪接触信息（来源：场景 report 的 gripper 段）")
        if grasp_pose.get("frame_id") != "world":
            raise ValueError("MuJoCo 抓取当前只接受 world 坐标系")

        target_body = self._body_id(target["body"])
        left_body = self._body_id(gripper["left_finger_body"])
        right_body = self._body_id(gripper["right_finger_body"])
        requested = grasp_pose["position"]
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            actual = tuple(float(value) for value in self.data.xpos[target_body])
        distance = math.sqrt(
            sum(
                (actual[index] - float(requested[key])) ** 2
                for index, key in enumerate(("x", "y", "z"))
            )
        )
        if distance > target["pose_tolerance_m"]:
            raise ValueError(
                "抓取位姿与目标位置不一致: "
                f"distance={distance:.6f}m tolerance={target['pose_tolerance_m']:.6f}m"
            )

        # 未知姿态支持：由目标姿态推出接近方向（= 顶面法向）。
        # 朝向未知时用单位四元数，此时接近方向退化为配置里的固定竖直方向。
        approach_axis, grasp_mode = self._resolve_grasp_axis(grasp_pose, gripper)

        duration_ms = max(1, int(duration_ms))
        open_ms = max(1, duration_ms * 2 // 5)
        close_ms = max(1, duration_ms - open_ms)
        lift_ms = 0
        if gripper.get("lift_positions"):
            lift_ms = max(1, duration_ms // 3)
            close_ms = max(1, duration_ms - open_ms - lift_ms)
        approach_positions = gripper.get("approach_positions")
        grasp_positions = gripper.get("grasp_positions")
        home_positions = gripper.get("home_positions")
        if home_positions:
            self._log_pick_phase("HOME_HOLD", target_body)
            self._move_trajectory(
                home_positions, max(1, duration_ms // 5), self._pick_ctrl_offsets("home")
            )
        if approach_positions:
            self._log_pick_phase("APPROACH", target_body)
            self._move_trajectory(
                approach_positions, max(1, duration_ms // 5),
                self._pick_ctrl_offsets("approach"),
            )
        if grasp_positions:
            self._log_pick_phase("DESCEND", target_body)
            self._move_trajectory(
                grasp_positions, max(1, duration_ms // 5),
                self._pick_ctrl_offsets("grasp"),
            )
        # 对齐门禁必须用目标实际姿态推出的接近轴换算抓取点：
        # 目标倾斜时仍按固定竖直轴减 pad_offset 会把抓取点算错半个高度。
        alignment = self._grasp_alignment_evidence(
            target_body, left_body, right_body, approach_axis
        )
        if alignment["center_distance_m"] > target["pose_tolerance_m"]:
            raise ValueError(
                "末端未到达目标抓取位姿: "
                f"distance={alignment['center_distance_m']:.6f}m "
                f"tolerance={target['pose_tolerance_m']:.6f}m "
                f"delta={alignment['center_delta_m']} "
                f"qpos={alignment['joint_qpos']}"
            )
        self._log_pick_phase("GRIP_OPEN", target_body)
        self._set_gripper_controls(gripper["open_positions"])
        self._advance_for(open_ms)
        self._log_pick_phase("GRIP_CLOSE", target_body)
        self._set_gripper_controls(gripper["closed_positions"])
        bilateral = self._advance_for(
            close_ms,
            contact_bodies=(target_body, left_body, right_body),
        )
        force_evidence = self._contact_force_evidence(
            target_body, left_body, right_body
        )
        force_ok = (
            bilateral
            and force_evidence["left_normal_force_n"]
            >= gripper["min_normal_force_n"]
            and force_evidence["right_normal_force_n"]
            >= gripper["min_normal_force_n"]
            and force_evidence["force_imbalance_ratio"]
            <= gripper["max_force_imbalance_ratio"]
        )
        if os.environ.get("IRAF_DEBUG_PICK") == "1":
            print("PICK_PRELIFT_EVIDENCE " + json.dumps({
                "target_position_m": [float(v) for v in self.data.xpos[target_body]],
                "left_finger_position_m": [float(v) for v in self.data.xpos[left_body]],
                "right_finger_position_m": [float(v) for v in self.data.xpos[right_body]],
                "alignment_grasp_point_m": alignment["grasp_point_position_m"],
                "alignment_center_delta_m": alignment["center_delta_m"],
                "alignment_center_distance_m": alignment["center_distance_m"],
                "bilateral_contact": bool(bilateral), **force_evidence,
            }, ensure_ascii=False), flush=True)
        constraint_activated = False
        equality_name = gripper.get("lift_constraint")
        if force_ok and equality_name:
            equality_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name
            )
            if equality_id < 0:
                raise ValueError("抓取约束不存在: " + str(equality_name))
            self.data.eq_active[equality_id] = 1
            constraint_activated = True
        with self._data_lock:
            before_lift_z = float(self.data.xpos[target_body][2])
        lifted = not lift_ms
        if lift_ms and force_ok:
            self._log_pick_phase("LIFT", target_body)
            self._move_trajectory(
                gripper["lift_positions"], lift_ms, self._pick_ctrl_offsets("lift")
            )
            if constraint_activated and gripper.get("lift_anchor_body"):
                self._advance_with_grasp_anchor(0, target_body, left_body, right_body, gripper["lift_anchor_body"])
            else:
                self._advance_for(0, contact_bodies=(target_body, left_body, right_body))
            with self._data_lock:
                after_lift_z = float(self.data.xpos[target_body][2])
            lifted = (
                after_lift_z - before_lift_z
                >= gripper["min_lift_delta_m"]
                and (
                    constraint_activated
                    or self._has_bilateral_contact(target_body, left_body, right_body)
                )
            )
        self.stopped = False
        if os.environ.get("IRAF_DEBUG_PICK") == "1":
            print(
                "PICK_CONTACT_EVIDENCE "
                + json.dumps(
                    {
                        "bilateral_contact": bool(bilateral),
                        "target_position_m": [float(v) for v in self.data.xpos[target_body]],
                        "left_finger_position_m": [float(v) for v in self.data.xpos[left_body]],
                        "right_finger_position_m": [float(v) for v in self.data.xpos[right_body]],
                        **force_evidence,
                        "force_ok": bool(force_ok),
                        "lifted": bool(lifted),
                        "lift_delta_m": round(
                            (float(self.data.xpos[target_body][2]) - before_lift_z), 6
                        ),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
        return {
            "target_id": target_id,
            "grasped": bool(force_ok and lifted),
            "confirmation": "constraint" if constraint_activated else "contact",
            "evidence": {
                "target_body": target["body"],
                "left_finger_body": gripper["left_finger_body"],
                "right_finger_body": gripper["right_finger_body"],
                "bilateral_contact": bool(bilateral),
                **force_evidence,
                "force_ok": bool(force_ok),
                "lifted": bool(lifted),
                "constraint_activated": constraint_activated,
                "lift_delta_m": round(
                    (float(self.data.xpos[target_body][2]) - before_lift_z), 6
                ),
                "grasp_mode": grasp_mode,
                "approach_axis": [round(float(value), 9) for value in approach_axis],
                "grasp_alignment": alignment,
            },
        }

    def _log_pick_phase(self, phase, target_body):
        if os.environ.get("IRAF_DEBUG_PICK") != "1":
            return
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            # 腕部 body 名必须由配置声明：不再回退到机型专有名（原先缺省是
            # Piper 的 link6），避免换构型后诊断日志抛"缺少 body: link6"。
            wrist = self._body_id(
                _declared_gripper_geometry(
                    self._manipulation.get("gripper") or {}
                )["wrist_body"]
            )
            print(
                "PICK_PHASE " + json.dumps({
                    "phase": phase,
                    "target_z_m": float(self.data.xpos[target_body][2]),
                    "wrist_z_m": float(self.data.xpos[wrist][2]),
                }, ensure_ascii=False),
                flush=True,
            )

    def _resolve_grasp_axis(self, grasp_pose, gripper):
        """由目标姿态推出接近轴，返回 (单位轴, grasp_mode)。

        - 姿态为单位四元数（朝向未知）：沿用配置里的 pad_offset_axis；
        - 姿态含实际朝向：接近轴 = 目标自身 z 轴（顶面法向），
          与配置竖直方向夹角超过 max_tilt_deg 时回退竖直抓取并在证据里标注，
          避免侧面进近把指尖压进工作台。
        """
        default_axis = self._unit_axis(gripper.get("pad_offset_axis"))
        orientation = grasp_pose.get("orientation") or {}
        w = float(orientation.get("w", 1.0))
        x = float(orientation.get("x", 0.0))
        y = float(orientation.get("y", 0.0))
        z = float(orientation.get("z", 0.0))
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-9:
            raise ValueError("抓取姿态四元数为零向量")
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        # 旋转矩阵第三列即目标自身 z 轴（顶面法向）。
        axis = np.array(
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
            dtype=float,
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-9:
            return default_axis, "fallback_vertical"
        axis = axis / axis_norm
        # 法向朝上统一，避免朝向定义差异导致门禁判反。
        if axis[2] < 0:
            axis = -axis
        max_tilt = float(gripper.get("max_tilt_deg", 30.0))
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2])))))
        if tilt_deg > max_tilt:
            return default_axis, "fallback_vertical"
        return axis, "pose_adaptive"

    @staticmethod
    def _unit_axis(raw_axis):
        values = list(raw_axis or (0.0, 0.0, 1.0))[:3]
        norm = math.sqrt(sum(float(value) ** 2 for value in values))
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        return np.array([float(value) / norm for value in values], dtype=float)

    def _require_geom_position(self, geom_id, geom_name):
        """取声明的 geom 世界位置；不存在即显式失败（不回退到 body 原点）。

        回退会让"配置里写错 geom 名"变成"门禁用了别的点"，表现为固定偏差的
        精度问题而非配置错误，正是本项目反复踩过的坑。
        """
        if int(geom_id) < 0:
            raise ValueError("夹爪声明的 geom 不存在: " + str(geom_name))
        return self.data.geom_xpos[int(geom_id)].copy()

    def _grasp_alignment_evidence(
        self, target_body, left_body, right_body, approach_axis=None
    ):
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            target = self.data.xpos[target_body].copy()
            # 接触面 geom 名必须由配置声明，**不提供机型默认值**：
            # 2F-85 的 pad body 原点在铰链处，与 pad box 中心相差约 2cm，
            # 回退到 body 位置会让对齐门禁把正确抓取误报成 94mm 偏差；
            # 而回退到某个机型的专有 geom 名又会在换构型后静默用错几何。
            # 缺失即由 `_declared_gripper_geometry` 显式失败。
            gripper_cfg = self._manipulation.get("gripper") or {}
            names = _declared_gripper_geometry(gripper_cfg)
            left_geom_name = names["left_finger_geom"]
            right_geom_name = names["right_finger_geom"]
            left_geom = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, str(left_geom_name)
            )
            right_geom = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, str(right_geom_name)
            )
            left = self._require_geom_position(left_geom, left_geom_name)
            right = self._require_geom_position(right_geom, right_geom_name)
            # **夹持区中点必须与 IK / 参考姿态证据同一口径**：
            # 求解器把"配置声明的全部 pad box 中点"对齐到目标点，门禁若改用
            # 左右单个 pad 的中点，两者会差一个固定几何量（UR5e + 2F-85 实测
            # 9.37mm，且随工具指向翻转而变号）。未声明 pad_boxes 时回退到
            # 左右代表接触面，保证 Piper 的既有行为逐位不变。
            grip_region = [str(name) for name in (gripper_cfg.get("pad_boxes") or ())]
            if grip_region:
                grip_positions = []
                for name in grip_region:
                    ident = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_GEOM, name
                    )
                    if ident < 0:
                        raise ValueError(
                            "夹持区声明的 geom 不存在: " + name
                        )
                    grip_positions.append(self.data.geom_xpos[ident].copy())
                grip_midpoint = sum(grip_positions) / float(len(grip_positions))
            else:
                grip_midpoint = None
        gripper = self._manipulation["gripper"]
        pad_offset = float(gripper.get("pad_offset_m", 0.0) or 0.0)
        if approach_axis is None:
            axis = self._unit_axis(gripper.get("pad_offset_axis"))
        else:
            axis = np.asarray(approach_axis, dtype=float)
            norm = float(np.linalg.norm(axis))
            axis = (
                self._unit_axis(gripper.get("pad_offset_axis"))
                if norm < 1e-9
                else axis / norm
            )
        # 夹持区中点：优先用配置声明的**夹持区**（与 IK / 参考姿态证据同一口径），
        # 未声明时才回退到左右代表接触面的中点（Piper 既有行为，逐位不变）。
        midpoint = grip_midpoint if grip_midpoint is not None else (left + right) / 2.0
        center = midpoint - axis * pad_offset
        delta = center - target
        return {
            "target_position_m": target.tolist(),
            "left_finger_position_m": left.tolist(),
            "right_finger_position_m": right.tolist(),
            "finger_center_position_m": midpoint.tolist(),
            "grasp_point_position_m": center.tolist(),
            # 抓取点口径留证：换夹爪后必须能一眼看出门禁用的是哪一种定义，
            # 否则"IK 用 4 点、门禁用 2 点"这类口径错位会以
            # "精度莫名不达标（固定偏差）"的形式反复出现。
            "grasp_point_source": (
                "grip_region" if grip_midpoint is not None else "finger_pair"
            ),
            "grip_region_geoms": grip_region,
            "pad_offset_m": pad_offset,
            "center_delta_m": delta.tolist(),
            "center_distance_m": float((delta @ delta) ** 0.5),
            "z_error_m": float(delta[2]),
            # 取证用的关节列表取自 profile 声明的 arm 角色关节，
            # 这样换构型后自检信息仍然可读（原实现写死 joint1..joint6）。
            "joint_qpos": {
                name: self._joint_qpos(name)
                for name in self._arm_joint_names()
            },
        }

    def _arm_joint_names(self):
        """返回 profile 声明为 arm 角色的关节名（按 profile.joints 顺序）。

        语义说明：这里返回的是**关节名**，而 `_joint_qpos` 按 actuator 名
        查 ctrl 通道；两者在 Piper 上同名，在 UR5 上不同名
        （joint: shoulder_pan_joint / actuator: shoulder_pan）。
        因此 `_joint_qpos` 需同时支持两种命名：先按 actuator 名查，
        查不到再按同名关节查 qpos。
        """
        roles = getattr(self.profile, "joint_roles", None) or {}
        names = [name for name in self.profile.joints if roles.get(name) == "arm"]
        return names or list(self.profile.joints)

    def _joint_qpos(self, name):
        """读取关节角：既接受执行器名，也接受关节名。

        - 执行器名：走 actuator_trnid 反查它驱动的关节；
        - 关节名：直接查 jnt_qposadr（UR5 的 actuator 名与关节名不同，
          只查 actuator 会全部返回 None，自检信息失去意义）。
        """
        actuator = self._actuators.get(self._model_name(name))
        if actuator is not None:
            joint_id = int(self.model.actuator_trnid[actuator, 0])
            return float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, self._model_name(name)
        )
        if joint_id < 0:
            return None
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        self._cancel_event.clear()
        with self._data_lock:
            for joint, value in positions.items():
                # 传入的键可能是**关节名**（profile.joints 口径）或**执行器名**
                # （场景 report 口径）。Piper 两者同名掩盖了这个差异，
                # UR5e 上 Home 动作因此报 "actuator not found: shoulder_pan_joint"。
                channel = self._actuator_channel(joint)
                self.data.ctrl[self._actuators[channel]] = float(value)
                # 状态按**关节名**回写，与 profile.joints / 返回值口径一致。
                self.last_positions[self._joint_name_of(joint)] = float(value)
            self.stopped = False
        if self._continuous_mode:
            deadline = time.monotonic() + max(1, int(duration_ms)) / 1000.0
            while not self._cancel_event.is_set() and time.monotonic() < deadline:
                time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
            return {
                joint: float(self.last_positions[joint])
                for joint in self.profile.joints
            }
        for _ in range(max(1, int(duration_ms / 10))):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            self.step()
        return {
            joint: float(self.last_positions[joint]) for joint in self.profile.joints
        }

    def stop(self, lease):
        self.authority.validate(lease)
        self._cancel_event.set()
        self._safe_stop_controls()

    def step(self, count=1):
        count = int(count)
        if count < 1:
            raise ValueError("MuJoCo 步进次数必须为正数")
        for _ in range(count):
            fault_kind, fault_delay = self._consume_fault()
            started = time.monotonic()
            if fault_kind == "step_failure":
                raise RuntimeError("注入的 MuJoCo 步进故障")
            if fault_kind == "step_delay":
                time.sleep(fault_delay)
            with self._data_lock:
                mujoco.mj_step(self.model, self.data)
            self._record_step(time.monotonic() - started)
            if self._realtime:
                time.sleep(float(self.model.opt.timestep))
        return {
            joint: float(self.last_positions[joint]) for joint in self.profile.joints
        }

    def start_continuous(self):
        """启动唯一物理步进线程；故障后必须显式清除故障才能重启。"""
        with self._loop_lock:
            if self._loop_thread is not None and self._loop_thread.is_alive():
                return False
            with self._metrics_lock:
                if self._last_error is not None:
                    raise RuntimeError("MuJoCo 步进异常尚未清除，拒绝重新启动")
                self._reset_metrics_locked()
            self._loop_stop.clear()
            self._continuous_mode = True
            self._loop_thread = threading.Thread(
                target=self._run_loop,
                name="mujoco-stepper",
                daemon=True,
            )
            self._loop_thread.start()
            return True

    def stop_continuous(self, timeout=2.0):
        with self._loop_lock:
            thread = self._loop_thread
            if thread is None:
                self._continuous_mode = False
                return False
            self._loop_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout)))
        with self._loop_lock:
            stopped = not thread.is_alive()
            if stopped:
                self._loop_thread = None
                self._continuous_mode = False
            return stopped

    def is_continuous(self):
        with self._loop_lock:
            return bool(
                self._continuous_mode
                and self._loop_thread
                and self._loop_thread.is_alive()
            )

    def simulation_status(self):
        """返回稳定、可序列化的仿真运行指标快照。"""
        running = self.is_continuous()
        now = time.monotonic()
        with self._metrics_lock:
            elapsed = max(0.0, now - self._loop_started_at)
            mean_duration = (
                self._total_step_duration_seconds / self._step_count
                if self._step_count
                else 0.0
            )
            effective_frequency = self._step_count / elapsed if elapsed > 0 else 0.0
            last_step_age = (
                max(0.0, now - self._last_step_at)
                if self._last_step_at is not None
                else None
            )
            return {
                "schema_version": "iraf.mujoco.simulation-status/v1",
                "simulation": True,
                "healthy": bool(running and self._last_error is None),
                "running": running,
                "target_frequency_hz": round(1.0 / self._target_period_seconds, 3),
                "effective_frequency_hz": round(effective_frequency, 3),
                "step_count": self._step_count,
                "step_overrun_count": self._step_overrun_count,
                "step_failure_count": self._step_failure_count,
                "last_step_duration_ms": round(
                    self._last_step_duration_seconds * 1000.0, 3
                ),
                "mean_step_duration_ms": round(mean_duration * 1000.0, 3),
                "max_step_duration_ms": round(
                    self._max_step_duration_seconds * 1000.0, 3
                ),
                "last_step_age_ms": (
                    round(last_step_age * 1000.0, 3)
                    if last_step_age is not None
                    else None
                ),
                "uptime_ms": round(elapsed * 1000.0, 3),
                "last_error": self._last_error,
                "fault_injection_enabled": self._fault_injection_enabled,
            }

    def inject_fault(self, kind, duration_ms=0, occurrences=1):
        """仅供显式启用的开发验收实例注入有界故障。"""
        if not self._fault_injection_enabled:
            raise PermissionError("当前 Backend 未启用仿真故障注入")
        if kind not in {"step_delay", "step_failure"}:
            raise ValueError("不支持的仿真故障类型: " + str(kind))
        occurrences = int(occurrences)
        if occurrences < 1:
            raise ValueError("故障注入次数必须为正数")
        delay_seconds = 0.0
        if kind == "step_delay":
            duration_ms = float(duration_ms)
            if duration_ms <= 0:
                raise ValueError("step_delay 的 duration_ms 必须为正数")
            delay_seconds = duration_ms / 1000.0
        with self._metrics_lock:
            if self._fault_remaining:
                raise RuntimeError("已有仿真故障等待执行")
            self._fault_kind = kind
            self._fault_delay_seconds = delay_seconds
            self._fault_remaining = occurrences
        return {
            "kind": kind,
            "duration_ms": round(delay_seconds * 1000.0, 3),
            "occurrences": occurrences,
        }

    def clear_faults(self):
        """停止后显式清除故障锁存和待执行注入。"""
        if not self._fault_injection_enabled:
            raise PermissionError("当前 Backend 未启用仿真故障注入")
        if self.is_continuous():
            raise RuntimeError("仿真运行期间禁止清除故障")
        with self._metrics_lock:
            self._fault_kind = None
            self._fault_delay_seconds = 0.0
            self._fault_remaining = 0
            self._last_error = None
        return True

    def render_frame(self, renderer):
        """渲染一致快照，不向外暴露 MuJoCo 控制状态。"""
        with self._data_lock:
            renderer.update_scene(self.data)
            return renderer.render()


    def _display_renderer(self):
        """惰性创建离屏渲染器，供显示降级路径使用。"""
        with self._data_lock:
            if self._display_renderer_cache is None:
                width, height = self._display_size
                self._display_renderer_cache = mujoco.Renderer(
                    self.model, height, width
                )
            return self._display_renderer_cache

    def render_frames(self, count=1):
        """渲染指定数量的 RGB 帧，返回 (H, W, 3) 数组列表。

        内部自行加锁，调用方不持有锁；渲染与物理步进解耦。
        """
        count = int(count)
        if count < 1:
            raise ValueError("render_frames 的 count 必须为正数")
        renderer = self._display_renderer()
        frames = []
        for _ in range(count):
            with self._data_lock:
                renderer.update_scene(self.data)
                frame = renderer.render()
            frames.append(np.array(frame))
        return frames

    def display_size(self):
        """返回离屏渲染分辨率 (width, height)。"""
        return self._display_size

    def home_pose(self):
        """返回 Home 位姿（执行器名 -> 目标角）。

        优先取 profile 的结构化 home 段；缺失时回退到 manipulation 的
        home_positions，保证既有配置行为不变。
        """
        structured = getattr(self.profile, "home", None)
        if structured:
            return {str(key): float(value) for key, value in dict(structured).items()}
        gripper = self._manipulation.get("gripper") or {}
        fallback = gripper.get("home_positions")
        if fallback:
            return dict(fallback)
        raise RuntimeError("profile 与 manipulation 均未声明 Home 位姿")

    def hold_current_pose(self):
        """把当前关节位置锁存为目标角，返回被锁存的位形。

        Runtime 收尾会清零控制量，显示窗口需要保持末态而不塌回零位。
        这里只操作公开可观测的关节目标，不暴露底层控制数组。
        """
        with self._data_lock:
            positions = {}
            for name in self.last_positions:
                qpos = self._joint_qpos(name)
                if qpos is None:
                    continue
                # `last_positions` 按**关节名**存放，而 ctrl 按**执行器名**索引
                # （UR5e 两者不同名）。腱驱动关节（如 2F-85 的 driver）没有
                # 可直接下发的通道，跳过而不是硬写关节角：
                # 它的位形由夹爪自己的开合指令决定，硬写会破坏欠驱动一致性。
                channel = self._direct_actuator_channel(name)
                if channel is None:
                    continue
                positions[channel] = qpos
            if not positions:
                raise RuntimeError("无法锁存当前位姿：没有可直接控制的关节通道")
            for channel, value in positions.items():
                self.data.ctrl[self._actuators[channel]] = float(value)
                self.last_positions[name] = float(value)
            self.stopped = False
        return positions

    def display_lock(self):
        """显示同步互斥锁。

        Viewer 的 sync() 会复制 mjData，必须与物理步进互斥，否则 MuJoCo
        会报 "copy mjData while stack is in use"。这里把锁作为公开契约暴露，
        使显示层无需访问私有成员即可正确加锁。
        """
        return self._data_lock

    def display_available(self):
        """报告离屏渲染通道是否可用（图形会话可用性由显示入口判定）。"""
        try:
            self._display_renderer()
            return True
        except Exception:
            return False

    def close_display(self):
        """释放离屏渲染器；可重复调用。"""
        with self._data_lock:
            if self._display_renderer_cache is not None:
                try:
                    self._display_renderer_cache.close()
                finally:
                    self._display_renderer_cache = None
        return True

    def _run_loop(self):
        period = self._target_period_seconds
        while not self._loop_stop.is_set():
            started = time.monotonic()
            try:
                self.step()
            except Exception as exc:
                self._safe_stop_controls()
                with self._metrics_lock:
                    self._step_failure_count += 1
                    self._last_error = f"{type(exc).__name__}: {exc}"
                self._loop_stop.set()
                break
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                self._loop_stop.wait(remaining)

    def _safe_stop_controls(self):
        with self._data_lock:
            self.data.ctrl[:] = 0.0
            self.stopped = True

    def _model_name(self, name):
        """把**声明名**解析为模型里的实际名字；未声明映射即原样返回（单本体路径行为不变）。

        解析点必须覆盖所有"调用方按声明名点对象"的入口，否则会出现
        "某条路径能动、另一条报找不到对象"（Piper 上同名恰好掩盖过同类差异）。
        """
        text = str(name)
        return self._name_map.get(text, text)

    def _declaration_name(self, name):
        """把**模型名**还原为声明名；无反向映射即原样返回（状态回写的键空间统一口径）。"""
        text = str(name)
        return self._declaration_by_model.get(text, text)

    def _direct_actuator_channel(self, name):
        """返回**直接**驱动该关节的执行器通道名；没有直接执行器时返回 None。

        与 `_actuator_channel` 的区别是语义而非实现：
        - 后者用于"必须能下发指令"的路径（找不到即显式失败）；
        - 本方法用于"能控制的才控制"的路径（如锁存当前位姿）。
          2F-85 的 driver 关节由 tendon 执行器经 equality + 耦合杆驱动，
          MuJoCo 里 tendon 执行器的 `trnid` 指向 **tendon 而非关节**，
          因此这些关节没有可直接写的 ctrl 通道 —— 按关节角硬写会破坏欠驱动一致性。

        关节名不存在时仍然显式失败（那是配置错误，不是"不可直接控制"）。
        """
        declared = str(name)
        # 声明名先解析到模型名：联合模型里臂的关节叫 `piper_joint1`，Profile 口径是 `joint1`
        name = self._model_name(declared)
        if name in self._actuators:
            return name
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(
                "找不到关节或执行器: " + declared
                + ("" if name == declared else "（经 name_map 解析为 %s）" % name))
        for index in range(int(self.model.nu)):
            if int(self.model.actuator_trnid[index, 0]) != int(joint_id):
                continue
            channel = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            )
            if channel:
                return str(channel)
        return None

    def _actuator_channel(self, name):
        """把**关节名**解析为驱动它的执行器通道名；已是执行器名则原样返回。

        为什么必须做：对外契约（`profile.joints`、move_joint 参数）用的是**关节名**，
        而 ctrl 数组按**执行器名**索引。两者在 Piper 上恰好同名
        （joint1..joint8），在 UR5e 上不同名（关节 shoulder_pan_joint /
        执行器 shoulder_pan）—— 早期只在场景生成器里做了翻译，
        `move_joint` 这条路径漏了，表现为 Piper 正常、
        UR5e 报 `actuator not found: shoulder_pan_joint`（实测 Home 动作失败）。

        解析失败即显式失败：静默忽略会让"动了但没动对关节"更难定位。
        """
        channel = self._direct_actuator_channel(name)
        if channel is None:
            raise ValueError(
                "关节没有可直接下发的执行器通道（可能是腱驱动）: %s" % name
            )
        return channel

    def _joint_name_of(self, name):
        """把执行器名解析为被驱动关节名；已是关节名则原样返回。

        对外契约统一到**关节名**口径（`profile.joints`、`move_joint` 返回值），
        因此内部按执行器通道写 ctrl 时，要把状态回写到关节名键上。
        """
        declared = str(name)
        model_name = self._model_name(declared)
        if model_name in self._actuators:
            joint_id = int(self.model.actuator_trnid[self._actuators[model_name], 0])
            joint = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            # 回写键统一到**声明名**（联合模型里模型名是 `piper_joint1`，而
            # `last_positions` / `profile.joints` 的口径是 `joint1`）
            return self._declaration_name(str(joint)) if joint else declared
        return declared

    def _set_controls(self, positions):
        with self._data_lock:
            for actuator, value in positions.items():
                channel = self._actuator_channel(actuator)
                self.data.ctrl[self._actuators[channel]] = float(value)

    def _set_gripper_controls(self, positions):
        """只更新夹爪通道，避免开合动作覆盖机械臂关节。

        泛化说明（原实现写死 joint7/joint8）：
        夹爪驱动通道的**名字由构型决定**——Piper 是 joint7/joint8 两个位置
        关节，Robotiq 2F-85 是单个 tendon 执行器 `rq2f85_fingers_actuator`。
        调用方传入的 `positions` 来自场景配置的 `gripper.open/closed`，
        本来就**只含夹爪通道**，因此直接使用即可，不能再按写死的名字过滤
        （否则 UR5 会被过滤成空集并误报"必须包含 joint7/joint8"）。
        """
        finger_positions = {
            str(name): float(value) for name, value in positions.items()
        }
        if not finger_positions:
            raise ValueError("夹爪开合配置不能为空")
        self._set_controls(finger_positions)

    def _pick_ctrl_offsets(self, phase):
        """取某抓取段的伺服前馈（ctrl 通道名 → 增量，rad）；未声明＝零前馈。

        数据来源是场景 report 的 `gripper.gravity_feedforward`，由参考姿态求解器
        按 model 的 qfrc_bias 与 gainprm[0] 算出（见
        `scripts/build_ur5_baseline._gravity_hold_ctrl`）。
        这是**构型无关的平台参数**：纯 PD 执行器需要它抵消稳态静差，
        真机或已做重力补偿的模型不声明即零前馈，行为与既有实现逐位一致。
        """
        gripper = self._manipulation.get("gripper") or {}
        offsets = (gripper.get("gravity_feedforward") or {}).get(str(phase))
        return dict(offsets) if offsets else {}

    def _move_trajectory(self, target_positions, duration_ms, ctrl_offsets=None):
        """以五次多项式从实际关节状态平滑移动到目标状态。

        `ctrl_offsets` 是逐段伺服前馈（ctrl 通道名 → 增量）：重力矩不为零的
        位形下纯 PD 执行器会停在 `ctrl - τ_g / gain`，必须把指令偏置
        `τ_g / gain` 加回去，才能让**稳态落点**等于目标关节角。
        含未在目标位形里声明的通道时显式失败（静默忽略会让前馈失效而难以察觉）。
        """
        # 目标位形与前馈键统一解析到**执行器通道名**空间：
        # 调用方可能给关节名（profile.joints 口径）或执行器名（场景 report 口径），
        # 两者必须能混用 —— 否则多机型下会出现"某条路径能动、另一条报找不到执行器"。
        resolved_targets = {
            self._actuator_channel(name): float(value)
            for name, value in target_positions.items()
        }
        names = list(resolved_targets)
        offsets = {
            self._actuator_channel(name): float(value)
            for name, value in (ctrl_offsets or {}).items()
        }
        unknown = sorted(set(offsets) - set(names))
        if unknown:
            raise ValueError("轨迹前馈含未声明的控制通道: " + str(unknown))
        commands = {
            name: resolved_targets[name] + offsets.get(name, 0.0) for name in names
        }
        with self._data_lock:
            starts = []
            for name in names:
                actuator = self._actuators.get(name)
                if actuator is None:
                    raise ValueError("actuator not found: " + name)
                joint_id = int(self.model.actuator_trnid[actuator, 0])
                starts.append(float(self.data.qpos[self.model.jnt_qposadr[joint_id]]))
        steps = max(1, int(math.ceil((int(duration_ms) / 1000.0) / self.model.opt.timestep)))
        for step in range(1, steps + 1):
            elapsed = step * float(self.model.opt.timestep)
            values = quintic_position(starts, [commands[name] for name in names], duration_ms / 1000.0, elapsed)
            self._set_controls(dict(zip(names, values)))
            self._advance_for(0)
        # 位置执行器有自身阻尼和力矩限制，轨迹结束后必须留出稳定时间。
        # 多目标场景中 joint1 要带着整臂绕基座旋转，其阻尼(300)远高于
        # 近端关节(2~100)，收敛时间按秒计：实测需要约 16 秒才能到目标角，
        # 而 4 秒时只走 61%、2 秒时只走 41%。
        # 注意这里不能按"段时长"缩放：调用方传入的是每段时长（总时长/5），
        # 按它缩放会把稳定窗口压到 4 秒以内，joint1 永远到不了位。
        settle_ms = max(250, min(16000, int(duration_ms) * 4))
        self._set_controls(commands)
        self._advance_for(settle_ms)

    def _advance_for(self, duration_ms, contact_bodies=None):
        bilateral = False
        deadline = time.monotonic() + max(1, int(duration_ms)) / 1000.0
        if self._continuous_mode:
            while not self._cancel_event.is_set() and time.monotonic() < deadline:
                if contact_bodies and self._has_bilateral_contact(*contact_bodies):
                    bilateral = True
                time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
            return bilateral

        steps = max(1, int(math.ceil((duration_ms / 1000.0) / self.model.opt.timestep)))
        for _ in range(steps):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            # 与 Viewer 的 Home 阶段使用同一条受锁保护的 MuJoCo 步进路径。
            with self._data_lock:
                mujoco.mj_step(self.model, self.data)
            self._record_step(float(self.model.opt.timestep))
            if self._realtime:
                time.sleep(float(self.model.opt.timestep))
            if contact_bodies and self._has_bilateral_contact(*contact_bodies):
                bilateral = True
        return bilateral

    def _advance_with_grasp_anchor(self, duration_ms, target_body, left_body, right_body, anchor_name):
        anchor_body = self._body_id(anchor_name)
        mocap_id = int(self.model.body_mocapid[anchor_body])
        if mocap_id < 0:
            raise ValueError("抓取中点必须是 mocap body: " + str(anchor_name))
        steps = max(1, int(math.ceil((duration_ms / 1000.0) / self.model.opt.timestep)))
        for _ in range(steps):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            with self._data_lock:
                self.data.mocap_pos[mocap_id] = (self.data.xpos[left_body] + self.data.xpos[right_body]) / 2.0
                self.data.mocap_quat[mocap_id] = (1.0, 0.0, 0.0, 0.0)
            self.step()

    def _has_bilateral_contact(self, target_body, left_body, right_body):
        contacted = set()
        with self._data_lock:
            for contact in self.data.contact[: self.data.ncon]:
                first = int(self.model.geom_bodyid[contact.geom1])
                second = int(self.model.geom_bodyid[contact.geom2])
                if first == target_body:
                    contacted.add(second)
                elif second == target_body:
                    contacted.add(first)
        return left_body in contacted and right_body in contacted

    def _contact_force_evidence(self, target_body, left_body, right_body):
        import numpy as np

        forces = {left_body: [], right_body: []}
        with self._data_lock:
            for index in range(self.data.ncon):
                contact = self.data.contact[index]
                bodies = (
                    int(self.model.geom_bodyid[contact.geom1]),
                    int(self.model.geom_bodyid[contact.geom2]),
                )
                finger = left_body if left_body in bodies else right_body if right_body in bodies else None
                if finger is None or target_body not in bodies:
                    continue
                result = np.zeros(6, dtype=float)
                mujoco.mj_contactForce(self.model, self.data, index, result)
                forces[finger].append(max(0.0, float(result[0])))
        left = max(forces[left_body] or [0.0])
        right = max(forces[right_body] or [0.0])
        low = min(left, right)
        ratio = max(left, right) / low if low > 1e-9 else float("inf")
        return {
            "left_normal_force_n": round(left, 6),
            "right_normal_force_n": round(right, 6),
            "force_imbalance_ratio": round(ratio, 6) if math.isfinite(ratio) else None,
        }

    def _body_id(self, name):
        declared = str(name)
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self._model_name(declared)
        )
        if body_id < 0:
            raise ValueError("body not found: " + declared)
        return int(body_id)

    @staticmethod
    def _parse_manipulation_config(config):
        if config is None:
            return {"targets": {}, "gripper": None}
        if not isinstance(config, dict):
            raise ValueError("manipulation 配置必须是对象")
        raw_targets = config.get("targets") or {}
        if not isinstance(raw_targets, dict):
            raise ValueError("manipulation.targets 必须是对象")
        targets = {}
        for target_id, item in raw_targets.items():
            if not isinstance(item, dict) or not item.get("body"):
                raise ValueError("每个 MuJoCo 目标必须声明 body")
            tolerance = float(item.get("pose_tolerance_m", 0.03))
            if not math.isfinite(tolerance) or tolerance <= 0:
                raise ValueError("目标 pose_tolerance_m 必须是正有限数")
            targets[str(target_id)] = {
                "body": str(item["body"]),
                "pose_tolerance_m": tolerance,
            }
        raw_gripper = config.get("gripper")
        if raw_gripper is None:
            gripper = None
        else:
            required = {
                "left_finger_body",
                "right_finger_body",
                "open_positions",
                "closed_positions",
            }
            # 必需字段：夹具几何相关的名字一律显式声明，禁止机型默认值。
            required = required | set(GRIPPER_GEOMETRY_FIELDS)
            missing = sorted(required - set(raw_gripper))
            if missing:
                raise ValueError("夹爪配置缺少字段: " + str(missing))
            open_positions = dict(raw_gripper["open_positions"])
            closed_positions = dict(raw_gripper["closed_positions"])
            if set(open_positions) != set(closed_positions) or not open_positions:
                raise ValueError("夹爪开合执行器配置必须一致且非空")
            gripper = {
                "left_finger_body": str(raw_gripper["left_finger_body"]),
                "right_finger_body": str(raw_gripper["right_finger_body"]),
                "open_positions": {
                    str(key): float(value) for key, value in open_positions.items()
                },
                "closed_positions": {
                    str(key): float(value) for key, value in closed_positions.items()
                },
            }
            for key in ("approach_positions", "grasp_positions"):
                if raw_gripper.get(key) is not None:
                    gripper[key] = {str(name): float(value) for name, value in dict(raw_gripper[key]).items()}
            if raw_gripper.get("home_positions") is not None:
                gripper["home_positions"] = {
                    str(name): float(value)
                    for name, value in dict(raw_gripper["home_positions"]).items()
                }
            if raw_gripper.get("lift_positions") is not None:
                lift_positions = dict(raw_gripper["lift_positions"])
                if not set(open_positions).issubset(lift_positions):
                    raise ValueError("抬升执行器配置必须包含夹爪执行器")
                min_lift = float(raw_gripper.get("min_lift_delta_m", 0.02))
                if not math.isfinite(min_lift) or min_lift <= 0:
                    raise ValueError("min_lift_delta_m 必须是正有限数")
                gripper["lift_positions"] = {
                    str(key): float(value) for key, value in lift_positions.items()
                }
                gripper["min_lift_delta_m"] = min_lift
                min_force = float(raw_gripper.get("min_normal_force_n", 0.2))
                max_ratio = float(raw_gripper.get("max_force_imbalance_ratio", 4.0))
                if not math.isfinite(min_force) or min_force <= 0:
                    raise ValueError("min_normal_force_n 必须是正有限数")
                if not math.isfinite(max_ratio) or max_ratio < 1:
                    raise ValueError("max_force_imbalance_ratio 必须不小于 1")
                gripper["min_normal_force_n"] = min_force
                gripper["max_force_imbalance_ratio"] = max_ratio
            else:
                gripper["lift_positions"] = None
                gripper["min_lift_delta_m"] = 0.02
            # 保证没有抬升动作的旧配置也能走统一的力闭环判定。
            gripper.setdefault("min_normal_force_n", 0.2)
            gripper.setdefault("max_force_imbalance_ratio", 4.0)
            # 伺服前馈（逐段 ctrl 增量）。**这是平台参数，不是机器人名**：
            # 纯 PD 执行器（如 Menagerie 的 UR5e，官方模型没有真机控制器自带的
            # 重力补偿）在重力矩不为零的位形下必然有稳态误差 Δq = τ_g / gain，
            # 实测 shoulder_lift 约 0.015 rad ≈ 末端 15mm，超过抓取容差。
            # 未声明 = 零前馈（真机或已做补偿的模型），行为与既有实现逐位一致。
            feedforward_raw = raw_gripper.get("gravity_feedforward")
            if feedforward_raw is None:
                gripper["gravity_feedforward"] = {}
            else:
                if not isinstance(feedforward_raw, dict):
                    raise ValueError("gravity_feedforward 必须是对象")
                phase_positions = {
                    "home": "home_positions",
                    "approach": "approach_positions",
                    "grasp": "grasp_positions",
                    "lift": "lift_positions",
                }
                unknown_phases = sorted(set(feedforward_raw) - set(phase_positions))
                if unknown_phases:
                    raise ValueError(
                        "gravity_feedforward 含未知段名: " + str(unknown_phases)
                    )
                feedforward = {}
                for phase, positions_key in phase_positions.items():
                    declared = feedforward_raw.get(phase)
                    if declared is None:
                        continue
                    if not isinstance(declared, dict):
                        raise ValueError(
                            "gravity_feedforward.%s 必须是对象" % phase
                        )
                    positions = gripper.get(positions_key)
                    if not positions:
                        raise ValueError(
                            "gravity_feedforward.%s 缺少对应的 %s 声明"
                            % (phase, positions_key)
                        )
                    unknown = sorted(set(declared) - set(positions))
                    if unknown:
                        raise ValueError(
                            "gravity_feedforward.%s 含未在该段位置指令中声明的"
                            "通道: %s" % (phase, unknown)
                        )
                    offsets = {}
                    for name, value in declared.items():
                        offset = float(value)
                        if not math.isfinite(offset):
                            raise ValueError(
                                "gravity_feedforward.%s.%s 必须是有限数"
                                % (phase, name)
                            )
                        # 前馈是"伺服静差补偿"，量级必须远小于关节行程。
                        # 上限用于拦截单位/符号写错（例如误填 N·m 或写反方向），
                        # 而不是用来限制正常取值（实测最大 0.017 rad）。
                        if abs(offset) > GRAVITY_FEEDFORWARD_LIMIT_RAD:
                            raise ValueError(
                                "gravity_feedforward.%s.%s=%.6f 超出上限 %.3f rad"
                                % (phase, name, offset, GRAVITY_FEEDFORWARD_LIMIT_RAD)
                            )
                        offsets[str(name)] = offset
                    feedforward[phase] = offsets
                gripper["gravity_feedforward"] = feedforward
            # 指腹接触区相对指尖 geom 中点的偏移，默认 0 保持既有配置行为。
            pad_offset = float(raw_gripper.get("pad_offset_m", 0.0) or 0.0)
            if not math.isfinite(pad_offset) or pad_offset < 0:
                raise ValueError("pad_offset_m 必须是非负有限数")
            gripper["pad_offset_m"] = pad_offset
            raw_axis = list(raw_gripper.get("pad_offset_axis") or (0.0, 0.0, 1.0))[:3]
            axis_values = [float(value) for value in raw_axis]
            if len(axis_values) != 3 or not all(
                math.isfinite(value) for value in axis_values
            ):
                raise ValueError("pad_offset_axis 必须是 3 个有限数")
            if math.sqrt(sum(value * value for value in axis_values)) < 1e-9:
                raise ValueError("pad_offset_axis 不能为零向量")
            gripper["pad_offset_axis"] = axis_values
            # 未知姿态支持：目标顶面法向与竖直方向夹角超过该阈值时
            # 回退竖直抓取，避免侧面进近把指尖压进工作台。
            max_tilt = float(raw_gripper.get("max_tilt_deg", 30.0))
            if not math.isfinite(max_tilt) or not 0.0 <= max_tilt <= 90.0:
                raise ValueError("max_tilt_deg 必须在 0..90 之间")
            gripper["max_tilt_deg"] = max_tilt
            constraint = raw_gripper.get("lift_constraint")
            if constraint is not None:
                if not isinstance(constraint, str) or not constraint:
                    raise ValueError("lift_constraint 必须是非空字符串")
                gripper["lift_constraint"] = constraint
            anchor_body = raw_gripper.get("lift_anchor_body")
            if anchor_body is not None:
                if not isinstance(anchor_body, str) or not anchor_body:
                    raise ValueError("lift_anchor_body 必须是非空字符串")
                gripper["lift_anchor_body"] = anchor_body
            # 构型相关的名字必须**原样保留**：本解析器只对已知字段做
            # 类型校验，但下面的字段是"由配置声明替代写死名字"的载体，
            # 不在这里透传就会在运行时被静默丢弃——
            # 实测表现为 UR5 场景下报 "body not found: link6"
            # （配置里明明写了 wrist_body=wrist_3_link）。
            for key in (
                "wrist_body",
                "left_finger_geom",
                "right_finger_geom",
            ):
                value = raw_gripper.get(key)
                if value is None:
                    continue
                if not isinstance(value, str) or not value:
                    raise ValueError(key + " 必须是非空字符串")
                gripper[key] = value
            # 夹持区定义（夹持区中点 = 这些 geom 中心的算术平均）。
            # **必须透传**：IK、参考姿态证据与运行时对齐门禁都按它定义"抓取点"，
            # 只让 left/right 两个代表接触面参与会让三方口径不一致
            # （UR5e + 2F-85 实测差 9.37mm，直接顶穿 5mm 容差）。
            # 未声明时保持 None，门禁回退到左右代表接触面（Piper 既有行为）。
            pad_boxes = raw_gripper.get("pad_boxes")
            if pad_boxes is not None:
                if not isinstance(pad_boxes, (list, tuple)) or not pad_boxes:
                    raise ValueError("pad_boxes 必须是非空的 geom 名列表")
                names = []
                for item in pad_boxes:
                    if not isinstance(item, str) or not item:
                        raise ValueError("pad_boxes 元素必须是非空字符串")
                    if item in names:
                        raise ValueError("pad_boxes 含重复 geom: " + item)
                    names.append(item)
                gripper["pad_boxes"] = names
        return {"targets": targets, "gripper": gripper}

    def _consume_fault(self):
        with self._metrics_lock:
            if self._fault_remaining < 1:
                return None, 0.0
            kind = self._fault_kind
            delay = self._fault_delay_seconds
            self._fault_remaining -= 1
            if self._fault_remaining == 0:
                self._fault_kind = None
                self._fault_delay_seconds = 0.0
            return kind, delay

    def _record_step(self, duration_seconds):
        with self._metrics_lock:
            self._step_count += 1
            self._total_step_duration_seconds += duration_seconds
            self._last_step_duration_seconds = duration_seconds
            self._max_step_duration_seconds = max(
                self._max_step_duration_seconds, duration_seconds
            )
            self._last_step_at = time.monotonic()
            if duration_seconds > self._target_period_seconds:
                self._step_overrun_count += 1

    def _reset_metrics(self):
        with self._metrics_lock:
            self._reset_metrics_locked()

    def _reset_metrics_locked(self):
        self._target_period_seconds = max(0.0001, float(self.model.opt.timestep))
        self._loop_started_at = time.monotonic()
        self._last_step_at = None
        self._step_count = 0
        self._step_overrun_count = 0
        self._step_failure_count = 0
        self._last_step_duration_seconds = 0.0
        self._total_step_duration_seconds = 0.0
        self._max_step_duration_seconds = 0.0
        self._last_error = None
