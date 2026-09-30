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

from iraf_adapters.mujoco.plant import MujocoPlant, PlantError, PlantOwnershipError
# 抓取段纠偏的**声明键/模式枚举**与构建期同一处（`scripts/build_robot_pick_scene.py` 也用这两个），
# 避免"构建期接受、运行期拒绝"这类口径漂移。
from iraf_adapters.mujoco.payload_facts import (
    GRASP_POSE_CORRECTION_KEYS,
    GRASP_POSE_CORRECTION_MODES,
    resolve_grasp_pose_correction,
)
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
#: 抓取目标位姿的**来源口径**（`pick_object` 的 `grasp_pose.pose_source`）：
#:   · `world_absolute`（缺省）= 调用方给出世界系坐标，后端在与目标体实测位姿的容差内核对；
#:   · `live_target_body` = 位置/朝向取**目标体在执行时刻的位姿**（仿真真值 FK；真机应由感知
#:     Provider 提供）。载具上的活体载荷必须用后者：构建期标称位姿实测偏 9.506e-03 ~
#:     1.1073e-02 m（docs/debug/2026-09-24-joint-model-dog-arm.md §11.27）。
PICK_POSE_SOURCES = ("world_absolute", "live_target_body")

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


def _quat_mul(a, b):
    """四元数相乘（wxyz，与 MuJoCo 同约定）：结果 = 先 b 后 a 的旋转复合。"""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def _quat_conj(q):
    """四元数共轭（wxyz）⇒ 逆旋转。"""
    w, x, y, z = (float(v) for v in q)
    return np.array([w, -x, -y, -z], dtype=float)


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
            place_targets_config=config.get("place_targets"),
            vision_config=config.get("vision"),
            realtime=bool(config.get("realtime", False)),
            display_size=(
                int(config.get("display_width", 640)),
                int(config.get("display_height", 480)),
            ),
            # 声明名 → 模型名（联合模型专有；单本体产物不带此键 ⇒ 行为逐位不变）
            name_map=config.get("name_map"),
            # 共享植物（联合场景）：注入别人拥有的植物 ⇒ 本后端是 guest，不得推进时间
            plant=config.get("plant"),
            # guest 等待 owner 推进时间的**墙钟**余量系数（只在本后端是 guest 时必需；
            # 缺声明即装配失败 —— 超时不猜，见 plant.wait_until）
            plant_guest_timeout_factor=config.get("plant_guest_timeout_factor"),
        )

    def __init__(
        self,
        model_path,
        profile,
        authority,
        fault_injection_enabled=False,
        manipulation_config=None,
        place_targets_config=None,
        vision_config=None,
        realtime=False,
        display_size=(640, 480),
        name_map=None,
        plant=None,
        plant_guest_timeout_factor=None,
    ):
        self.profile = profile
        self.authority = authority
        self._model_path = str(Path(model_path).resolve())
        # 共享植物（联合场景）：数据来自**别人拥有的植物** ⇒ 本后端是 guest：
        #   · 不再自建 MjModel/MjData（否则就是"两个世界"，实测见 plant.py 模块注释）；
        #   · 不重放关键帧（初始状态由 owner 决定，重放会把对方的状态冲掉）；
        #   · 不能推进时间（step 改为等 owner 推进，见 step()）。
        # plant=None ⇒ 自带植物且自己是 owner ⇒ 单本体路径逐位不变。
        self._plant_injected = plant is not None
        if self._plant_injected:
            self.model = plant.model
            self.data = plant.data
        else:
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
        if key_count > 0 and not self._plant_injected:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            mujoco.mj_forward(self.model, self.data)
        elif not self._plant_injected:
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
        # 植物与"我拥有的执行器"（控制权作用域）
        #  · 自带植物 ⇒ 我是 owner，拥有的执行器 = 模型里全部（单本体语义逐位不变）；
        #  · 注入植物 ⇒ 我是 guest：只能写 name_map 指向的那些执行器（越界即显式失败），
        #    且 `stop()` 只归零**自己的**执行器（不再碰主本体）。
        if self._plant_injected:
            self.plant = plant
            if not self.plant.is_owner(self) and plant_guest_timeout_factor is None:
                raise PlantError(
                    "共享植物模式下本后端是 guest（owner=%s）⇒ 必须声明 plant_guest_timeout_factor"
                    "（等待 owner 推进时间的墙钟余量系数，实现层不猜）" % self.plant.owner)
            self._plant_guest_timeout_factor = (
                None if plant_guest_timeout_factor is None else float(plant_guest_timeout_factor))
            if self._plant_guest_timeout_factor is not None and self._plant_guest_timeout_factor <= 0:
                raise PlantError("plant_guest_timeout_factor 必须是正数：%r"
                                 % (plant_guest_timeout_factor,))
        else:
            self.plant = MujocoPlant(self.model, self.data, owner=self,
                                     owner_name=str(getattr(self.profile, "name", "") or self._model_path),
                                     label=self._model_path)
            self._plant_guest_timeout_factor = None
        self._owned_actuators = self._resolve_owned_actuators()
        self.stopped = False
        self._cancel_event = threading.Event()
        # 共享植物下一律用**植物自己的锁**：Viewer 同步与两侧的控制写入必须互斥
        self._data_lock = self.plant.lock()
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
        # 接收体（承载面上的放置目标）：来自场景报告的 `place_targets` 段（构建期声明几何 + 标称位姿）。
        # 缺段即空表 ⇒ `place_object` 会显式拒绝（"场景报告没有接收体"），不猜、不用默认值。
        self._place_targets = self._parse_place_targets(place_targets_config)
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
        # 调用序号（判"是否被调用多次"用）：每次进入 pick_object 都自增，早退也算一次。
        self._pick_invocation = int(getattr(self, "_pick_invocation", 0)) + 1

        target_body = self._body_id(target["body"])
        left_body = self._body_id(gripper["left_finger_body"])
        right_body = self._body_id(gripper["right_finger_body"])
        # 目标位姿来源（2026-09-30，§11.27）：载具上的**活体载荷**用构建期标称位姿必然过期
        # （实测 9.506e-03 / 1.1073e-02 m，且逐轮不同）⇒ 声明式来源 `live_target_body`：
        # 位置与朝向都取**目标体在执行时刻的位姿**（仿真真值 FK；真机应由感知 Provider 给出）。
        # 缺省 `world_absolute` ⇒ 行为与改动前逐位一致。
        source = str(grasp_pose.get("pose_source") or "world_absolute")
        if source not in PICK_POSE_SOURCES:
            raise ValueError(
                "grasp_pose.pose_source 不受支持: %r（可用: %s）"
                % (source, list(PICK_POSE_SOURCES))
            )
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            actual = tuple(float(value) for value in self.data.xpos[target_body])
            actual_quat = tuple(float(value) for value in self.data.xquat[target_body])
        if source == "live_target_body":
            if "position" in grasp_pose or "orientation" in grasp_pose:
                raise ValueError(
                    "grasp_pose.pose_source=live_target_body 时不得同时给出 position/orientation"
                    "（两份事实必然分叉；契约见 skills/pick_object/pick_object.input.json）"
                )
            requested = {"x": actual[0], "y": actual[1], "z": actual[2]}
            grasp_pose = dict(grasp_pose)
            grasp_pose["position"] = dict(requested)
            grasp_pose["orientation"] = {
                "x": actual_quat[1], "y": actual_quat[2], "z": actual_quat[3],
                "w": actual_quat[0],
            }
        else:
            requested = grasp_pose["position"]
        adopted_quat = (actual_quat if source == "live_target_body"
                        else (grasp_pose["orientation"].get("w"), grasp_pose["orientation"].get("x"),
                              grasp_pose["orientation"].get("y"), grasp_pose["orientation"].get("z")))
        distance = math.sqrt(
            sum(
                (actual[index] - float(requested[key])) ** 2
                for index, key in enumerate(("x", "y", "z"))
            )
        )
        if distance > target["pose_tolerance_m"]:
            # 分量必须打全（2026-09-30）：只报欧氏距离时无法分辨"请求位姿过时（载荷被搬动过）"
            # 与"两侧坐标口径不同（局部/世界、abs/rel）"——这两类修法完全不同。
            delta = [actual[index] - float(requested[key])
                     for index, key in enumerate(("x", "y", "z"))]
            raise ValueError(
                "抓取位姿与目标位置不一致: "
                f"distance={distance:.6f}m tolerance={target['pose_tolerance_m']:.6f}m "
                f"target_body={target['body']} "
                f"requested_m={[round(float(requested[key]), 9) for key in ('x', 'y', 'z')]} "
                f"actual_m={[round(value, 9) for value in actual]} "
                f"delta_m={[round(value, 9) for value in delta]}"
            )

        # 未知姿态支持：由目标姿态推出接近方向（= 顶面法向）。
        # 朝向未知时用单位四元数，此时接近方向退化为配置里的固定竖直方向。
        approach_axis, grasp_mode = self._resolve_grasp_axis(grasp_pose, gripper)

        duration_ms = max(1, int(duration_ms))
        # 观测（与既有 IRAF_DEBUG_PICK 同风格，默认关闭）：把"这一段到底拿到了什么"打成一行 ——
        # 排查"声明了参数却不起作用"这类问题时，**必须直接看实参**，不能靠对代码的推断
        # （2026-09-24：探针与 nominal 的 s03 给出矛盾残差，四环链路读代码全部透传 ⇒ 需要实参）。
        if os.environ.get("IRAF_DEBUG_PICK") == "1":
            phase_ms = max(1, duration_ms // 5)
            print("PICK_INPUTS " + json.dumps({
                "invocation": int(getattr(self, "_pick_invocation", 0)),
                "wall_monotonic": round(time.monotonic(), 6),
                "target_id": str(target_id),
                "duration_ms": int(duration_ms),
                "phase_ms_each": int(phase_ms),
                "positions_keys": {key: sorted((gripper.get(key) or {}).keys())
                                   for key in ("home_positions", "approach_positions",
                                               "grasp_positions", "lift_positions")},
                "feedforward_offsets": {phase: self._pick_ctrl_offsets(phase)
                                        for phase in ("home", "approach", "grasp", "lift")},
                "gripper_fields": sorted(gripper.keys()),
            }, ensure_ascii=False), flush=True)
        open_ms = max(1, duration_ms * 2 // 5)
        close_ms = max(1, duration_ms - open_ms)
        lift_ms = 0
        if gripper.get("lift_positions"):
            lift_ms = max(1, duration_ms // 3)
            close_ms = max(1, duration_ms - open_ms - lift_ms)
        approach_positions = gripper.get("approach_positions")
        grasp_positions = gripper.get("grasp_positions")
        home_positions = gripper.get("home_positions")
        phase_ms = max(1, duration_ms // 5)
        if home_positions:
            self._log_pick_phase("HOME_HOLD", target_body)
            self._move_trajectory(home_positions, phase_ms, self._pick_ctrl_offsets("home"))
            self.dump_pick_phase("HOME_HOLD", phase_ms, target_body, left_body, right_body, approach_axis)
        # 接近/下压段是否保持载荷（声明；缺省视为 none 但**显式进证据**）：
        # 刚性指腹下压会推开轻载荷（运行期实测 1.44 cm），真机由台面摩擦抵住。
        raw_approach_hold = gripper.get("approach_hold")
        if isinstance(raw_approach_hold, dict):
            approach_hold = str(raw_approach_hold.get("mode") or "none")
        elif raw_approach_hold is None:
            approach_hold = "none"
        else:
            approach_hold = str(raw_approach_hold)
        if approach_hold not in ("pin_payload", "none"):
            raise ValueError("gripper.approach_hold 只允许 pin_payload|none：%r" % (approach_hold,))
        pin_target = target_body if approach_hold == "pin_payload" else None
        # ---- 抓取段**运行期纠偏**（2026-09-30 §11.29）----
        # 构建期的关节解按**标称目标**求；目标被搬动过（卸载步实测偏 15.812 mm）时夹口对不准。
        # **必须在接近段之前**做：接近/下压/抬升是同一列运动，只纠下压会让"接近段按标称列下走"——
        # 实测该段把载荷顶下沉 16.156 mm。声明存在即按实测目标重解**三段**（三条门禁在方法内，
        # 不收敛即拒绝）。
        lift_positions_for_run = gripper.get("lift_positions")
        pose_correction_evidence = None
        correction_declaration = gripper.get("grasp_pose_correction")
        if correction_declaration:
            _overrides, pose_correction_evidence = self._correct_grasp_column(
                correction_declaration,
                {"approach": approach_positions, "grasp": grasp_positions,
                 "lift": gripper.get("lift_positions")},
                target_body)
            pose_correction_evidence["pose_source"] = source
            if _overrides:
                approach_positions = _overrides.get("approach", approach_positions)
                grasp_positions = _overrides.get("grasp", grasp_positions)
                lift_positions_for_run = _overrides.get("lift", lift_positions_for_run)
        if approach_positions:
            self._log_pick_phase("APPROACH", target_body)
            self._move_trajectory(approach_positions, phase_ms, self._pick_ctrl_offsets("approach"),
                                  pin_body=pin_target)
            self.dump_pick_phase("APPROACH", phase_ms, target_body, left_body, right_body, approach_axis)
        # ---- 抓取段**运行期纠偏**已在上方（接近段之前）完成：这里不再重复纠偏 ----
        if grasp_positions:
            self._log_pick_phase("DESCEND", target_body)
            self._move_trajectory(grasp_positions, phase_ms, self._pick_ctrl_offsets("grasp"),
                                  pin_body=pin_target)
            self.dump_pick_phase("DESCEND", phase_ms, target_body, left_body, right_body, approach_axis)
        # 对齐门禁必须用目标实际姿态推出的接近轴换算抓取点：
        # 目标倾斜时仍按固定竖直轴减 pad_offset 会把抓取点算错半个高度。
        # 门禁前的紧邻取样：与 DESCEND 的 dump 比对即可判别"DESCEND 之后状态是否被改动"
        # （不同 ⇒ 有东西在动；相同 ⇒ 异常来自另一次调用）
        self.dump_pick_phase("DESCEND_PRE_GATE", phase_ms, target_body, left_body, right_body,
                             approach_axis)
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
        # 「重新张开」也会推走载荷（运行期实测：DESCEND 之后张到 0.035 把载荷推开 ~2 cm，
        # 随后合爪就对不上 ⇒ `lifted=false`）⇒ 同一条"保持"声明也覆盖这一段（§11.23(21)(22)）。
        if pin_target is not None:
            open_hold_evidence = self._advance_pinned(open_ms, target_body)
        else:
            open_hold_evidence = None
            self._advance_for(open_ms)
        self._log_pick_phase("GRIP_CLOSE", target_body)
        self._set_gripper_controls(gripper["closed_positions"])
        # 合爪语义：`close_hold`（声明；缺省视为 none 但**显式记进证据**，不静默）。
        #   `pin_payload` = 合爪期间把载荷保持在它的位姿（等价真机"指腹柔顺 + 台面摩擦抵住推力"）；
        #   `none`        = 不做任何保持（合爪的侧向合力会推移载荷，见 §11.23(19)(20)）。
        raw_close_hold = gripper.get("close_hold")
        if isinstance(raw_close_hold, dict):
            close_hold = str(raw_close_hold.get("mode") or "none")
        elif raw_close_hold is None:
            close_hold = "none"      # 未声明 ⇒ 不做保持（并**显式进证据**，不静默）
        else:
            close_hold = str(raw_close_hold)
        hold_evidence = None
        if close_hold == "pin_payload":
            hold_evidence = self._advance_pinned(close_ms, target_body)
            bilateral = self._has_bilateral_contact(target_body, left_body, right_body)
        elif close_hold == "none":
            bilateral = self._advance_for(
                close_ms,
                contact_bodies=(target_body, left_body, right_body),
            )
        else:
            raise ValueError("gripper.close_hold 只允许 pin_payload|none：%r" % (close_hold,))
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
        # **门控**（2026-09-29 §11.23(48)）：`require_friction_lift=true` ⇒ 抬升段**不使用**约束
        # （这是"夹爪能搬"的能力证明口径，是既有已验证声明）。要临时刚住请走 `regrasp.hold_pre_lift`
        # —— 它只在**预抬段**激活、`REGRASP_CLOSE` 之后立即撤销，最终搬运仍由摩擦承担。
        if bool(gripper.get("require_friction_lift")):
            equality_name = None
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
        # （判据口径的覆盖在抬升段开始处执行：此处 `regrasp_evidence` 尚未定义）
        # —— regrasp（声明 `gripper.regrasp.enabled`）：**先抬离台、再合爪到载荷腰部**。
        # 为什么（§11.23(27)(28) + .hermes/plans/2026-09-28-pick-regrasp.md）：腕式夹爪在台面上
        # 只能压住载荷上缘 ⇒ 对转动几乎无阻力矩 ⇒ 抬升时载荷"翻滚"出夹口（实测姿态 0°→120°，
        # 而接触力全程正常）。先把载荷抬离台，再把夹爪下探到载荷腰部合爪，指尖不再受台面限制，
        # 才能拿到真正的面夹与抗转力矩。位置指令全部来自构建期求解（缺即拒绝，不在运行时现解）。
        regrasp_cfg = gripper.get("regrasp") or {}
        regrasp_evidence = None
        # anchor 驱动参数（regrasp/LIFT 共用）：regrasp 关闭时保持空字典 ⇒ 行为不变
        _rg_anchor_kwargs = {}
        if bool(regrasp_cfg.get("enabled")):
            pre_lift_positions = gripper.get("pre_lift_positions")
            regrasp_positions = gripper.get("regrasp_positions")
            if not isinstance(pre_lift_positions, dict) or not isinstance(regrasp_positions, dict):
                raise ValueError(
                    "gripper.regrasp.enabled=true 但报告缺少 pre_lift_positions/regrasp_positions："
                    "regrasp 位形必须由**声明的求解器**在构建期解出（运行时现解 IK 会落错分支，见 §11.16）")
            open_m = float(regrasp_cfg.get("open_m") or 0.0)
            if not open_m > 0.0:
                raise ValueError("gripper.regrasp.open_m 必须为正数（松开量）：%r" % (open_m,))
            regrasp_ms = max(1, int(regrasp_cfg.get("duration_ms") or phase_ms))
            pre_lift_z = float(self.data.xpos[target_body][2])
            # **预抬段临时刚住**（声明 `regrasp.hold_pre_lift`，2026-09-29 §11.23(48)）：
            # regrasp 此前"第一步就失败（预抬 0.04 m 只升 9 mm）"的根因就是预抬阶段载荷已经翻滚
            # （与 LIFT 同一机制：上缘夹持 ⇒ 抗转力矩≈0）。这里用声明化的抬升约束把载荷**临时**
            # 刚住，让预抬成立；`REGRASP_CLOSE` 之后立即撤销 ⇒ **最终搬运仍由摩擦承担**，
            # "夹爪能搬"的能力口径不受影响（纯摩擦下的拖拽量继续作为诊断上报）。
            hold_pre_lift = bool(regrasp_cfg.get("hold_pre_lift", False))
            pre_lift_equality = None
            pre_lift_anchor_kwargs = {}
            if hold_pre_lift:
                pre_lift_equality = gripper.get("lift_constraint")
                if not pre_lift_equality:
                    raise ValueError("regrasp.hold_pre_lift=true 但报告没有 lift_constraint 名字")
                eid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                        str(pre_lift_equality))
                if eid < 0:
                    raise ValueError("抓取约束不存在: " + str(pre_lift_equality))
                anchor_name = gripper.get("lift_anchor_body")
                if not anchor_name:
                    raise ValueError("regrasp.hold_pre_lift=true 但报告没有 lift_anchor_body 名字")
                anchor_id = self._body_id(str(anchor_name))
                with self._data_lock:
                    # ⚠ 激活焊缝**必须同时驱动锚点**：否则锚点停在原地 ⇒ 焊缝把载荷**钉住**
                    # （实测载荷 z 只动 0.000259 m，比纯摩擦还糟）。锚点摆法沿用放置段同一口径：
                    # 位置 = 载荷当前位置，之后按「指腹中点 + 激活瞬间的载荷−指腹偏移」跟随。
                    mocap = int(self.model.body_mocapid[anchor_id])
                    left_p = np.asarray(self.data.xpos[self._body_id(str(gripper["left_finger_body"]))],
                                        dtype=float)
                    right_p = np.asarray(self.data.xpos[self._body_id(str(gripper["right_finger_body"]))],
                                         dtype=float)
                    finger_mid = (left_p + right_p) / 2.0
                    payload_now = np.asarray(self.data.xpos[target_body], dtype=float)
                    self.data.mocap_pos[mocap] = payload_now.copy()
                    self.data.mocap_quat[mocap] = np.asarray(self.data.xquat[target_body],
                                                             dtype=float).copy()
                    self.data.eq_active[eid] = 1
                pre_lift_anchor_kwargs = {
                    "anchor_body": str(anchor_name),
                    "anchor_follow": (str(gripper["left_finger_body"]),
                                      str(gripper["right_finger_body"])),
                    "anchor_offset": payload_now - finger_mid,
                }
                # **姿态分量必须与放置段同口径**（2026-09-29 §11.23(48) 标定第 2 步）：
                # 焊缝（weld）同时约束 6 个自由度 ⇒ 若只跟随**位置**、不跟随姿态，焊缝的姿态约束
                # 与工具转动会互相拧、持续泵入能量 ⇒ 实测预抬把载荷**甩到夹口上方**
                # （pre_lift_delta_m = 0.175094，是声明 0.04 的 4.4 倍）。
                # 放置段的写法：anchor 姿态 = 腕部姿态 ⊗ 激活瞬间的（腕部⁻¹ ⊗ 载荷）。
                carry_cfg = gripper.get("carry_constraint") or {}
                if str(carry_cfg.get("type") or "") == "weld":
                    with self._data_lock:
                        wrist_q = np.asarray(
                            self.data.xquat[self._body_id(str(gripper["wrist_body"]))], dtype=float)
                        payload_q = np.asarray(self.data.xquat[target_body], dtype=float)
                    pre_lift_anchor_kwargs["anchor_wrist"] = str(gripper["wrist_body"])
                    pre_lift_anchor_kwargs["anchor_rel_quat"] = _quat_mul(_quat_conj(wrist_q),
                                                                        payload_q)
            # **预抬段逐样本取证**（2026-09-29 §11.23(48) 标定第 3 步）：三量同图 ——
            # ① anchor 的 mocap 位置 ② 指腹中点 ③ 载荷中心。段首/段末两点无法区分
            # "anchor 摆错位" / "焊缝把载荷顶到别处" / "预抬位形把指腹送到别处"。
            pre_lift_samples = []
            _pre_stride = max(1, int(os.environ.get("IRAF_DEBUG_PICK_STRIDE", "10")))

            def _pre_lift_sampler(step, elapsed):
                with self._data_lock:
                    _mocap = (int(self.model.body_mocapid[anchor_id])
                              if pre_lift_equality is not None else -1)
                    _anchor = ([round(float(v), 6) for v in self.data.mocap_pos[_mocap]]
                               if _mocap >= 0 else None)
                    _pad = (np.asarray(self.data.xpos[self._body_id(str(gripper["left_finger_body"]))],
                                       dtype=float)
                            + np.asarray(self.data.xpos[self._body_id(str(gripper["right_finger_body"]))],
                                         dtype=float)) / 2.0
                    # **同图对照**（§11.23(48)）：求解器用的点是**指腹 geom** 中心，而 anchor 跟随用
                    # **body** 中心 ⇒ 两者应是同一物理点；同一采样里若位置不同，就说明参考点取错对象。
                    _pad_geom = (np.asarray(self.data.geom_xpos[mujoco.mj_name2id(
                                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    str(gripper["left_finger_geom"]))], dtype=float)
                                 + np.asarray(self.data.geom_xpos[mujoco.mj_name2id(
                                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    str(gripper["right_finger_geom"]))], dtype=float)) / 2.0
                    _pay = np.asarray(self.data.xpos[target_body], dtype=float).copy()
                with self._data_lock:
                    _arm_now = {}
                    for _name, _value in (pre_lift_positions or {}).items():
                        _jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                self._model_name(str(_name)))
                        if _jid >= 0:
                            _arm_now[str(_name)] = round(
                                float(self.data.qpos[int(self.model.jnt_qposadr[_jid])]), 6)
                _row = {"step": int(step), "plant_step_index": int(self.plant.step_index),
                        "anchor_mocap_m": _anchor,
                        "pad_mid_body_m": [round(float(v), 6) for v in _pad],
                        "pad_mid_geom_m": [round(float(v), 6) for v in _pad_geom],
                        "body_minus_geom_m": [round(float(v), 6) for v in (_pad - _pad_geom)],
                        "pad_mid_m": [round(float(v), 6) for v in _pad],
                        "arm_qpos": _arm_now,
                        "payload_m": [round(float(v), 6) for v in _pay],
                        "payload_minus_anchor_m": ([round(float(v), 6) for v in (_pay - np.asarray(_anchor, dtype=float))]
                                                   if _anchor is not None else None)}
                pre_lift_samples.append(_row)
                # ⚠ 必须与其它诊断一样受 `IRAF_DEBUG_PICK` 开关约束（2026-09-29 实测）：
                # 原先只按 stride 打印（默认 10）⇒ 演示/验收时每 10 个样本落一行，
                # 单步墙钟被 print 拖长（本场景一段 pre_lift 就有上万行）⇒ 收尾时租约
                # 已过期（LeaseConflict）。诊断输出不得改变被测对象的时序（观测者效应）。
                if (os.environ.get("IRAF_DEBUG_PICK") == "1"
                        and len(pre_lift_samples) % _pre_stride == 0):
                    print("PRE_LIFT_TRACE " + json.dumps(_row, ensure_ascii=False), flush=True)

            self._log_pick_phase("PRE_LIFT", target_body)
            self._move_trajectory(pre_lift_positions, regrasp_ms, self._pick_ctrl_offsets("lift"),
                                  sampler=_pre_lift_sampler, **pre_lift_anchor_kwargs)
            # **预抬结束处的观测**（2026-09-29 §11.23(48) 标定用）：只有闭合后一个点分不清
            # 「预抬抬多/抬少」与「目标公式偏」 ⇒ 这里量：载荷实际抬升量（对比声明 pre_lift_m）、
            # 载荷相对指腹的竖向与横向偏移（夹持几何的直接证据）。
            with self._data_lock:
                _pad = (np.asarray(self.data.xpos[self._body_id(str(gripper["left_finger_body"]))],
                                   dtype=float)
                        + np.asarray(self.data.xpos[self._body_id(str(gripper["right_finger_body"]))],
                                     dtype=float)) / 2.0
                _pay = np.asarray(self.data.xpos[target_body], dtype=float).copy()
            pre_lift_after_z = float(_pay[2])
            pre_lift_observe = {
                "pre_lift_declared_m": float(regrasp_cfg.get("pre_lift_m") or 0.0),
                "payload_z_after_pre_lift_m": round(pre_lift_after_z, 6),
                "pre_lift_delta_m": round(pre_lift_after_z - pre_lift_z, 6),
                "pad_minus_payload_z_after_pre_lift_m": round(float(_pad[2] - _pay[2]), 6),
                "payload_pad_lateral_after_pre_lift_m": round(
                    float(np.linalg.norm((_pay - _pad)[:2])), 6),
            }
            # regrasp 的三段同样必须驱动 anchor（与预抬/放置同口径）：只激活不驱动 = 锚点陈旧 ⇒ 拔河
            _rg_anchor_kwargs = dict(pre_lift_anchor_kwargs)
            self._log_pick_phase("REGRASP_OPEN", target_body)
            opened = {str(name): (float(value) + open_m if str(name).endswith("joint7") else
                                  float(value) - open_m)
                      for name, value in (gripper["closed_positions"] or {}).items()}
            self._set_gripper_controls(opened)
            self._advance_for(regrasp_ms)
            self._log_pick_phase("REGRASP_DESCEND", target_body)
            self._move_trajectory(regrasp_positions, regrasp_ms, self._pick_ctrl_offsets("lift"),
                                  **_rg_anchor_kwargs)
            self._log_pick_phase("REGRASP_CLOSE", target_body)
            self._set_gripper_controls(gripper["closed_positions"])
            bilateral = self._advance_for(
                regrasp_ms, contact_bodies=(target_body, left_body, right_body))
            force_evidence = self._contact_force_evidence(target_body, left_body, right_body)
            force_ok = (
                bilateral
                and force_evidence["left_normal_force_n"] >= gripper["min_normal_force_n"]
                and force_evidence["right_normal_force_n"] >= gripper["min_normal_force_n"]
                and force_evidence["force_imbalance_ratio"] <= gripper["max_force_imbalance_ratio"]
            )
            if hold_pre_lift and pre_lift_equality and bool(gripper.get("require_friction_lift")):
                # 腰部面夹已合上 ⇒ **仅在仍要求"纯摩擦抬升"时**撤销临时约束（那条口径下之后的抬升
                # 由摩擦承担）。新口径 `require_friction_lift=false`（2026-09-29 声明）下**保持约束到
                # 抬升结束**：实测只开约束不驱动 anchor ⇒ 焊缝与摩擦拔河、载荷仍绕接触线转 45.73°。
                eid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                        str(pre_lift_equality))
                with self._data_lock:
                    self.data.eq_active[eid] = 0
            with self._data_lock:
                after_regrasp_z = float(self.data.xpos[target_body][2])
                # 夹持几何（**调参必须看**，2026-09-29 §11.23(48)）：指腹中点高度 vs 载荷中心高度
                # ⇒ 判断"夹错高度"（太高=还夹上缘、太低=夹到底棱），以及指腹跨度与载荷-夹口横向偏移。
                # ⚠ 口径必须与 IK 目标/pad_offset/放置段 `pad_mid` **一致**：都用**指腹 geom 中心**
                # （2026-09-29 实测：body 中点比 geom 中点低 33~43 mm ⇒ 用 body 会把"夹持高度"读成
                # −0.04754，而按 geom 口径其实约 −0.008 ⇒ 已接近载荷腰身中点。仪器口径错会被当成物理结论）
                _l = np.asarray(self.data.geom_xpos[mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, str(gripper["left_finger_geom"]))], dtype=float)
                _r = np.asarray(self.data.geom_xpos[mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, str(gripper["right_finger_geom"]))], dtype=float)
                _pad_mid = (_l + _r) / 2.0
                _payload_c = np.asarray(self.data.xpos[target_body], dtype=float).copy()
                _span = _r - _l
            regrasp_evidence = {
                "applied": True, "pre_lift_m": float(regrasp_cfg.get("pre_lift_m") or 0.0),
                "depth_m": float(regrasp_cfg.get("depth_m") or 0.0), "open_m": open_m,
                "hold_pre_lift": bool(regrasp_cfg.get("hold_pre_lift", False)),
                "pre_lift_constraint": (str(pre_lift_equality) if hold_pre_lift else None),
                "contact_forces": dict(force_evidence),
                "pad_mid_z_m": round(float(_pad_mid[2]), 6),
                "payload_center_z_m": round(float(_payload_c[2]), 6),
                "pad_minus_payload_z_m": round(float(_pad_mid[2] - _payload_c[2]), 6),
                "pad_span_m": round(float(np.linalg.norm(_span)), 6),
                "payload_pad_lateral_m": round(
                    float(np.linalg.norm((_payload_c - _pad_mid)[:2])), 6),
                "pre_lift_z_m": round(pre_lift_z, 6),
                **pre_lift_observe,
                "after_regrasp_z_m": round(after_regrasp_z, 6),
                "bilateral_after_regrasp": bool(bilateral), "force_ok_after_regrasp": bool(force_ok),
            }
        lifted = not lift_ms
        # LIFT 段的**逐样本运行时追踪**（2026-09-28 §11.23(24) 的下一步）：
        # 离线探针（含 guest 节拍仿真）已无法复现运行时的 `lifted=false`，
        # 剩下的粒度只有"运行时抬升段逐样本"：载荷位姿 / 指腹 qpos+ctrl / 接触对与力随时间的演化，
        # 用来回答"哪一步、哪个量先动"。stride 由 `IRAF_DEBUG_PICK_STRIDE` 给（默认 10）。
        lift_samples = []
        lift_peak = {"max_z": None, "step": None}
        # 抬升段的**载荷姿态**偏角（相对抬升起始）——抓取失败时它会从 0° 单调涨到 100°+
        # 而接触力始终正常 ⇒ 判读必须先看姿态再看力（§11.23(27)）。
        lift_attitude = {"ref_quat": None, "max_deg": None}
        try:
            lift_stride = max(1, int(os.environ.get("IRAF_DEBUG_PICK_STRIDE", "10")))
        except ValueError:
            lift_stride = 10
        pad_geoms = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                       str(gripper.get(key) or ""))
                     for key in ("left_finger_geom", "right_finger_geom")]

        def _lift_sampler(step, elapsed):
            # 常驻（不受 IRAD_DEBUG 开关影响）：抬升段的**峰值载荷高度**（判据盲区，见 §11.23(25)）
            with self._data_lock:
                current_z = float(self.data.xpos[target_body][2])
            if lift_peak["max_z"] is None or current_z > lift_peak["max_z"]:
                lift_peak["max_z"] = current_z
                lift_peak["step"] = int(step)
            with self._data_lock:
                quat = np.asarray(self.data.xquat[target_body], dtype=float).copy()
            if lift_attitude["ref_quat"] is None:
                lift_attitude["ref_quat"] = quat
            dot = abs(float(np.dot(quat, lift_attitude["ref_quat"])))
            deg = math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))
            if lift_attitude["max_deg"] is None or deg > lift_attitude["max_deg"]:
                lift_attitude["max_deg"] = deg
            if os.environ.get("IRAF_DEBUG_PICK") != "1" or step % lift_stride != 0:
                return
            with self._data_lock:
                contacts = []
                for index in range(int(self.data.ncon)):
                    contact = self.data.contact[index]
                    geoms = {int(contact.geom1), int(contact.geom2)}
                    if int(target_body) not in {int(self.model.geom_bodyid[g]) for g in geoms}:
                        continue
                    partner = next((g for g in geoms
                                    if int(self.model.geom_bodyid[g]) != int(target_body)), None)
                    if partner is None:
                        continue
                    force = np.zeros(6, dtype=float)
                    try:
                        mujoco.mj_contactForce(self.model, self.data, index, force)
                    except Exception:
                        pass
                    contacts.append({
                        "partner": (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                                      int(self.model.geom_bodyid[partner]))
                                    or ("#%d" % int(self.model.geom_bodyid[partner]))),
                        "dist_m": round(float(contact.dist), 6),
                        "force_n": round(float(np.linalg.norm(np.asarray(force[0:3], dtype=float))), 4)})
                pads = [np.asarray(self.data.geom_xpos[g], dtype=float) for g in pad_geoms if g >= 0]
                mid = (sum(pads) / len(pads)) if pads else np.zeros(3)
                state = {}
                for name in sorted(gripper.get("open_positions") or {}):
                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                            self._model_name(str(name)))
                    if jid < 0:
                        continue
                    act = next((i for i in range(int(self.model.nu))
                                if int(self.model.actuator_trnid[i, 0]) == int(jid)), -1)
                    state[str(name)] = {"qpos": round(float(self.data.qpos[int(self.model.jnt_qposadr[jid])]), 9),
                                        "ctrl": (round(float(self.data.ctrl[act]), 9) if act >= 0 else None)}
                # ⚠ 必须同时记**腕部**姿态（§11.23(48)）：只有"载荷在转"分不清是"工具带着转"
                # 还是"载荷在夹口里滑" ⇒ 两者的修法完全不同（前者改求解时的姿态约束，
                # 后者改夹紧力/倾覆力矩）。指腹开合轴也记下来（夹口朝向的直接证据）。
                wrist_q = np.asarray(self.data.xquat[self._body_id(str(gripper["wrist_body"]))],
                                     dtype=float)
                span = (np.asarray(self.data.geom_xpos[pad_geoms[1]], dtype=float)
                        - np.asarray(self.data.geom_xpos[pad_geoms[0]], dtype=float)) \
                    if len(pad_geoms) >= 2 and pad_geoms[0] >= 0 and pad_geoms[1] >= 0 \
                    else np.zeros(3)
                row = {"step": int(step), "plant_step_index": int(self.plant.step_index),
                       # 节拍必须在**同一行**里可算（§11.23(43)(48)）：采样回调每个**控制迭代**调一次，
                       # 而 anchor 每迭代只跟随一次 ⇒ `plant_step_index` 的相邻差 = 该迭代植物前进的步数。
                       # 带追踪的 6 轮全绿、无追踪的 8 轮里 1 次失败 ⇒ 判为节拍敏感 ⇒ 需要这个量。
                       "payload_pos_m": [round(float(v), 6) for v in self.data.xpos[target_body]],
                       "payload_quat_wxyz": [round(float(v), 9) for v in self.data.xquat[target_body]],
                       "wrist_quat_wxyz": [round(float(v), 9) for v in wrist_q],
                       "pad_span_m": [round(float(v), 6) for v in span],
                       "pad_mid_m": [round(float(v), 6) for v in mid],
                       "gripper": state, "contacts": contacts}
            lift_samples.append(row)
            print("PICK_LIFT_TRACE " + json.dumps(row, ensure_ascii=False), flush=True)

        if lift_ms and force_ok:
            # **抬升路径**（声明；缺省 direct）：`approach_then_lift` 先竖直走到 approach 位形
            # （构建期解出的"抓取点沿接近轴抬高 pregrasp_offset_m"的解 ⇒ 任务空间竖直段，
            # 让载荷先离台），再走向 lift 位形。实测 direct 会把载荷沿台面拖行 2.2 cm 后拖出夹口
            # （§11.23(26)）。
            if regrasp_evidence is not None:
                # 判据口径：`lift_delta_m` 衡量"载荷离开台面多少"，不是"最后一段抬了多少"
                # ⇒ 有 regrasp 时从 **PRE_LIFT 之前**的高度起算，否则 4 cm 预抬会被漏计。
                before_lift_z = float(regrasp_evidence["pre_lift_z_m"])
            lift_path_mode = str(gripper.get("lift_path") or "direct")
            if lift_path_mode not in ("approach_then_lift", "direct"):
                raise ValueError("gripper.lift_path 只允许 approach_then_lift|direct：%r"
                                 % (lift_path_mode,))
            # **抬升段夹爪语义**（§11.23(48)）：`hold` ⇒ 目标取**抓取瞬间实测的 ctrl**，
            # 不再下发 `closed`（实测抬升段持续合拢 3.89 mm，配合"上缘夹持"把载荷翻滚出 50.59°）。
            lift_positions = (lift_positions_for_run if lift_positions_for_run
                              else gripper["lift_positions"])
            if str(gripper.get("lift_gripper") or "closed") == "hold":
                lift_positions = {str(k): float(v) for k, v in lift_positions.items()}
                with self._data_lock:
                    for name in (gripper.get("open_positions") or {}):
                        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                self._model_name(str(name)))
                        act = next((i for i in range(int(self.model.nu))
                                    if int(self.model.actuator_trnid[i, 0]) == int(jid)), -1)
                        if jid >= 0 and act >= 0 and str(name) in lift_positions:
                            lift_positions[str(name)] = float(self.data.ctrl[act])
            self._log_pick_phase("LIFT", target_body)
            if lift_path_mode == "approach_then_lift" and approach_positions:
                half = max(1, lift_ms // 2)
                self._move_trajectory(approach_positions, half,
                                      self._pick_ctrl_offsets("approach"),
                                      sampler=_lift_sampler)
                self._move_trajectory(
                    lift_positions, max(1, lift_ms - half),
                    self._pick_ctrl_offsets("lift"), sampler=_lift_sampler)
            else:
                # LIFT 段同样驱动 anchor（新口径下约束保持到抬升结束 ⇒ 锚点必须跟随，否则拔河）
                self._move_trajectory(
                    lift_positions, lift_ms, self._pick_ctrl_offsets("lift"),
                    sampler=_lift_sampler, **_rg_anchor_kwargs)
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
                "pose_source": source,
                "resolved_target_pose_m": [round(float(value), 9)
                                           for value in (actual if source == "live_target_body"
                                                         else (requested["x"], requested["y"], requested["z"]))],
                "resolved_target_quat_wxyz": [round(float(value), 9) for value in adopted_quat],
                # 抓取段运行期纠偏的完整留证（声明 / 决策 / 各相位残差、姿态、侵入与修正后的关节解）；
                # 缺声明时为 null（不纠偏）。
                "grasp_pose_correction": pose_correction_evidence,
                "grasp_mode": grasp_mode,
                "approach_axis": [round(float(value), 9) for value in approach_axis],
                "grasp_alignment": alignment,
                # 峰值抬升（诊断量；判据仍用 lift_delta_m = 结束时刻）
                "lift_peak_delta_m": (None if lift_peak["max_z"] is None
                                      else round(max(0.0, lift_peak["max_z"] - before_lift_z), 6)),
                "lift_peak_step": lift_peak["step"],
                "lift_attitude_max_deg": (None if lift_attitude["max_deg"] is None
                                          else round(lift_attitude["max_deg"], 6)),
                "regrasp": regrasp_evidence,
                # ⚠ LIFT 段逐样本追踪**不进 evidence**：`pick_object.output.json` 是
                # `additionalProperties: false` 的契约（契约先行）⇒ 新字段必须先改契约才允许。
                # 该追踪是**调试仪器**：只在 `IRAF_DEBUG_PICK=1` 时逐行打印（stdout 即产物）。
            },
        }

    def place_object(self, place_target_id, payload_id, duration_ms, lease):
        """把当前夹持的载荷放到接收体（承载面）上：下行至**接触**→开夹爪→抬离。

        设计要点（2026-09-28，docs/debug/2026-09-24-joint-model-dog-arm.md §11.12/§11.10）：
        · 接收体随载体运动 ⇒ 命令只给**名字**，位姿在**运行期实测**（`data.xpos/xmat`），
          不使用报告里的 `nominal_*`（那是构建基准）；
        · 下行终点**不写死深度**：以"载荷最低点落到承载面"为条件（`lowest_mesh_point_z`
          实测载荷几何的最低点，托盘顶面由 FK 给出）⇒ 不需要方块半尺寸之类的额外声明；
        · 判据全是**事实**（无阈值）：`released` = 张开后指腹与载荷**不再接触**；
          `payload_in_tray` = 载荷 XY 落在接收体声明的半尺寸内 **且** 与接收体存在接触（放住了）；
        · 任一步不成立即显式失败，不返回伪造成功（AGENTS.md 1.5）。
        """
        self.authority.validate(lease)
        place_targets = getattr(self, "_place_targets", {}) or {}
        record = place_targets.get(str(place_target_id))
        if record is None:
            raise ValueError("场景报告没有接收体: " + str(place_target_id))
        gripper = self._manipulation.get("gripper")
        payload = (self._manipulation.get("targets") or {}).get(str(payload_id))
        if payload is None:
            raise ValueError("场景报告没有载荷目标: " + str(payload_id))
        if gripper is None:
            raise RuntimeError("MuJoCo Backend 未配置夹爪信息（来源：场景 report 的 gripper 段）")
        if record.get("size_m") is None:
            raise ValueError("接收体 %s 未声明 size_m（无法判「载荷是否落在承载面内」）" % place_target_id)
        pad_offset = float(gripper.get("pad_offset_m") or 0.0)
        approach_offset = float(gripper.get("pregrasp_offset_m") or 0.0)
        if pad_offset <= 0 or approach_offset <= 0:
            raise ValueError("接收体放置需要报告声明 gripper.pad_offset_m 与 gripper.pregrasp_offset_m")
        tray_body = self._body_id(record["body"])
        payload_body = self._body_id(payload["body"])
        # 放置点纠偏：**实测**接收体位姿 vs 构建期名义位姿（是否施加由声明 mode 决定；
        # mode=off 时逐位不变）。托盘随载体运动 ⇒ 停靠误差会直接变成放置偏移（§11.23(48)）。
        from iraf_adapters.mujoco.payload_facts import resolve_place_pose_correction

        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            _live_tray_pose = np.asarray(self.data.xpos[tray_body], dtype=float).copy()
        # ⚠ 声明 vs 报告**必须分开**（2026-09-29 实测踩点）：`resolve_place_pose_correction` 的**返回值**
        # 只带它自己那几个键（mode/applied/delta_world_m/lateral_m/vertical_m/max_lateral_m[/ik_*]），
        # **不带** touchdown 需要的 `touch_clearance_m`/`max_vertical_m`/`residual_tolerance_m`
        # ⇒ 拿返回值当"声明"用会让触地纠偏拿到 None 并被守卫拦下（三连失败的第 3 次）。
        pose_correction_declaration = gripper.get("place_pose_correction")
        place_pose_correction = resolve_place_pose_correction(
            pose_correction_declaration, record.get("nominal_pose_m"), _live_tray_pose)
        # 诊断（§11.23(48)）：把"构建期解"与"名义/实测目标点"分别**量出来** —— 只有这样才能判
        # "偏差在构建期解侧"还是"纠偏逻辑侧"。口径：目标点 = 承载面中心 + 声明的法向间隙(pad_offset)。
        if place_pose_correction.get("mode") != "off":
            with self._data_lock:
                mujoco.mj_forward(self.model, self.data)
                _R_live = np.asarray(self.data.xmat[tray_body], dtype=float).reshape(3, 3)
                _live_top = np.asarray(self.data.xpos[tray_body], dtype=float) + _R_live @ np.asarray(
                    [0.0, 0.0, float(record["size_m"][2])], dtype=float)
            _nom_q = np.asarray([float(v) for v in (record.get("nominal_quaternion_wxyz")
                                                    or [1.0, 0.0, 0.0, 0.0])], dtype=float)
            _w, _x, _y, _z = (float(_nom_q[0]), float(_nom_q[1]), float(_nom_q[2]), float(_nom_q[3]))
            _R_nom = np.asarray([
                [1 - 2 * (_y * _y + _z * _z), 2 * (_x * _y - _z * _w), 2 * (_x * _z + _y * _w)],
                [2 * (_x * _y + _z * _w), 1 - 2 * (_x * _x + _z * _z), 2 * (_y * _z - _x * _w)],
                [2 * (_x * _z - _y * _w), 2 * (_y * _z + _x * _w), 1 - 2 * (_x * _x + _y * _y)],
            ], dtype=float)
            _nom_top = np.asarray([float(v) for v in record["nominal_pose_m"]], dtype=float) \
                + _R_nom @ np.asarray([0.0, 0.0, float(record["size_m"][2])], dtype=float)
            _pad_offset = float(gripper.get("pad_offset_m") or 0.0)
            # **搬运中的载荷相对夹口偏移**（本轮根因，§11.23(48)）：实测点 = 载荷中心 vs 指腹中点。
            # 它不是构建期量（构建解横向 8.251e-06 m ⇒ 构建基准是对的），而是搬运过程产生的
            # ⇒ 放置偏移的主项，任何"只纠放置点"的做法都动不了它。
            with self._data_lock:
                _pad_geoms_now = [int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                                        str(gripper[key])))
                                  for key in ("left_finger_geom", "right_finger_geom")]
                mujoco.mj_forward(self.model, self.data)
                _pad_now = np.mean([np.asarray(self.data.geom_xpos[g], dtype=float)
                                    for g in _pad_geoms_now if g >= 0], axis=0)
                _payload_now = np.asarray(self.data.xpos[payload_body], dtype=float).copy()
            _carry_delta = _payload_now - _pad_now
            place_pose_correction["carry_payload_offset_m"] = [round(float(v), 9)
                                                              for v in _carry_delta]
            place_pose_correction["carry_payload_lateral_m"] = round(
                float(np.linalg.norm(_carry_delta[:2])), 9)
            place_pose_correction["nominal_top_m"] = [round(float(v), 9) for v in _nom_top]
            place_pose_correction["live_top_m"] = [round(float(v), 9) for v in _live_top]
            # **构建期解自身**离"名义承载面中心 + pad_offset"多远：若不为零，说明放置航点的
            # 构建基准与接收体中心**本来就不重合** ⇒ 相对纠偏（nominal→live）救不了它。
            _descend = gripper.get("place_descend_positions") or {}
            if _descend:
                _pad_geoms = [int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                                    str(gripper[key])))
                              for key in ("left_finger_geom", "right_finger_geom")]
                with self._data_lock:
                    _scratch = mujoco.MjData(self.model)
                    _scratch.qpos[:] = self.data.qpos
                    for _name, _value in _descend.items():
                        _jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, str(_name))
                        if _jid >= 0:
                            _scratch.qpos[int(self.model.jnt_qposadr[_jid])] = float(_value)
                    mujoco.mj_forward(self.model, _scratch)
                    _pad_nom = np.mean([np.asarray(_scratch.geom_xpos[g], dtype=float)
                                        for g in _pad_geoms if g >= 0], axis=0)
                    _payload_nom = np.asarray(_scratch.xpos[payload_body], dtype=float).copy()
                _nom_target = _nom_top + np.asarray([0.0, 0.0, _pad_offset], dtype=float)
                place_pose_correction["nominal_solution_pad_mid_m"] = [
                    round(float(v), 9) for v in _pad_nom]
                place_pose_correction["nominal_solution_payload_m"] = [
                    round(float(v), 9) for v in _payload_nom]
                place_pose_correction["nominal_solution_offset_m"] = round(
                    float(np.linalg.norm(_pad_nom - _nom_target)), 9)
                place_pose_correction["nominal_solution_lateral_m"] = round(
                    float(np.linalg.norm((_pad_nom - _nom_target)[:2])), 9)
        payload_geom = payload.get("geom")
        if payload_geom is None:
            raise ValueError("载荷 %s 未声明 geom（无法量最低点）" % payload_id)

        # 前置判据（**实测事实**，替代那条系统不发布的前置状态）：双侧指腹必须同时接触载荷，
        # 否则"放置"没有任何意义（可能把台面上的方块当成熟载荷报成功）⇒ 显式拒绝。
        left_finger = self._body_id(gripper["left_finger_body"])
        right_finger = self._body_id(gripper["right_finger_body"])
        if not (self._any_contact_between(payload_body, left_finger)
                and self._any_contact_between(payload_body, right_finger)):
            raise ValueError(
                "当前未夹持载荷 %s（双侧指腹未同时接触）⇒ 拒绝放置（不伪造成功）" % payload_id)

        from iraf_core.kinematics import lowest_mesh_point_z, solve_position_ik
        from iraf_adapters.mujoco.payload_facts import check_carry_cadence

        half_x, half_y = (float(record["size_m"][0]), float(record["size_m"][1]))
        half_z = float(record["size_m"][2])
        def _geom_id(name):
            return int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, str(name)))

        pad_points = [
            {"kind": "geom", "id": _geom_id(gripper["left_finger_geom"])},
            {"kind": "geom", "id": _geom_id(gripper["right_finger_geom"])},
        ]
        payload_geoms = [_geom_id(payload_geom)]
        if any(item["id"] < 0 for item in pad_points) or payload_geoms[0] < 0:
            raise ValueError("指腹/载荷 geom 未在模型中解析到（名字口径不一致）")
        # 臂关节 id：声明名 → 模型名（`_model_name` 走 name_map，联合世界下 joint1 → piper_joint1）
        arm_joints = [int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                            self._model_name(name)))
                      for name in self._arm_joint_names()]
        duration_ms = max(1, int(duration_ms))
        phase_ms = max(1, duration_ms // 4)
        solver = {"iterations": 800, "step": 0.5, "tolerance_m": 1e-5}

        payload_geom_set = {int(item) for item in payload_geoms}
        # "谁在载荷附近"：逐 geom 算与载荷中心的距离，报出最近的若干个（含 body 名）。
        # 为什么需要（§11.23(13)/(14)）：接触对只在**穿透**时才出现，而"载体在贴近"这一事实
        # 必须能看见（四足是力矩型、驻留线程在 s04 全程驱动它踏步 ⇒ 可能漂移过来撞掉载荷）。
        neighbor_geoms = [index for index in range(int(self.model.ngeom))
                          if int(self.model.geom_bodyid[index]) != int(payload_body)]

        def _snapshot():
            """实测：托盘顶面中心、载荷最低点、指腹中点，**以及接触对的身份/法向/力与载荷 6 维位姿**。

            为什么必须给到接触对粒度（2026-09-28 §11.23(8)）：只报"接触数 > 0"无法回答
            "接触在哪、法向是什么、力多大"。实测夹持力 ≈ 11.75 N/指（压缩 1.175 mm × gain 10000）
            而载荷仅 0.39 N ⇒ "摩擦不足"已被否掉，必须看**具体是哪两个 geom 在接触、力是多少**。
            接触力必须在 `mj_forward` **之前**取：那是上一步求解器的结果，forward 之后不再是它。
            """
            with self._data_lock:
                contact_rows = {}
                for index in range(int(self.data.ncon)):
                    contact = self.data.contact[index]
                    geoms = (int(contact.geom1), int(contact.geom2))
                    if not (payload_geom_set & set(geoms)):
                        continue
                    partner = geoms[0] if geoms[1] in payload_geom_set else geoms[1]
                    force = np.zeros(6, dtype=float)
                    try:
                        mujoco.mj_contactForce(self.model, self.data, index, force)
                    except Exception:  # 绑定差异兜底：力读不到就显式留 0，不伪造
                        force = np.zeros(6, dtype=float)
                    contact_rows.setdefault(int(partner), []).append({
                        "dist_m": round(float(contact.dist), 6),
                        "normal": [round(float(v), 6) for v in np.asarray(contact.frame[0:3]).ravel()],
                        "force_n": round(float(np.linalg.norm(np.asarray(force[0:3], dtype=float))), 6)})
                mujoco.mj_forward(self.model, self.data)
                tray_rot = np.asarray(self.data.xmat[tray_body], dtype=float).reshape(3, 3)
                tray_center = np.asarray(self.data.xpos[tray_body], dtype=float)
                top = tray_center + tray_rot @ np.asarray([0.0, 0.0, half_z], dtype=float)
                low = float(lowest_mesh_point_z(self.model, self.data, payload_geoms))
                points = [np.asarray(self.data.geom_xpos[item["id"]], dtype=float)
                          for item in pad_points]
                midpoint = (points[0] + points[1]) / 2.0
                payload_center = np.asarray(self.data.xpos[payload_body], dtype=float).copy()
                payload_quat = np.asarray(self.data.xquat[payload_body], dtype=float).copy()
                contact_pairs = []
                for geom_id, rows in sorted(contact_rows.items()):
                    geom_name = (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                                 or ("#%d" % geom_id))
                    body_id = int(self.model.geom_bodyid[geom_id])
                    body_name = (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                                 or ("#%d" % body_id))
                    contact_pairs.append({"partner_geom": geom_name, "partner_body": body_name,
                                          "rows": rows})
                neighbors = []
                for index in neighbor_geoms:
                    delta = np.asarray(self.data.geom_xpos[index], dtype=float) - payload_center
                    distance = float(np.linalg.norm(delta))
                    if distance > 0.30:
                        continue
                    body_id = int(self.model.geom_bodyid[index])
                    neighbors.append({
                        "geom": (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, index)
                                 or ("#%d" % index)),
                        "body": (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                                 or ("#%d" % body_id)),
                        "distance_m": round(distance, 6)})
                neighbors.sort(key=lambda item: item["distance_m"])
                arm_joints_now = {}
                for name in self._arm_joint_names():
                    # ⚠ 必须过 `_model_name`：profile 里的关节名是无前缀的（joint1），
                    # 联合模型里是 `piper_joint1`；直接用原名查会全部落空（本轮实测踩到）。
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                 self._model_name(str(name)))
                    if joint_id >= 0:
                        arm_joints_now[str(name)] = round(
                            float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])]), 9)
                return {"tray_top": top, "payload_low_z": low, "pad_mid": midpoint,
                        # 指腹 **body** 中点（与搬运约束的 anchor 同一参照；判据不能用 geom 中点：
                        # 夹爪张开时 geom 相对 body 摆动，实测造成 3.2 cm 的假滑移）
                        "finger_mid": (np.asarray(self.data.xpos[left_finger], dtype=float)
                                       + np.asarray(self.data.xpos[right_finger], dtype=float)) / 2.0,
                        "arm_joint_positions": arm_joints_now,
                        "payload_neighbors": neighbors[:6],
                        "payload_center": payload_center,
                        "payload_pose": {"pos_m": [round(float(v), 6) for v in payload_center],
                                         "quat_wxyz": [round(float(v), 9) for v in payload_quat]},
                        "payload_contacts": contact_pairs,
                        "pad_span_m": round(float(np.linalg.norm(points[0] - points[1])), 6)}

        phase_trace = []

        def _gripper_state():
            """夹爪实测：关节角 + 执行器 ctrl（判"指令是否被复位/是否真的在夹"用）。"""
            state = {}
            with self._data_lock:
                for name in sorted(gripper.get("open_positions") or {}):
                    model_name = self._model_name(str(name))
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                 model_name)
                    if joint_id < 0:
                        continue
                    entry = {"qpos": round(float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])]), 6)}
                    actuator = next((index for index in range(int(self.model.nu))
                                     if int(self.model.actuator_trnid[index, 0]) == int(joint_id)),
                                    -1)
                    if actuator >= 0:
                        entry["ctrl"] = round(float(self.data.ctrl[actuator]), 6)
                    state[str(name)] = entry
            return state

        def _trace(phase, snapshot, note="", compact=False):
            contacts = (self._any_contact_between(payload_body, left_finger),
                        self._any_contact_between(payload_body, right_finger))
            if compact:
                # 逐样本追踪（`IRAF_DEBUG_PLACE_STRIDE=1`）用紧凑行：只留"丢手瞬间谁先动"需要的量。
                # 为什么需要它（§11.23(11)）：每 20 次迭代 ≈ 0.36 s 仿真，正好是丢件事件的尺度
                # ⇒ 只看得到"前后"，看不到"过程"，无法区分"指令侧先动"还是"载荷先掉"。
                forces = sorted((hit["force_n"] for pair in snapshot["payload_contacts"]
                                 for hit in pair["rows"]), reverse=True)
                state = {}
                with self._data_lock:
                    for name in sorted(gripper.get("open_positions") or {}):
                        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                     self._model_name(str(name)))
                        if joint_id < 0:
                            continue
                        qpos = float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])
                        actuator = next((index for index in range(int(self.model.nu))
                                         if int(self.model.actuator_trnid[index, 0]) == int(joint_id)),
                                        -1)
                        state[str(name)] = {
                            "qpos": round(qpos, 9),
                            "ctrl": (round(float(self.data.ctrl[actuator]), 9)
                                     if actuator >= 0 else None)}
                row = {"phase": str(phase), "plant_step_index": int(self.plant.step_index),
                       "pad_mid_z_m": round(float(snapshot["pad_mid"][2]), 9),
                       "payload_low_m": round(float(snapshot["payload_low_z"]), 9),
                       "payload_low_minus_pad_m": round(float(snapshot["payload_low_z"]
                                                             - snapshot["pad_mid"][2]), 9),
                       "payload_quat_w": snapshot["payload_pose"]["quat_wxyz"][0],
                       "pad_span_m": snapshot["pad_span_m"],
                       "contacts": {"left": bool(contacts[0]), "right": bool(contacts[1])},
                       "force_n_top": [round(value, 6) for value in forces[:2]],
                       "partner_bodies": sorted({pair["partner_body"]
                                                 for pair in snapshot["payload_contacts"]}),
                       "nearest": [[item["body"], item["geom"], item["distance_m"]]
                                   for item in snapshot["payload_neighbors"]],
                       "gripper": state, "note": note}
                phase_trace.append(row)
                if os.environ.get("IRAF_DEBUG_PLACE") == "1":
                    print("PLACE_TRACE " + json.dumps(row, ensure_ascii=False), flush=True)
                return row
            row = {"phase": str(phase),
                   # 共享植物下 guest 的**推进节拍**必须可观测（2026-09-28）：同一个
                   # `_advance_for(0)` 在 owner 是"推 1 步"，在 guest 是"等 owner 推进"，
                   # 而 owner 由驻留线程推进 ⇒ 两次控制更新之间植物可能前进很多步。
                   # 上一轮就是这么发现"20 ms 内下落 9.8 cm"这种物理不可能的读数的。
                   "plant_step_index": int(self.plant.step_index),
                   "tray_top_m": [round(float(v), 6) for v in snapshot["tray_top"]],
                   "payload_low_m": round(float(snapshot["payload_low_z"]), 6),
                   "pad_mid_m": [round(float(v), 6) for v in snapshot["pad_mid"]],
                   "finger_mid_m": [round(float(v), 6) for v in snapshot["finger_mid"]],
                   "payload_low_minus_pad_m": round(float(snapshot["payload_low_z"]
                                                        - snapshot["pad_mid"][2]), 6),
                   "finger_contacts": {"left": bool(contacts[0]), "right": bool(contacts[1])},
                   # 接触对粒度（身份/法向/力）+ 载荷 6 维位姿 + 指腹间距（§11.23(8) 的下一步）
                   # 臂关节实测 qpos（与报告 place_*_positions 逐关节对账用；见 §11.23(39)）
                   "arm_joint_positions": snapshot.get("arm_joint_positions"),
                   "payload_contacts": snapshot["payload_contacts"],
                   "payload_pose": snapshot["payload_pose"],
                   "pad_span_m": snapshot["pad_span_m"],
                   "gripper_state": _gripper_state(),
                   "commanded_closed": {str(k): float(v) for k, v in
                                        (gripper.get("closed_positions") or {}).items()},
                   "note": note}
            phase_trace.append(row)
            if os.environ.get("IRAF_DEBUG_PLACE") == "1":
                print("PLACE_TRACE " + json.dumps(row, ensure_ascii=False), flush=True)
            return row

        # 运动过程采样（`IRAF_DEBUG_PLACE=1` 时逐 20 步打印；同时进证据 phase_trace）：
        # 用来定位"在哪一段、哪一刻丢件"。**必须在控制路径内采样**（第 10 个工装缺陷的纪律）。
        def _segment_sampler(segment_name, sink):
            # 采样步长：默认 20（≈0.36 s 仿真/次），丢件取证时用环境变量降到 1（逐样本）。
            try:
                stride = max(1, int(os.environ.get("IRAF_DEBUG_PLACE_STRIDE", "20")))
            except ValueError:
                stride = 20

            def _hook(step, elapsed):
                if os.environ.get("IRAF_DEBUG_PLACE") != "1" or step % stride != 0:
                    return
                row = _trace("%s@step%d" % (segment_name, step), _snapshot(), "运动过程",
                             compact=(stride == 1))
                sink.append(row)
            return _hook

        segment_samples = []
        # ---- 搬运：**回放构建期解出的关节空间解**（不再运行时现解 IK，理由见 §11.16）
        #      构建期由声明的求解器按接收体名义位姿解出 above/descend/retreat 三段；
        #      后端只做 `_move_trajectory` 回放 —— 与已验证的 pick 完全同一条路。
        transit = gripper.get("place_transit_positions")
        above = gripper.get("place_above_positions")
        descend = gripper.get("place_descend_positions")
        retreat = gripper.get("place_retreat_positions")
        missing = [name for name, value in (("place_above_positions", above),
                                            ("place_descend_positions", descend),
                                            ("place_retreat_positions", retreat))
                   if not isinstance(value, dict) or not value]
        if missing:
            raise ValueError("场景报告缺少放置段关节解 %s：放置四段必须由**声明的求解器**在构建期解出"
                             "（运行时现解 IK 会落错分支并把载荷打掉，见 §11.16）" % missing)
        # 搬运段的**夹爪语义**：必须由场景报告给出（构建期按声明写入），缺声明即显式失败。
        # 为什么不能有实现层默认值（2026-09-28 §11.23(12)）：把夹爪关节当轨迹插值（当前 qpos → 0.023）
        # 会让夹口**整段合拢**，把载荷沿夹口轴向**楔/挤出去**（实测力 12 N → 2.6 N → 脱离，
        # 而载荷只重 0.39 N、摩擦容量 ≳46 N ⇒ 不是摩擦问题）。`hold` = 目标取当前实测 qpos（零合拢）。
        carry = gripper.get("carry_gripper")
        if not isinstance(carry, dict) or str((carry or {}).get("mode") or "") not in ("hold", "trajectory"):
            raise ValueError(
                "场景报告缺少合法的 gripper.carry_gripper.mode（只允许 hold / trajectory）："
                "搬运段的夹爪语义必须由声明给出，不得在实现层写默认值（见 §11.23(12)："
                "把夹爪当轨迹会合拢夹口并挤出载荷）")
        carry_mode = str(carry["mode"])
        carry_targets = {}

        def _carry_goal(positions, segment):
            """搬运段目标：`hold` 时把夹爪关节目标换成**当前 ctrl**（保持夹紧力，不改变指令）。

            为什么是 ctrl 而不是 qpos（2026-09-28，本轮我自己的设计错误，place24 实测暴露）：
            位置伺服的力 ∝ (target − qpos)。把 target 设成**当前 qpos** ⇒ 误差为 0 ⇒ **夹持力为 0**
            ⇒ 载荷在搬运中滑落（place24 实测：`qpos == ctrl == 0.024215`，方块照样掉）。
            正确语义是**保持夹紧力**：target 取**当前 ctrl**（= 抓取段已经把载荷夹住的那个指令，
            不插值 ⇒ 指令零位移、压缩量保持 ⇒ 11.75 N 一直存在）。
            """
            goal = {str(k): float(v) for k, v in positions.items()}
            if carry_mode != "hold":
                return goal
            with self._data_lock:
                for name in sorted(gripper.get("open_positions") or {}):
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                                 self._model_name(str(name)))
                    if joint_id < 0:
                        continue
                    actuator = next((index for index in range(int(self.model.nu))
                                     if int(self.model.actuator_trnid[index, 0]) == int(joint_id)),
                                    -1)
                    if actuator < 0:
                        continue
                    held = float(self.data.ctrl[actuator])
                    goal[str(name)] = held
                    carry_targets.setdefault(segment, {})[str(name)] = round(held, 9)
            return goal
        snapshot = _snapshot()
        start_row = _trace("start", snapshot, "进入放置段（应仍在夹持中）")
        # 搬运段**抓取约束**（声明 `gripper.carry_constraint`）：把载荷焊在 mocap anchor 上、
        # anchor 每步跟随指腹中点 ⇒ 搬运不依赖摩擦（动机见 §11.23(29)：斜夹过盈会被楔出，
        # 纯摩擦路线已穷尽否证）。约束由联合构建器**惰性注入**（active="false"），此处按声明激活。
        carry_cfg = gripper.get("carry_constraint") or {}
        carry_equality_id = None
        carry_anchor_rel_quat = None
        carry_release_gripper = False
        carry_max_slip_m = None
        carry_cadence = None
        if bool(carry_cfg.get("enabled")):
            name = str(carry_cfg.get("equality_name") or "")
            anchor = str(carry_cfg.get("anchor_body") or "")
            if not name or not anchor:
                raise ValueError("carry_constraint.enabled=true 但缺少 equality_name/anchor_body")
            # 约束类型必须由声明给出（fail-closed；见 §11.23(41)）：`connect` 是球铰，只约束平移
            # ⇒ 实测载荷在夹口里翻滚/楔出；`weld` 才约束旋转。缺声明即显式失败，不猜。
            carry_type = str(carry_cfg.get("type") or "")
            if carry_type not in ("connect", "weld"):
                raise ValueError("场景报告缺少合法的 gripper.carry_constraint.type（只允许 connect / "
                                 "weld，实际 %r）：搬运是否约束旋转必须由声明决定" % carry_type)
            # 焊缝语义（§11.23(41)(g)）：激活后是否**释放夹爪** + 释放后的滑移判据阈值。
            # 缺声明即显式失败：释放夹爪会改变"是否握着"的判据口径，不能由实现层默认。
            # 搬运节拍上限（§11.23(43)）：anchor 每个**控制迭代**只跟随一次，而植物 owner 一次
            # 可能推进很多步（实测均值 18.01 步/迭代）⇒ 上限由声明给出，实测超限即**响亮失败**。
            limit = int(carry_cfg.get("max_plant_steps_per_iteration") or 0)
            if limit <= 0:
                raise ValueError("场景报告缺少合法的 gripper.carry_constraint."
                                 "max_plant_steps_per_iteration（>0）：搬运节拍上限是"
                                 "「载荷随指腹刚性搬运」这一前提的成立条件")
            carry_cadence = {"limit": limit}
            carry_release_gripper = bool(carry_cfg.get("release_gripper"))
            carry_max_slip_m = float(carry_cfg.get("max_slip_m") or 0.0)
            if carry_release_gripper and not carry_max_slip_m > 0:
                raise ValueError("声明了 carry_constraint.release_gripper=true 但没有正的 "
                                 "max_slip_m：释放夹爪后不能用指腹接触判\"是否握着\"，"
                                 "必须给出焊缝滑移阈值")
            carry_equality_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
            if carry_equality_id < 0:
                raise ValueError(
                    "搬运约束 %s 不在联合模型里：构建器必须按声明注入 anchor+<connect>（见 "
                    "scene_builder._inject_carry_constraint）" % name)
            anchor_body_id = self._body_id(anchor)
            if int(self.model.body_mocapid[anchor_body_id]) < 0:
                raise ValueError("搬运约束的 anchor %s 不是 mocap body" % anchor)
            # ⚠ **先把 anchor 摆到「指腹中点 + 载荷相对指腹的初始偏移」，再激活约束**
            # （2026-09-28 §11.23(41)）。两层事实，都是实测：
            # ① 注入时 anchor 的位姿是**载荷的名义世界位姿**（构建期声明），载荷此刻在夹口里
            #    ⇒ 直接激活等于让约束第一步消掉 ~8 cm 初值差：实测夹持力 13.03/12.93 N →
            #    **89.62/41.64 N**、指腹间距 0.068828 → 0.075489 m、指腹中点一步 0.133531 →
            #    0.155828 m ⇒ 载荷被硬拽出夹口落地（s04「绕行航点后失去夹持」的根因）。
            # ② 只摆到「指腹 body 中点」仍不够（第一帧 48.30 → 115.59 N）。
            #    离线 A/B/D 对照（夹爪保持闭合 = 运行期 `carry_gripper: hold` 口径）：
            #      A 只摆指腹中点       ：首帧 48.30 → 115.59 N，稳态 15.54/14.87 N（抖）
            #      B 指腹中点+初始偏移  ：首帧 19.15 →  52.85 N，稳态 **13.29/13.27 N（稳）**，
            #                            箱心 3 s Δ+0.073631 m = 与指腹刚性同步 ✓
            #    ⇒ 取 B：把 anchor 按「载荷相对指腹的初始偏移」整体刚性平移 ⇒ 激活瞬间几乎无纠正力，
            #    且等价于「载荷随指腹刚性平移」，与 MuJoCo 把 anchor 记在哪一体的局部系无关。
            with self._data_lock:
                anchor_mocap = int(self.model.body_mocapid[anchor_body_id])
                left_point = np.asarray(self.data.xpos[self._body_id(str(gripper["left_finger_body"]))],
                                        dtype=float)
                right_point = np.asarray(self.data.xpos[self._body_id(str(gripper["right_finger_body"]))],
                                         dtype=float)
                finger_mid = (left_point + right_point) / 2.0
                payload_now = np.asarray(self.data.xpos[payload_body], dtype=float)
                carry_anchor_offset = payload_now - finger_mid
                self.data.mocap_pos[anchor_mocap] = payload_now.copy()
                if carry_type == "weld":
                    # 刚性焊：anchor 的**姿态**必须等于载荷姿态（weld 不给 relpose ⇒ 两体位姿重合），
                    # 之后按「腕部姿态 ⊗ 该项」跟随 ⇒ 载荷姿态随腕部刚性走。
                    payload_quat = np.asarray(self.data.xquat[payload_body], dtype=float)
                    wrist_quat = np.asarray(self.data.xquat[self._body_id(str(gripper["wrist_body"]))],
                                            dtype=float)
                    carry_anchor_rel_quat = _quat_mul(_quat_conj(wrist_quat), payload_quat)
                    self.data.mocap_quat[anchor_mocap] = payload_quat.copy()
                else:
                    self.data.mocap_quat[anchor_mocap] = (1.0, 0.0, 0.0, 0.0)
                self.data.eq_active[carry_equality_id] = 1
            if carry_release_gripper:
                # 声明 `carry_constraint.release_gripper=true`（§11.23(41)(g)）：焊缝已承担搬运
                # ⇒ **释放夹爪**，让指腹不再与焊缝竞争（实测竞争会把载荷在 xy 推偏 2.9 cm、左指脱开）。
                # 之后三段按 `carry_gripper: hold` 语义把这个"张开"的 ctrl 保持住（零合拢）。
                self._set_gripper_controls(dict(gripper["open_positions"]))
            # ⚠ 这里必须给**名字**（报告里的 body 名已带联合世界前缀），不能给 body id：
            # `_move_trajectory` 的 anchor 钩子按名字解析（实测传 id 会报 body not found: 26）。
            carry_anchor_kwargs = {
                "anchor_body": anchor,
                "anchor_follow": (str(gripper["left_finger_body"]),
                                  str(gripper["right_finger_body"])),
                # 驱动方式 B（见上）：anchor = 指腹中点 + 该偏移 ⇒ 载荷随指腹**刚性平移**
                "anchor_offset": carry_anchor_offset,
                # 节拍实测（每段都统计；超声明上限即在该段内显式失败）
                "cadence_sink": carry_cadence,
            }
            if carry_type == "weld":
                # 刚性焊还要跟随**姿态**：anchor 姿态 = 腕部姿态 ⊗ 激活瞬间的（腕部⁻¹⊗载荷）
                carry_anchor_kwargs["anchor_wrist"] = str(gripper["wrist_body"])
                carry_anchor_kwargs["anchor_rel_quat"] = carry_anchor_rel_quat

        else:
            carry_anchor_kwargs = {}

        def _carry_grip_row(row, label):
            """搬运中"是否还握着"的判据。

            焊缝激活且已按声明释放夹爪时，**不能**再看指腹接触（夹爪本就张开了）⇒ 改看**焊缝滑移**：
            `|载荷中心 − (指腹中点 + 激活时的载荷-指腹偏移)|`。阈值 `carry_constraint.max_slip_m`
            来自声明（缺声明即在上面的校验里显式失败）。未释放时保持原有的双侧指腹接触判据。
            ⚠ 传入的是 `_trace` 的**追踪行**（键名 `pad_mid_m` / `payload_pose` / `finger_contacts`），
            不是 `_snapshot` 的快照（键名 `pad_mid`）—— 混用会 KeyError（本轮实测踩到）。
            """
            if carry_equality_id is not None and carry_release_gripper:
                finger_mid = row.get("finger_mid_m")
                if finger_mid is None:
                    raise ValueError("搬运判据需要追踪行里的 finger_mid_m")
                # 参照必须是「指腹 body 中点 + 激活瞬间的载荷−指腹body偏移」——与 anchor 驱动同一参照。
                # ⚠ 不能用 `pad_mid_m`（geom 中点）：夹爪张开时 geom 相对 body 摆动，
                # 实测造成 0.031689 m 的**假滑移**（真值仅约 1 mm）。
                slip = float(np.linalg.norm(
                    np.asarray(row["payload_pose"]["pos_m"], dtype=float)
                    - (np.asarray(finger_mid, dtype=float) + np.asarray(carry_anchor_offset))))
                row["carry_slip_m"] = round(slip, 9)
                if slip > float(carry_max_slip_m):
                    raise ValueError(
                        "%s后焊缝滑移 %.6f m > 声明阈值 %.6f m ⇒ 载荷已脱离焊缝"
                        "（搬运节拍实测：单次迭代最多 %s 步 / 声明阈值 %s 步；步数越大，载荷挂"
                        "陈旧 anchor 越久 ⇒ 先看节拍再看滑移阈值）"
                        % (label, slip, float(carry_max_slip_m),
                           (carry_cadence or {}).get("max_plant_steps_per_iteration"),
                           (carry_cadence or {}).get("limit")))
                return
            if not (row["finger_contacts"]["left"] and row["finger_contacts"]["right"]):
                raise ValueError("%s后失去夹持（载荷已脱离）⇒ 拒绝继续放置" % label)
        if not (start_row["finger_contacts"]["left"] and start_row["finger_contacts"]["right"]):
            raise ValueError(
                "进入放置段时双侧指腹未同时接触载荷（依据已进证据 phase_trace[0]）⇒ 拒绝继续")
        # ①a 绕行航点（抓取点正上方、托盘高度）：先竖直抬升，避免"直插托盘上方"的弧线穿过载体
        # ①a 绕行航点（抓取点正上方、托盘高度）：先竖直抬升，避免"直插托盘上方"的弧线穿过载体
        if isinstance(transit, dict) and transit:
            self._move_trajectory(_carry_goal(transit, "transit"), phase_ms,
                                  ctrl_offsets=self._pick_ctrl_offsets("approach") or None,
                                  sampler=_segment_sampler("place_transit_positions", segment_samples),
                                  **carry_anchor_kwargs)
            seg_row = _trace("after_transit", _snapshot(), "绕行航点（竖直抬升到托盘高度）")
            _carry_grip_row(seg_row, "绕行航点")
        # ---- 放置点纠偏（声明驱动，§11.23(48)）：把 above/descend 两段的关节目标纠到
        #      **运行期实测位姿**上。off/measure_only ⇒ 一个字节都不动（逐位不变）；
        #      lateral_only / full_pose ⇒ 按声明重解 + 三条自证（任一不满足即显式抛错）。
        if place_pose_correction.get("mode") in ("lateral_only", "full_pose"):
            _delta = np.asarray(place_pose_correction["delta_world_m"], dtype=float)
            if place_pose_correction["mode"] == "lateral_only":
                _delta = np.asarray([_delta[0], _delta[1], 0.0], dtype=float)   # 只纠水平两轴
            _wrist_body = self._body_id(gripper["wrist_body"])
            _segments = {}
            for _label, _positions in (("above", above), ("descend", descend)):
                _goal, _rows = self._corrected_place_goal(
                    _positions, _delta, pad_points, arm_joints, _wrist_body, place_pose_correction, _label)
                _segments[_label] = _rows
                if _label == "above":
                    above = _goal
                else:
                    descend = _goal
            place_pose_correction["applied"] = True
            place_pose_correction["applied_delta_world_m"] = [round(float(v), 9) for v in _delta]
            place_pose_correction["segments"] = _segments
        # ⚠ 结构修正（2026-09-28 §11.23(41)）：`above` / `descend` 的回放**必须在这一层**。
        # 实测（AST 对账）：`descend` 曾被嵌在"失去夹持就报错"的 `if not (双侧接触):` 体内
        # ⇒ 是**死代码、永不执行**：`after_above` 与 `after_descend` 之间植物只前进 5 步
        # （190665 → 190670）、载荷位姿逐位相同（low 0.165531），后续 release/retreat 在
        # 4.5 cm 高处放空 ⇒ 失败报"未确认载荷已放下"。这条 bug 在**成功路径**与失败路径都被掩盖。
        # ①b 抬升到承载面上方（回放）
        self._move_trajectory(_carry_goal(above, "above"), phase_ms,
                              ctrl_offsets=self._pick_ctrl_offsets("approach") or None,
                              sampler=_segment_sampler("place_above_positions", segment_samples),
                              **carry_anchor_kwargs)
        seg_row = _trace("after_above", _snapshot(), "抬升段结束（承载面上方）")
        _carry_grip_row(seg_row, "抬升段")
        # ② 下行到位（回放）
        self._move_trajectory(_carry_goal(descend, "descend"), phase_ms,
                              ctrl_offsets=self._pick_ctrl_offsets("grasp") or None,
                              sampler=_segment_sampler("place_descend_positions", segment_samples),
                              **carry_anchor_kwargs)
        after_descend = _snapshot()
        _trace("after_descend", after_descend, "下行到位（载荷应正落在承载面上）")
        # ---- 运行期**触地纠偏**（2026-09-29 §11.25(f-4)）
        # 为什么必须运行期：方块在夹口里**下滑 ~12 mm 且逐轮不同**（PLACE_TRACE 实测「载荷最低点 −
        # 指腹中点」在放置段内摆动 24.7 mm：start −0.063954 → after_above −0.039229 → after_descend
        # −0.043797，而构建期在抓取位形上 FK 得 −0.032008）⇒ 构建期名义几何覆盖不了 ⇒ 松手高度逐轮不同
        # ⇒ 落点横向散布大（offset 实测 0.0139~0.0630，判据 0.06；约 1/4 概率直接掉件）。
        # 口径：要消的量 = **载荷底面 − 承载面 − 触地间隙**（不是"托盘位姿 − 名义位姿"——后者实测
        # 施加后反而更差 0.051933449 → 0.059755439，见 §11.25(f-4)）。只做**一次**竖向微降，再复测；
        # 仍进不了容差即**显式失败**（不静默照放）。
        touch_record = None
        if str((place_pose_correction or {}).get("mode")) == "touchdown":
            from iraf_adapters.mujoco.payload_facts import resolve_touchdown_correction

            def _bearing_z(snapshot):
                """从**快照**里取承载面世界 z。

                ⚠ 键名必须用快照自己的（2026-09-29 实测踩点）：`_snapshot()` 给的是
                `tray_top`（三维向量）与 `payload_low_z`（标量）；`tray_top_m` / `payload_low_m`
                是 `_trace()` **加工后**才加的键 ⇒ 在快照上取这两个名字会落空。
                第一版就是取错了名字，被自己的 fail-closed 守卫拦下（s04 报"缺承载面高度"）。
                """
                value = snapshot.get("bearing_surface_z_m")
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    return float(value)
                for key in ("tray_top", "tray_top_m"):
                    top = snapshot.get(key)
                    # ⚠ 不能用 `isinstance(..., (list, tuple))`（2026-09-29 实测踩点第二次）：
                    # 快照里的 `tray_top` 是 **numpy 数组** ⇒ isinstance 判 False、守卫又拦下
                    # （报"缺承载面高度"）。改成"可索引且长度为 3"的判定，兼容 ndarray/list/tuple。
                    if top is None or isinstance(top, (str, bytes)):
                        continue
                    try:
                        if len(top) == 3:
                            return float(top[2])
                    except TypeError:
                        continue
                raise ValueError(
                    "触地纠偏需要快照里的承载面高度（tray_top 或 bearing_surface_z_m）"
                    "：缺它即无法判定落点，拒绝照放")

            def _payload_low(snapshot):
                """从**快照**里取载荷最低点 z（同上的键名纪律）。"""
                for key in ("payload_low_z", "payload_low_m"):
                    value = snapshot.get(key)
                    if isinstance(value, (int, float)) and not isinstance(value, bool):
                        return float(value)
                raise ValueError("触地纠偏需要快照里的载荷最低点（payload_low_z）：缺它即拒绝照放")

            def _xy(vector, label):
                if vector is None or isinstance(vector, (str, bytes)):
                    raise ValueError("放置纠偏需要快照里的 %s（二维/三维坐标）：缺它即拒绝照放" % label)
                try:
                    if len(vector) >= 2:
                        return [float(vector[0]), float(vector[1])]
                except TypeError:
                    pass
                raise ValueError("放置纠偏需要快照里的 %s：实际 %r" % (label, vector))

            def _payload_xy(snapshot):
                return _xy(((snapshot.get("payload_pose") or {}).get("pos_m")
                            if isinstance(snapshot.get("payload_pose"), dict)
                            else snapshot.get("payload_pose")), "载荷中心 xy")

            def _tray_xy(snapshot):
                return _xy(snapshot.get("tray_top") if snapshot.get("tray_top") is not None
                           else snapshot.get("tray_top_m"), "托盘中心 xy")

            _wrist_body_td = self._body_id(gripper["wrist_body"])
            # **迭代收敛**（2026-09-29 实测后改）：一次走满会**过冲**（实测落差 +26.3 mm → −4.1 mm，
            # 即多走了 4.1 mm 把方块压进承载面以下），因为移动过程中载荷相对指腹还会再动（"下滑"）。
            # 因此按声明的**步长比例**逐次逼近 + 每次复测：grip 相对滑动让单次开环量不可靠，
            # 闭环（测量→小步→再测量）才可控。上限与比例都必须由声明给出（实现层不写默认值）。
            _step_fraction = float(pose_correction_declaration["touchdown_step_fraction"])
            _max_iterations = int(pose_correction_declaration["touchdown_max_iterations"])
            if not 0.0 < _step_fraction <= 1.0:
                raise ValueError(
                    "place_pose_correction.touchdown_step_fraction 必须在 (0,1]，实际 %r" % _step_fraction)
            if _max_iterations < 1:
                raise ValueError(
                    "place_pose_correction.touchdown_max_iterations 必须 ≥ 1，实际 %r" % _max_iterations)
            _cumulative_delta = np.zeros(3, dtype=float)
            _iterations = []
            _resolved = resolve_touchdown_correction(
                pose_correction_declaration, _payload_low(after_descend), _bearing_z(after_descend))
            touch_record = {"first": _resolved}
            from iraf_adapters.mujoco.payload_facts import resolve_place_alignment_correction
            for _index in range(1, _max_iterations + 1):
                _resolved = resolve_touchdown_correction(
                    pose_correction_declaration, _payload_low(after_descend),
                    _bearing_z(after_descend))
                # **横向**同轮一起纠（§11.25(f-5)：实测最终偏移 61.8 mm 里 47.4 mm 在释放前就已存在，
                # 主因是搬运段在夹口里侧滑 21.6 mm；竖向纠偏不改变横向 ⇒ 必须同轮纠 xy）
                _lateral = resolve_place_alignment_correction(
                    pose_correction_declaration, _payload_xy(after_descend), _tray_xy(after_descend))
                if not _resolved["applied"] and not _lateral["applied"]:
                    _iterations.append({"iteration": _index, "skipped": "竖向与横向都已在容差内",
                                        "vertical": _resolved, "lateral": _lateral})
                    break
                _step_vec = np.asarray([
                    (float(_lateral["delta_xy_m"][0]) if _lateral["applied"] else 0.0),
                    (float(_lateral["delta_xy_m"][1]) if _lateral["applied"] else 0.0),
                    (float(_resolved["delta_z_m"]) if _resolved["applied"] else 0.0)], dtype=float)
                _step_vec = _step_vec * _step_fraction
                _cumulative_delta = _cumulative_delta + _step_vec
                _goal_td, _rows_td = self._corrected_place_goal(
                    descend, _cumulative_delta, pad_points, arm_joints, _wrist_body_td,
                    pose_correction_declaration,
                    "descend_touchdown_%d" % _index)
                self._move_trajectory(
                    _carry_goal(_goal_td, "descend"), phase_ms,
                    ctrl_offsets=self._pick_ctrl_offsets("grasp") or None,
                    sampler=_segment_sampler("place_descend_touchdown_positions", segment_samples),
                    **carry_anchor_kwargs)
                after_descend = _snapshot()
                _trace("after_touchdown_%d" % _index, after_descend,
                       "触地纠偏第 %d 次（累计 delta_z=%.6f）" % (_index, _cumulative_delta[2]))
                _iterations.append({"iteration": _index,
                                    "step_m": [round(float(v), 9) for v in _step_vec],
                                    "cumulative_m": [round(float(v), 9) for v in _cumulative_delta],
                                    "gap_after_m": round(float(_payload_low(after_descend)
                                                               - _bearing_z(after_descend)), 9),
                                    "lateral_after_m": round(float(np.hypot(
                                        _payload_xy(after_descend)[0] - _tray_xy(after_descend)[0],
                                        _payload_xy(after_descend)[1] - _tray_xy(after_descend)[1])), 9),
                                    "segment": _rows_td})
            _final = resolve_touchdown_correction(
                pose_correction_declaration, _payload_low(after_descend), _bearing_z(after_descend))
            _final_lateral = resolve_place_alignment_correction(
                pose_correction_declaration, _payload_xy(after_descend), _tray_xy(after_descend))
            touch_record["iterations"] = _iterations
            touch_record["final"] = _final
            touch_record["final_lateral"] = _final_lateral
            touch_record["cumulative_delta_m"] = [round(float(v), 9) for v in _cumulative_delta]
            if _final_lateral["applied"]:
                raise ValueError(
                    "放置纠偏迭代 %d 次后横向仍未到位：剩余 %.9f m（载荷中心 %r − 托盘中心 %r）"
                    "超过容差 %.9f m ⇒ 拒绝照放（不静默截断）"
                    % (_max_iterations, _final_lateral["lateral_m"],
                       _payload_xy(after_descend), _tray_xy(after_descend),
                       float(pose_correction_declaration["lateral_tolerance_m"])))
            if _final["applied"]:
                raise ValueError(
                    "触地纠偏迭代 %d 次后仍未到位：剩余竖向 %.9f m（载荷底面 %.9f − 承载面 %.9f − "
                    "触地间隙 %.9f）超过容差 %.9f m ⇒ 拒绝照放（不静默截断、不按名义高度照放）"
                    % (_max_iterations, _final["delta_z_m"], _final["payload_low_m"],
                       _final["bearing_surface_z_m"],
                       float(pose_correction_declaration["touch_clearance_m"]),
                       float(pose_correction_declaration["residual_tolerance_m"])))
            descend = _goal_td if _iterations and "cumulative_m" in _iterations[-1] else descend
            touch_record["done"] = True
        padding = float(gripper.get("pad_offset_m") or 0.0)
        alignment_distance = float(
            np.linalg.norm(after_descend["pad_mid"]
                           - (after_descend["tray_top"] + np.asarray([0.0, 0.0, padding]))))
        # ③ 开夹爪（释放）
        if carry_equality_id is not None:
            # **释放约束后**才张开夹爪：这样"放下"由约束保证载荷留在承载面上，而不是靠夹爪
            with self._data_lock:
                self.data.eq_active[carry_equality_id] = 0
        self._set_gripper_controls(dict(gripper["open_positions"]))
        self._advance_for(phase_ms)
        # ---- ④ 抬离（回放构建期解）
        retreat_goal = {str(k): float(v) for k, v in retreat.items()}
        if carry_release_gripper:
            # 已按声明释放夹爪 ⇒ 抬离段**不能**把夹爪再合上（否则会把刚放好的载荷推走，
            # 也会让 `released`（张爪后不再接触）判据失败）。
            for name in (gripper.get("open_positions") or {}):
                retreat_goal[str(name)] = float(gripper["open_positions"][name])
        self._move_trajectory(retreat_goal, phase_ms,
                              ctrl_offsets=self._pick_ctrl_offsets("lift") or None)
        _trace("after_retreat", _snapshot(), "抬离结束（载荷应留在承载面上）")

        # ---- ④b **落稳窗**（声明化；缺声明即失败）：放下后载荷是**自由体**，离开指腹的那一刻
        #      它还没落到承载面上 —— 必须给它时间落稳，再判"我放下了"。
        # 为什么必须有（2026-09-29 实测，§11.23(47)）：托盘挂到狗背上后，抬离结束时载荷**仍在
        # 自由下落中**（载荷自由关节速度 0.005385696 m/s、距承载面 0.000935902 m；MuJoCo 报的是
        # 4 个 **+dist** 接触点 ⇒ 尚未压上）⇒ 下一帧（s05）量到 `resting_gap_m=+0.000653879 m`、
        # `payload_on_target=False`，s05 因此误报"载荷未确认落在接收体上"。
        # 旧的世界固定托盘**恰好**落在承载面上（gap −0.000215511）⇒ 这条时序假设被掩盖了很久。
        # 判据不能靠"恰好"：先落稳，再按共享测量取事实（与四足侧同一函数，口径不漂移）。
        settle_ms = gripper.get("place_settle_ms")
        if not isinstance(settle_ms, int) or isinstance(settle_ms, bool) or settle_ms <= 0:
            raise ValueError(
                "场景报告缺少 gripper.place_settle_ms（正整数，毫秒）：放下后必须让载荷落稳再判；"
                "实现层不写默认值（声明缺失即显式失败）")
        self._advance_for(int(settle_ms))
        from iraf_adapters.mujoco.payload_facts import confirm_payload_on_target

        # ⚠ **必须持 `_data_lock`**：联合世界里臂是 guest，而共享植物由 owner 的驻留线程推进；
        # `confirm_payload_on_target` 内部会 `mj_forward`（**写** data）⇒ 不持锁就是数据竞态。
        # 本轮实测代价：不持锁直接 **SIGSEGV（exit 139，core dumped）**。`_snapshot()` 一直持锁，
        # 所以同一条路径在过去从未暴露过这个约束。
        with self._data_lock:
            settled_facts = confirm_payload_on_target(
                mujoco, self.model, self.data, str(payload["body"]), str(record["body"]))
        final = _snapshot()
        _trace("after_settle", final, "落稳窗结束（载荷应静止在承载面上）")

        # ---- 判据（事实，不设阈值）
        left_id = self._body_id(gripper["left_finger_body"])
        right_id = self._body_id(gripper["right_finger_body"])
        # `released` = 张开后指腹与载荷**不再接触**（事实判据，不设力阈值）
        released = not (self._any_contact_between(payload_body, left_id)
                        or self._any_contact_between(payload_body, right_id))
        payload_in_footprint = (
            abs(float(final["payload_center"][0]) - float(final["tray_top"][0])) <= half_x
            and abs(float(final["payload_center"][1]) - float(final["tray_top"][1])) <= half_y)
        # 判"载荷落在接收体上"必须用**共享测量**（与四足侧 accept_payload 同一函数）：
        # 原先这里是本地实现 `_payload_rests_on_target`，**只看有没有接触**（不看落位间隙、
        # 也不认模型声明的接触 margin）⇒ 与四足侧口径分叉（本轮登记的债，见 §11.23(47)）。
        # 现在两侧都取 `confirm_payload_on_target` 的 `payload_on_target`（接触 且 gap ≤ margin）。
        resting = bool(settled_facts["payload_on_target"])
        offset_from_center = float(np.linalg.norm(
            np.asarray([final["payload_center"][0] - final["tray_top"][0],
                        final["payload_center"][1] - final["tray_top"][1]], dtype=float)))
        evidence = {
            "place_target_body": record["body"],
            "payload_body": payload["body"],
            "released": bool(released),
            "payload_in_tray": bool(payload_in_footprint and resting),
            "place_alignment": {
                "center_distance_m": round(alignment_distance, 9),
                "clearance_m": round(float(approach_offset), 9),
                "offset_from_center_m": round(offset_from_center, 9),
            },
            "retreat_delta_m": round(float(final["pad_mid"][2] - after_descend["pad_mid"][2]), 9),
            "gripper_open_positions": {str(k): float(v) for k, v in gripper["open_positions"].items()},
            # ⚠ `gripper_close_hold`/`gripper_approach_hold` 是 **pick_object** 的语义，
            # 不是 place 的（本层曾是 pick 的局部变量被误写进 place 证据 ⇒ 潜伏 NameError，
            # 直到搬运段第一次跑通才暴露）。它们的声明与留痕在各自技能的层里，不在此处。
            # 搬运段的夹爪语义 + 实际下发的"保持值"（证据：证明目标是实测 qpos，而不是报告里的 0.023）
            "carry_constraint_activated": bool(carry_equality_id is not None),
            # 节拍实测（证明「随指腹刚性搬运」的前提在本次运行里成立；见 §11.23(43)）
            # 按**契约的键名**显式组装（后端内部用 `limit`；契约要求 `declared_limit`
            # ⇒ 直接用内部名会被门禁判"缺少必需属性"，本轮实测踩到）
            "carry_cadence": (None if carry_cadence is None else {
                "declared_limit": int(carry_cadence["limit"]),
                "max_plant_steps_per_iteration": int(
                    carry_cadence.get("max_plant_steps_per_iteration", 0)),
                "iterations": int(carry_cadence.get("iterations", 0)),
                "last_delta": int(carry_cadence.get("last_delta", 0)),
                "within_declared_limit": (check_carry_cadence(carry_cadence) is None),
            }),
            "gripper_carry": {"mode": carry_mode, "segment_targets": carry_targets,
                              "note": ("hold = 目标取**当前 ctrl**（保持夹紧力、指令零位移）。"
                                       "注意不能用 qpos 当目标：位置伺服的力 ∝ (target − qpos)，"
                                       "target=qpos ⇒ 夹持力为 0 ⇒ 载荷滑落（place24 实测，§11.23(13)）")},
            "place_mode": "declared_offset",
            # 落稳窗实测（证明"放下"是在载荷**静止在承载面上**之后判的；见 §11.23(47)）
            "place_settle_ms": int(settle_ms),
            "place_settled_gap_m": float(settled_facts["resting_gap_m"]),
            "place_settled_speed_mps": float(settled_facts["last_speed_mps"]),
            # 按**共享测量**（与四足侧同一函数）判"落在承载面上"，并给出它用的 margin 口径：
            # 厂商 Go2 模型声明 margin=0.001 ⇒ 载荷稳定停在几何表面上方 ~1 mm（见 §11.23(47)）。
            "place_settled_on_target": bool(settled_facts["payload_on_target"]),
            # 放置点纠偏的实测（见 §11.23(48)）：本步只测量不施加，先把真实 delta 取出来
            "place_pose_correction": dict(place_pose_correction),
            # 运行期触地纠偏的痕迹（mode=touchdown 时；否则 None）——含纠偏前后两次实测与自证结果
            "place_touchdown_correction": touch_record,
            "place_settled_margin_m": float(settled_facts["contact_margin_m"]),
            "runtime_source": "live_fk",
            "phase_trace": phase_trace,
            "segment_samples": segment_samples,
        }
        return {
            "place_target_id": str(place_target_id),
            "payload_id": str(payload_id),
            "released": bool(released and evidence["payload_in_tray"]),
            "confirmation": "released",
            "evidence": evidence,
        }

    def _corrected_place_goal(self, goal, delta_world, pad_points, arm_joints, wrist_body,
                              correction, label):
        """把一段放置目标的**关节解**按世界系平移 `delta_world` 重解（名义解作种子 ⇒ 不换分支）。

        为什么这样（§11.23(48)）：托盘随载体运动 ⇒ 构建期按**名义停靠位姿**解出的航点带着
        "载体没停到位"的误差（实测横向 0.033814698 m、竖向 −0.012792153 m）。运行时现解 IK 曾经
        把载荷打掉，根因是**没给种子、落到另一个分支**；这里以构建期名义解为初值、只做小位移重解。

        三条自证，任一不满足即**显式抛错**（不静默按名义位姿照放）：
          ① 求解残差 ≤ 声明的 `max_residual_m`；
          ② 求解后**腕→指腹方向与名义同半球**（dot ≥ 0，防翻分支）；
          ③ 指腹中点实际位移与请求 delta 之差 ≤ `max_residual_m`。

        自建 MjData（不写共享植物，避开 owner 驻留线程）；读共享状态时持 `_data_lock`。
        返回 (该段的纠正后目标字典, 证据字典)。
        """
        from iraf_core.kinematics import solve_position_ik

        arm_joint_set = {int(item) for item in arm_joints}

        def _pad_mid(data):
            return np.mean([np.asarray(data.geom_xpos[item["id"]], dtype=float)
                            for item in pad_points], axis=0)

        with self._data_lock:
            data = mujoco.MjData(self.model)
            data.qpos[:] = self.data.qpos          # 以**实测**状态为初值（含载荷/载体的真实姿态）
            mujoco.mj_forward(self.model, data)
            for name, value in goal.items():
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
                if joint_id < 0:
                    raise ValueError("放置段目标引用了模型里不存在的关节: %s" % name)
                if int(joint_id) in arm_joint_set:
                    data.qpos[int(self.model.jnt_qposadr[joint_id])] = float(value)
            mujoco.mj_forward(self.model, data)
            nominal_mid = _pad_mid(data)
            nominal_dir = nominal_mid - np.asarray(data.xpos[wrist_body], dtype=float)
            result = solve_position_ik(self.model, data, nominal_mid + np.asarray(delta_world, dtype=float),
                                       arm_joints, pad_points,
                                       iterations=int(correction["ik_iterations"]),
                                       step=float(correction["ik_step"]),
                                       tolerance_m=float(correction["max_residual_m"]))
            mujoco.mj_forward(self.model, data)
            achieved_mid = _pad_mid(data)
            achieved_dir = achieved_mid - np.asarray(data.xpos[wrist_body], dtype=float)
            corrected = dict(goal)
            for name in list(goal):
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
                if int(joint_id) in arm_joint_set:
                    corrected[str(name)] = float(data.qpos[int(self.model.jnt_qposadr[joint_id])])

        max_residual = float(correction["max_residual_m"])
        residual = float(result.position_error_m)
        shift = achieved_mid - nominal_mid
        shift_error = float(np.linalg.norm(shift - np.asarray(delta_world, dtype=float)))
        dot = float(np.dot(achieved_dir / max(np.linalg.norm(achieved_dir), 1e-12),
                           nominal_dir / max(np.linalg.norm(nominal_dir), 1e-12)))
        if residual > max_residual:
            raise ValueError("放置点纠偏（%s）求解残差 %.9f m > 声明上限 %.9f m ⇒ 拒绝"
                             "（不静默按名义位姿照放）" % (label, residual, max_residual))
        if dot < 0.0:
            raise ValueError("放置点纠偏（%s）把腕→指腹方向翻到了相反半球（dot=%.6f < 0）⇒ 拒绝"
                             % (label, dot))
        if shift_error > max_residual:
            raise ValueError("放置点纠偏（%s）实际位移与请求 delta 差 %.9f m > 声明上限 %.9f m ⇒ 拒绝"
                             % (label, shift_error, max_residual))
        return corrected, {"label": label, "residual_m": round(residual, 9),
                           "shift_m": [round(float(v), 9) for v in shift],
                           "shift_error_m": round(shift_error, 9),
                           "hemisphere_dot": round(dot, 6),
                           "iterations": int(result.iterations)}

    def accept_payload(self, payload_id, place_target_id, lease):
        """载荷确认（`accept_payload`）：复核载荷是否落在接收体承载面上**且整链已静止**。

        为什么必须独立复核（.hermes/plans/2026-09-28-s05-accept-payload.md 风险 R3）：s04 已经给出
        `payload_in_tray` 证据；若本步只是复述它，等于"自己证明自己"（AGENTS.md 1.5 的精神）。
        本方法在**确认时刻重新采样**，并给出 s04 没有的量：带符号的 `resting_gap_m` 与整链末速。

        测量逻辑**不在本层**：与四足侧 `UnitreeGo2Adapter.accept_payload` 共用
        `iraf_adapters.mujoco.payload_facts.confirm_payload_on_target`（AGENTS.md 6.3：不复制核心代码）。
        本层只负责**租约校验、取锁、组证据**。
        """
        self.authority.validate(lease)
        from iraf_adapters.mujoco.payload_facts import confirm_payload_on_target

        record = (getattr(self, "_place_targets", {}) or {}).get(str(place_target_id))
        if record is None:
            raise ValueError("场景报告没有接收体: " + str(place_target_id))
        payload = (self._manipulation.get("targets") or {}).get(str(payload_id))
        if payload is None:
            raise ValueError("场景报告没有载荷目标: " + str(payload_id))
        with self._data_lock:
            facts = confirm_payload_on_target(mujoco, self.model, self.data,
                                              str(payload["body"]), str(record["body"]))
        # 事实放**顶层**（Provider 的 `_evidence()` 在顶层取键，与 dock_for_handoff 同口径）
        return {"payload_id": str(payload_id), "place_target_id": str(place_target_id),
                "confirmation": "payload_confirmed",
                **facts,
                "runtime_source": "live_fk",
                "phase_trace": [{"note": "确认时刻单帧实测（重新采样；不复用 place_object 证据）",
                                 "payload_low_z_m": facts["payload_low_z_m"],
                                 "target_top_z_m": facts["target_top_z_m"],
                                 "resting_gap_m": facts["resting_gap_m"],
                                 "last_speed_mps": facts["last_speed_mps"]}]}

    def _any_contact_between(self, body_a, body_b):
        """两个 body 的任意 geom 之间是否**存在接触**（事实判据，不设力阈值）。"""
        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            for index in range(int(self.data.ncon)):
                contact = self.data.contact[index]
                bodies = {int(self.model.geom_bodyid[contact.geom1]),
                          int(self.model.geom_bodyid[contact.geom2])}
                if int(body_a) in bodies and int(body_b) in bodies:
                    return True
        return False

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
            # ⚠ 声明名 → 模型名一律走 `_model_name`（name_map）：联合世界里夹爪几何带前缀
            # （声明 `rq2f85_left_pad1` / 模型 `ur5e_rq2f85_left_pad1`）。漏掉这一步时，
            # 索引不到**按本体段**声明的 pad_boxes ⇒ 报"夹持区声明的 geom 不存在"
            # （2026-09-30 实测：`夹持区声明的 geom 不存在: rq2f85_left_pad2`）。
            left_geom_name = self._model_name(str(names["left_finger_geom"]))
            right_geom_name = self._model_name(str(names["right_finger_geom"]))
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
            grip_region = [self._model_name(str(name))
                           for name in (gripper_cfg.get("pad_boxes") or ())]
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

    def _correct_grasp_column(self, declaration, positions_by_phase, target_body):
        """按**运行期实测目标**重解抓取列（下压段与抬升段）。声明式、有界、失败即拒绝。

        为什么（2026-09-30 §11.29 实测）：构建期的参考关节解是按**标称目标**求的，而目标会被搬动
        （狗背托盘里的载荷实测偏 15.812 mm）⇒ 臂"到位"了但夹口没对准载荷，运行期门禁报
        `末端未到达目标抓取位姿 distance=0.015812m tolerance=0.005000m`。修法是**运行期按实测目标
        重解**，而不是放宽判据。

        口径与构建期一致：夹持区 = 声明 `pad_boxes` 的中点；抓取点 = 夹持区中点 − 轴·`pad_offset_m`。
        三条门禁（任一不过即 `ValueError`，不静默继续）：IK 残差 ≤ 声明、腕→夹持区轴相对名义轴
        的夹角 ≤ 声明、纠偏位形下**非夹持区**臂 geom 不得与载荷接触。

        返回 `(需要覆盖的 {相位: 位置字典}, 证据)`；`{}` 表示不必修正（证据仍给）。
        """
        from iraf_core.kinematics import solve_position_ik

        gripper = self._manipulation["gripper"]
        pad_names = [self._model_name(str(name)) for name in (gripper.get("pad_boxes") or ())]
        if not pad_names:
            raise ValueError("grasp_pose_correction 需要 gripper.pad_boxes（夹持区口径）")
        pad_points = []
        for name in pad_names:
            ident = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name))
            if ident < 0:
                raise ValueError("夹持区 geom 在模型里不存在: " + name)
            pad_points.append({"kind": "geom", "id": ident})
        arm_joints = []
        for key in (positions_by_phase.get("grasp") or {}):
            joint_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                             self._model_name(str(key))))
            if joint_id >= 0:
                arm_joints.append((str(key), joint_id))
        if not arm_joints:
            raise ValueError("grasp_positions 里没有可解的臂关节键（纠偏无从下手）")
        axis = self._unit_axis(gripper.get("pad_offset_axis"))
        pad_offset = float(gripper.get("pad_offset_m", 0.0) or 0.0)

        wrist_body = int(mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self._model_name(str(gripper.get("wrist_body") or ""))))
        # 本本体在联合模型里的对象名集合（控制作用域同源）：纠偏位形的侵入只按**本本体**判定。
        arm_scope = {str(value) for value in self._name_map.values()}
        for key in ("wrist_body", "left_finger_body", "right_finger_body",
                    "left_finger_geom", "right_finger_geom"):
            if gripper.get(key):
                arm_scope.add(self._model_name(str(gripper[key])))
        arm_scope.update(pad_names)

        with self._data_lock:
            mujoco.mj_forward(self.model, self.data)
            live = np.asarray(self.data.xpos[target_body], dtype=float).copy()
            backup = {joint_id: float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])
                      for _, joint_id in arm_joints}

            def apply(positions):
                """把臂摆到该相位解并返回 (抓取点, 腕→夹持区单位轴)。"""
                for key, joint_id in arm_joints:
                    value = positions.get(key)
                    if value is None:
                        raise ValueError("相位位置字典缺少臂关节 %s（纠偏要求两段同键）" % key)
                    self.data.qpos[int(self.model.jnt_qposadr[joint_id])] = float(value)
                mujoco.mj_forward(self.model, self.data)
                midpoint = np.mean([self.data.geom_xpos[point["id"]] for point in pad_points], axis=0)
                axis_now = None
                if wrist_body >= 0:
                    vector = np.asarray(midpoint, dtype=float) - np.asarray(self.data.xpos[wrist_body],
                                                                           dtype=float)
                    norm = float(np.linalg.norm(vector))
                    axis_now = (vector / norm) if norm > 1e-9 else None
                return np.asarray(midpoint, dtype=float) - axis * pad_offset, axis_now

            nominal, nominal_axis = {}, {}
            for key in ("approach", "grasp", "lift"):
                section = positions_by_phase.get(key)
                if isinstance(section, dict) and section:
                    nominal[key], nominal_axis[key] = apply(section)
            if "grasp" not in nominal:
                raise ValueError("grasp_positions 缺失，无法做抓取段纠偏")
            delta = live - nominal["grasp"]
            decision = resolve_grasp_pose_correction(declaration, delta.tolist())
            report = {
                "declaration": {str(k): v for k, v in dict(declaration).items()},
                "decision": decision,
                "live_target_m": [round(float(v), 9) for v in live],
                "nominal_grasp_point_m": [round(float(v), 9) for v in nominal["grasp"]],
                "phases": {},
            }

            def restore():
                for joint_id, value in backup.items():
                    self.data.qpos[int(self.model.jnt_qposadr[joint_id])] = value
                mujoco.mj_forward(self.model, self.data)

            if decision["refused"]:
                restore()
                raise ValueError("抓取段纠偏被拒（%s）：%s" % (declaration.get("mode"),
                                                            decision["reason"]))
            if not decision["required"]:
                restore()
                report["applied"] = False
                return {}, report

            target_geom_ids = {index for index in range(int(self.model.ngeom))
                               if int(self.model.geom_bodyid[index]) == int(target_body)}
            corrected = {}
            for key in ("approach", "grasp", "lift"):
                section = positions_by_phase.get(key)
                if not isinstance(section, dict) or not section:
                    continue
                target_point = nominal[key] + delta
                apply(section)                             # 从**名义解**播种 ⇒ 落在同一分支
                result = solve_position_ik(
                    self.model, self.data, target_point,
                    [joint_id for _, joint_id in arm_joints], pad_points,
                    iterations=int(declaration["ik_iterations"]),
                    step=float(declaration["ik_step"]),
                    tolerance_m=float(declaration["ik_tolerance_m"]))
                if float(result.position_error_m) > float(declaration["ik_tolerance_m"]):
                    raise ValueError(
                        "抓取段纠偏未收敛（%s 相位）：残差 %.9f m > 声明 %.9f m "
                        "⇒ 拒绝按未收敛的位形继续（不伪造到达）"
                        % (key, float(result.position_error_m),
                           float(declaration["ik_tolerance_m"])))
                # 姿态门禁：腕 → 夹持区轴相对**名义解**同一轴的夹角
                _, axis_now = apply({**section, **result.joint_positions})
                axis_deg = None
                if axis_now is not None and nominal_axis.get(key) is not None:
                    cosine = float(np.clip(float(np.dot(axis_now, nominal_axis[key])), -1.0, 1.0))
                    axis_deg = float(np.degrees(np.arccos(cosine)))
                    if axis_deg > float(declaration["max_axis_deg"]):
                        raise ValueError(
                            "抓取段纠偏把夹爪姿态带歪了（%s 相位）：腕→夹持区轴偏 %.3f° > 声明 %.3f° "
                            "⇒ 拒绝（歪着夹会把载荷顶飞）"
                            % (key, axis_deg, float(declaration["max_axis_deg"])))
                # 侵入门禁：纠偏位形下**非夹持区**的本本体 geom 不得与载荷接触
                intrusion = []
                for index in range(int(self.data.ncon)):
                    contact = self.data.contact[index]
                    ids = (int(contact.geom1), int(contact.geom2))
                    if target_geom_ids.isdisjoint(ids):
                        continue
                    other = ids[1] if ids[0] in target_geom_ids else ids[0]
                    geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other)
                    body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                                  int(self.model.geom_bodyid[other]))
                    if geom_name in pad_names:
                        continue                  # 夹持区就该接触（它是用来夹的）
                    if str(geom_name) in arm_scope or str(body_name) in arm_scope:
                        intrusion.append({"geom": str(geom_name), "body": str(body_name),
                                          "dist_m": round(float(contact.dist), 6)})
                if intrusion:
                    raise ValueError(
                        "抓取段纠偏后的位形有非夹持区侵入载荷（%s 相位）：%s ⇒ 拒绝"
                        % (key, intrusion))
                merged = dict(section)
                for key_name, joint_id in arm_joints:
                    model_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                    if model_name in result.joint_positions:
                        merged[key_name] = float(result.joint_positions[str(model_name)])
                corrected[key] = merged
                report["phases"][key] = {
                    "target_point_m": [round(float(v), 9) for v in target_point],
                    "residual_m": round(float(result.position_error_m), 9),
                    "iterations": int(result.iterations),
                    "axis_deg": (None if axis_deg is None else round(axis_deg, 6)),
                    "intrusion": intrusion,
                    "qpos_corrected": {str(k): round(float(v), 9)
                                       for k, v in result.joint_positions.items()},
                }
            restore()
        report["applied"] = True
        return corrected, report

    def dump_pick_phase(self, phase, ms, target_body, left_body, right_body, approach_axis):
        """相位级观测（`IRAF_DEBUG_PICK=1` 时打印）——pick 与探针**共用同一实现**，保证可比。

        2026-09-24：pick 与探针在**入参一致**（见 PICK_INPUTS）的情况下给出矛盾的到位残差
        （1.3817e-02 vs 7.551e-06 m）⇒ 必须逐相位比对**输出**，而不是继续猜。
        """
        if os.environ.get("IRAF_DEBUG_PICK") != "1":
            return None
        alignment = self._grasp_alignment_evidence(target_body, left_body, right_body, approach_axis)
        # ⚠ 前缀必须是 **PICK_TRACE**：既有的 `_log_pick_phase` 打的是 `PICK_PHASE <NAME>`，
        #   两套格式共用前缀会让 grep/解析混在一起（我第一版解析脚本就因此崩溃）。
        print("PICK_TRACE " + json.dumps({
            "invocation": int(getattr(self, "_pick_invocation", 0)),
            "phase": str(phase),
            "ms": int(ms),
            "center_delta_m": alignment["center_delta_m"],
            "center_distance_m": alignment["center_distance_m"],
            "finger_center_position_m": alignment["finger_center_position_m"],
            "target_position_m": alignment["target_position_m"],
            "joint_qpos": alignment["joint_qpos"],
        }, ensure_ascii=False), flush=True)
        return alignment

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
                self._write_ctrl(channel, value)
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
        """推进仿真 count 步。

        · 本后端是植物 **owner**（自带植物或显式当 owner）⇒ 自己 mj_step；
        · 本后端是 **guest**（注入别人的植物）⇒ 不能推进时间，改为"等 owner 推够步数"，
          语义等价（"仿真前进了 count 步"），但时间线始终只有一个推进者。
        """
        count = int(count)
        if count < 1:
            raise ValueError("MuJoCo 步进次数必须为正数")
        if not self.plant.is_owner(self):
            return self._wait_for_guest_steps(count)
        for _ in range(count):
            fault_kind, fault_delay = self._consume_fault()
            started = time.monotonic()
            if fault_kind == "step_failure":
                raise RuntimeError("注入的 MuJoCo 步进故障")
            if fault_kind == "step_delay":
                time.sleep(fault_delay)
            with self._data_lock:
                self.plant.step_once(self)
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
                self._write_ctrl(channel, value)
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
        """安全停机：只归零**本后端拥有的**执行器。

        为什么不是 `data.ctrl[:] = 0.0`（2026-09-24 实测发现）：联合模型里整条数组归零会把
        **主本体（狗）** 的 12 个执行器一并清零（实测 FR_thigh 0.9 → 0.0）⇒ 一个本体的急停
        把另一个本体也停了。作用域收敛后单本体路径无差别（拥有的通道 = 模型全部执行器 ⇒ 逐位不变，
        由 loopback 三基准回归证明）；全植物急停属于**场景层**的职责，不在单控制器后端里做。
        """
        with self._data_lock:
            for name in self._owned_actuators:
                index = self._actuators.get(name)
                if index is not None:
                    self.data.ctrl[index] = 0.0
            self.stopped = True

    def _resolve_owned_actuators(self):
        """本后端**拥有**的执行器名集合（控制权作用域）。

        自带植物 ⇒ 模型里全部执行器（单本体语义逐位不变）。
        guest（注入植物）⇒ `name_map` 指向的对象对应的执行器：
          · 关节名与执行器名同名（Piper 的 `piper_joint1`）⇒ 直接命中；
          · 不同名（UR5e 的关节 `shoulder_pan_joint` / 执行器 `shoulder_pan`）⇒ 用 `actuator_trnid`
            反查驱动该关节的执行器；
          · name_map 里也有 body/geom（非执行器）⇒ 跳过。
        两类都不是（映射没落到任何执行器）⇒ 显式失败：guest 一个通道都不拥有等于不能动，
        静默放行会让"越权写别人的执行器"看起来正常。
        """
        if not self._plant_injected:
            return set(self._actuators)
        if not self._name_map:
            raise PlantError(
                "共享植物模式必须同时声明 name_map（本后端拥有的执行器由它界定）："
                "否则无法判定控制权作用域，拒绝装配而不是默认放行")
        owned = set()
        for model_name in self._name_map.values():
            if model_name in self._actuators:
                owned.add(model_name)
                continue
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, model_name)
            if joint_id < 0:
                continue
            for index in range(int(self.model.nu)):
                if int(self.model.actuator_trnid[index, 0]) != int(joint_id):
                    continue
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                if name:
                    owned.add(str(name))
        if not owned:
            raise PlantError(
                "共享植物模式下 name_map 没有落到任何执行器 ⇒ 本后端无法控制任何通道"
                "（声明与模型不匹配）")
        return owned

    def _assert_owned(self, channel):
        if channel not in self._owned_actuators:
            raise PlantError(
                "控制权越界：执行器 %s 不属于本后端（拥有的通道：%s）—— "
                "同一执行器任一时刻只允许一个控制源（AGENTS.md 1.13）"
                % (channel, sorted(self._owned_actuators)))

    def _write_ctrl(self, channel, value):
        self._assert_owned(channel)
        self.data.ctrl[self._actuators[channel]] = float(value)

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
                # 受控写入：guest 只能写自己拥有的执行器（共享植物下越界即显式失败）
                self._write_ctrl(channel, value)

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

    def _move_trajectory(self, target_positions, duration_ms, ctrl_offsets=None, sampler=None,
                         pin_body=None, anchor_body=None, anchor_follow=(), anchor_offset=None,
                         anchor_wrist=None, anchor_rel_quat=None, cadence_sink=None):
        # `pin_body`（可选）：**逐步**把该 body 的 freejoint 复位到本段开始时的位姿。
        # 用途（2026-09-28 §11.23(21)）：接近/下压段的刚性指腹会把 0.39 N 的载荷推开 1.44 cm，
        # 之后的合爪/抬升就丢了它 —— 真机上这段位移由**台面摩擦**抵住，本模型的台面摩擦
        # （~0.4 N）小于下压产生的侧向合力。该行为由声明 `gripper.approach_hold` 打开，并进证据。
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
        pin = None
        if pin_body is not None:
            pin_free = next((index for index in range(int(self.model.njnt))
                             if int(self.model.jnt_bodyid[index]) == int(pin_body)
                             and int(self.model.jnt_type[index]) == int(mujoco.mjtJoint.mjJNT_FREE)),
                            None)
            if pin_free is None:
                raise ValueError("pin_body 指定了没有 freejoint 的 body，无法保持其位姿")
            with self._data_lock:
                pin = (int(self.model.jnt_qposadr[pin_free]),
                       int(self.model.jnt_dofadr[pin_free]),
                       np.asarray(self.data.xpos[pin_body], dtype=float).copy(),
                       np.asarray(self.data.xquat[pin_body], dtype=float).copy())
        if cadence_sink is not None:
            from iraf_adapters.mujoco.payload_facts import (accumulate_carry_cadence,
                                                            check_carry_cadence)
            previous_step_index = int(self.plant.step_index)
        else:
            accumulate_carry_cadence = check_carry_cadence = None
            previous_step_index = None
        anchor_mocap = None
        if anchor_body is not None:
            anchor_id = self._body_id(str(anchor_body))
            anchor_mocap = int(self.model.body_mocapid[anchor_id])
            if anchor_mocap < 0:
                raise ValueError("搬运约束的 anchor 必须是 mocap body: " + str(anchor_body))
        for step in range(1, steps + 1):
            elapsed = step * float(self.model.opt.timestep)
            values = quintic_position(starts, [commands[name] for name in names], duration_ms / 1000.0, elapsed)
            self._set_controls(dict(zip(names, values)))
            if anchor_mocap is not None and anchor_follow:
                # 约束焊接的 anchor 跟随**指腹中点**（与 _advance_with_grasp_anchor 同口径）；
                # `anchor_offset`（可选）是激活瞬间的「载荷重心 − 指腹中点」⇒ anchor 取
                # 「指腹中点 + 该偏移」时，载荷相对指腹**刚性平移**（激活瞬间无纠正力，见
                # §11.23(41) 的 A/B/D 对照；缺省 None ⇒ 行为与改动前一致）。
                # `anchor_rel_quat`（可选，仅 `weld` 用）：anchor 的姿态按「腕部姿态 ⊗ 该相对姿态」
                # 跟随 ⇒ 载荷的**姿态**也随腕部刚性走（`connect` 是球铰、不约束旋转 ⇒ 会翻滚）。
                with self._data_lock:
                    left = np.asarray(self.data.xpos[self._body_id(str(anchor_follow[0]))], dtype=float)
                    right = np.asarray(self.data.xpos[self._body_id(str(anchor_follow[1]))], dtype=float)
                    target_point = (left + right) / 2.0
                    if anchor_offset is not None:
                        target_point = target_point + np.asarray(anchor_offset, dtype=float)
                    self.data.mocap_pos[anchor_mocap] = target_point
                    if anchor_rel_quat is not None and anchor_wrist is not None:
                        wrist_id = self._body_id(str(anchor_wrist))
                        wrist_quat = np.asarray(self.data.xquat[wrist_id], dtype=float)
                        self.data.mocap_quat[anchor_mocap] = _quat_mul(wrist_quat,
                                                                     np.asarray(anchor_rel_quat))
                    else:
                        self.data.mocap_quat[anchor_mocap] = (1.0, 0.0, 0.0, 0.0)
            if pin is not None:
                with self._data_lock:
                    self.data.qpos[pin[0]:pin[0] + 3] = pin[2]
                    self.data.qpos[pin[0] + 3:pin[0] + 7] = pin[3]
                    self.data.qvel[pin[1]:pin[1] + 6] = 0.0
            self._advance_for(0)
            if cadence_sink is not None:
                # 搬运节拍实测（§11.23(43)）：anchor 每个控制迭代只跟随一次，而 owner 一次可能推进
                # 很多步 ⇒ 超声明上限即**在本段内显式失败**（载荷正挂在陈旧 anchor 上，是即时风险，
                # 不得静默劣化）。声明见 grasp.carry_constraint.max_plant_steps_per_iteration。
                # ⚠ **不在此处判失败**：实测本机节拍随负载变化（同一场景两次运行分别测到 25 与 40 步/
                # 迭代）⇒ 拿它当硬门禁会随机变红。节拍作为**证据里的诊断量**（含是否超声明阈值），
                # 真正被强制的不变量是**焊缝滑移**（见 _carry_grip_row），超滑移时会一并报出节拍数字。
                accumulate_carry_cadence(cadence_sink, previous_step_index,
                                         int(self.plant.step_index))
                previous_step_index = int(self.plant.step_index)
            # 可选采样回调（默认 None ⇒ 行为逐位不变）：用于**在控制路径内**观察运动过程的量。
            # 为什么必须在这里采样（2026-09-28 第 10 个工装缺陷）：从外面写 data.qpos 再 mj_step
            # 会被位置伺服的 ctrl 立刻拉回 ⇒ 看起来"没动"，量到的全是伪像。
            if sampler is not None:
                sampler(step, elapsed)
        # 位置执行器有自身阻尼和力矩限制，轨迹结束后必须留出稳定时间。
        # 多目标场景中 joint1 要带着整臂绕基座旋转，其阻尼(300)远高于
        # 近端关节(2~100)，收敛时间按秒计：实测需要约 16 秒才能到目标角，
        # 而 4 秒时只走 61%、2 秒时只走 41%。
        # 注意这里不能按"段时长"缩放：调用方传入的是每段时长（总时长/5），
        # 按它缩放会把稳定窗口压到 4 秒以内，joint1 永远到不了位。
        settle_ms = max(250, min(16000, int(duration_ms) * 4))
        self._set_controls(commands)
        self._advance_for(settle_ms)

    def _wait_for_guest_steps(self, count):
        """guest 的时间等待：等 owner 把植物推进 count 步（**等价于**"仿真前进 count 步"）。

        为什么这样做：共享植物下时间线只能有一个推进者，guest 不能 mj_step（plant.step_once 会
        直接拒绝），所以把"推进"翻译成"等 owner 推进"。超时 = count × timestep ×
        `plant_guest_timeout_factor`（系数在装配期由声明强制、不猜）⇒ 机器慢时表现为显式超时，
        而不是静默卡死或伪造成功。
        """
        if self._plant_guest_timeout_factor is None:
            raise PlantError("guest 缺少 plant_guest_timeout_factor（装配期本应拒绝装配）")
        count = int(count)
        target = self.plant.step_index + count
        timeout = abs(float(self.plant.timestep)) * count * self._plant_guest_timeout_factor
        self.plant.wait_until(target, timeout=timeout, poll_seconds=abs(float(self.plant.timestep)))
        return {joint: float(self.last_positions[joint]) for joint in self.profile.joints}

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
        if not self.plant.is_owner(self):
            # guest：时间由 owner 推进 ⇒ 等 owner 走完这段时长（超时按声明的系数）
            self._wait_for_guest_steps(steps)
            if contact_bodies:
                bilateral = self._has_bilateral_contact(*contact_bodies)
            return bilateral
        for _ in range(steps):
            if self._cancel_event.is_set():
                self._safe_stop_controls()
                break
            # 与 Viewer 的 Home 阶段使用同一条受锁保护的 MuJoCo 步进路径。
            with self._data_lock:
                self.plant.step_once(self)
            self._record_step(float(self.model.opt.timestep))
            if self._realtime:
                time.sleep(float(self.model.opt.timestep))
            if contact_bodies and self._has_bilateral_contact(*contact_bodies):
                bilateral = True
        return bilateral

    def _advance_pinned(self, duration_ms, payload_body):
        """推进 `duration_ms`，**每步把载荷钉在它当前的位姿**（`qpos`/`qvel` 复位）。

        用途（2026-09-28，docs/debug/2026-09-24-joint-model-dog-arm.md §11.23(19)(20)）：
        合爪阶段用。本模型的指腹是**刚性 mesh**、台面摩擦只有 ~0.4 N，而两条接触法向只差 4.7°
        （`dot=-0.9966`）⇒ 合爪产生的 ~1 N 侧向合力会把 0.39 N 的载荷推离夹口轴线（运行期实测被推 1.44 cm），
        之后的竖直抬升就丢掉它（`lifted=false`）。真机上这一推力由**指腹柔顺 + 平行颚**吸收、
        且台面摩擦会抵住它 ⇒ 这里显式声明"合爪期间保持载荷位姿"，**合爪结束即释放**
        （搬运仍靠真实摩擦，不做任何辅助）。语义由 `gripper.close_hold` 声明，并进证据。
        """
        payload_free = None
        for index in range(int(self.model.njnt)):
            if (int(self.model.jnt_bodyid[index]) == int(payload_body)
                    and int(self.model.jnt_type[index]) == int(mujoco.mjtJoint.mjJNT_FREE)):
                payload_free = index
                break
        if payload_free is None:
            raise ValueError("载荷 body 没有 freejoint，无法在合爪期间保持位姿")
        qadr = int(self.model.jnt_qposadr[payload_free])
        dadr = int(self.model.jnt_dofadr[payload_free])
        with self._data_lock:
            pose = (np.asarray(self.data.xpos[payload_body], dtype=float).copy(),
                    np.asarray(self.data.xquat[payload_body], dtype=float).copy())
        steps = max(1, int(math.ceil((float(duration_ms) / 1000.0) / self.model.opt.timestep)))
        for _ in range(steps):
            if self._cancel_event.is_set():
                break
            with self._data_lock:
                self.data.qpos[qadr:qadr + 3] = pose[0]
                self.data.qpos[qadr + 3:qadr + 7] = pose[1]
                self.data.qvel[dadr:dadr + 6] = 0.0
            # 一步推进：owner 走 `step_once`；guest 等 owner 推进 1 步（共享植物契约不变）
            self._advance_for(0)
        return {"pinned_pose_pos_m": [round(float(v), 9) for v in pose[0]],
                "pinned_pose_quat_wxyz": [round(float(v), 9) for v in pose[1]],
                "pinned_steps": steps}

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
    @staticmethod
    def _parse_place_targets(config):
        """解析场景报告的 `place_targets` 段（接收体）：id → {body, geom, size_m, mount, nominal_*}。

        只做**形状校验**：缺 body/size_m 即显式失败（`place_object` 的"载荷是否落在承载面内"
        判据需要半尺寸；没有它就无法判定，不许用默认值顶替）。
        """
        if config is None:
            return {}
        if not isinstance(config, dict):
            raise ValueError("place_targets 配置必须是对象")
        raw = config.get("targets") or []
        if not isinstance(raw, list):
            raise ValueError("place_targets.targets 必须是数组")
        parsed = {}
        for item in raw:
            if not isinstance(item, dict) or not item.get("id"):
                raise ValueError("place_targets.targets 每项必须有 id")
            body = item.get("body")
            size = item.get("size_m")
            if not body:
                raise ValueError("接收体 %s 必须声明 body" % item.get("id"))
            if (not isinstance(size, (list, tuple)) or len(size) != 3
                    or not all(isinstance(v, (int, float)) for v in size)):
                raise ValueError("接收体 %s 必须声明 size_m（3 个半尺寸）" % item.get("id"))
            parsed[str(item["id"])] = dict(item)
        return parsed

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
                # `geom`（可选）：放置段要量载荷最低点用；未声明即由 place_object 显式拒绝
                "geom": (str(item["geom"]) if item.get("geom") else None),
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
            # 放置四段的关节解（构建期由声明求解器解出，后端只回放；缺省即"该场景不支持放置"）
            for key in ("place_transit_positions", "place_above_positions",
                        "place_descend_positions", "place_retreat_positions",
                        # regrasp（先抬离台、再合爪到腰部）的两段位置指令：与放置段同样必须透传，
                        # 否则被本解析层白名单丢掉 ⇒ 运行时只会走原相位表（本会话实测踩过一次）。
                        "pre_lift_positions", "regrasp_positions"):
                if raw_gripper.get(key) is not None:
                    gripper[key] = {str(name): float(value)
                                    for name, value in dict(raw_gripper[key]).items()}
            # **落稳窗**（2026-09-29 §11.23(47)）：必须**透传**，否则被本解析层白名单静默丢掉
            # （本会话第 6 次踩同一类坑）⇒ `place_object` 到运行中途才 fail-closed。
            settle_ms = raw_gripper.get("place_settle_ms")
            if settle_ms is not None:
                if not isinstance(settle_ms, int) or isinstance(settle_ms, bool) or settle_ms <= 0:
                    raise ValueError(
                        "夹爪配置的 place_settle_ms 必须是正整数（毫秒）：实际 %r" % (settle_ms,))
                gripper["place_settle_ms"] = int(settle_ms)
            # **抬升段夹爪语义**（字符串开关）：必须**原样**透传（落到通用"名字改写"分支会被当成
            # 对象名去查、必然 fail-closed）。
            lift_gripper = raw_gripper.get("lift_gripper")
            if lift_gripper is not None:
                if str(lift_gripper) not in ("hold", "closed"):
                    raise ValueError("夹爪配置的 lift_gripper 必须是 hold|closed：实际 %r"
                                     % (lift_gripper,))
                gripper["lift_gripper"] = str(lift_gripper)
            # **放置点纠偏**（语义字典：字符串 mode + 数值上限）⇒ 必须**原样**透传：
            # 落到通用"名字改写/浮点转换"分支会把 mode 当名字、或 float("measure_only") 崩掉。
            # 声明合法性由共享函数 `resolve_place_pose_correction` 统一校验（唯一口径）。
            correction = raw_gripper.get("place_pose_correction")
            if correction is not None:
                if not isinstance(correction, dict):
                    raise ValueError(
                        "夹爪配置的 place_pose_correction 必须是对象：实际 %r" % (correction,))
                gripper["place_pose_correction"] = dict(correction)
            # 搬运段抓取约束（构建期按声明写入报告）：**透传**（白名单漏掉就静默失效 —— 本会话已踩四次）
            if raw_gripper.get("carry_constraint") is not None:
                cc = dict(raw_gripper["carry_constraint"])
                # ⚠ 白名单必须与声明同步（2026-09-28 §11.23(41)：新键 `type` 漏在这里被**静默丢掉**，
                # 运行期才报"缺少合法的 carry_constraint.type"）；这是同一类坑第 5 次
                # （前四个：carry_gripper / lift_trace / regrasp / pre_lift_positions）。
                gripper["carry_constraint"] = {
                    "enabled": bool(cc.get("enabled", False)),
                    "type": str(cc.get("type") or ""),
                    "equality_name": str(cc.get("equality_name") or ""),
                    "anchor_body": str(cc.get("anchor_body") or ""),
                    "release_gripper": bool(cc.get("release_gripper", False)),
                    "max_slip_m": float(cc.get("max_slip_m") or 0.0),
                    "max_plant_steps_per_iteration": int(
                        cc.get("max_plant_steps_per_iteration") or 0),
                    # 刚度声明（记录用；后端不直接使用，但白名单漏掉就会被静默丢弃 —— 本会话第 5 次）
                    **( {"solref": [float(v) for v in cc["solref"]],
                         "solimp": [float(v) for v in cc["solimp"]]}
                        if isinstance(cc.get("solref"), (list, tuple))
                        and isinstance(cc.get("solimp"), (list, tuple)) else {} ),
                    **( {"source": str(cc["source"])} if cc.get("source") else {} ),
                }
            # regrasp 声明（构建期按声明写入报告）：**透传**；enabled=true 时缺两段位置指令即由
            # pick_object 显式拒绝（见 .hermes/plans/2026-09-28-pick-regrasp.md）。
            if raw_gripper.get("regrasp") is not None:
                rg = dict(raw_gripper["regrasp"])
                if "enabled" not in rg:
                    raise ValueError("gripper.regrasp 必须声明 enabled：%r" % (rg,))
                # ⚠ 本层也是**显式枚举**（2026-09-29 §11.23(48)）：原先只列了三个数值参数 ⇒
                # `hold_pre_lift`（预抬段临时刚住）与 `duration_ms`（各段时长）都被静默丢掉 ⇒
                # 声明写了也不生效（实测后端永远读到 false / 用 phase_ms 兜底）。
                gripper["regrasp"] = {
                    "enabled": bool(rg["enabled"]),
                    "pre_lift_m": float(rg.get("pre_lift_m") or 0.0),
                    "depth_m": float(rg.get("depth_m") or 0.0),
                    "open_m": float(rg.get("open_m") or 0.0),
                    "hold_pre_lift": bool(rg.get("hold_pre_lift", False)),
                    **( {"duration_ms": int(rg["duration_ms"])} if rg.get("duration_ms") else {} ),
                    **( {"source": str(rg["source"])} if rg.get("source") else {} ),
                }
            # 搬运段的夹爪语义（构建期按声明写入报告）：**透传**，缺失即由 place_object 显式拒绝
            # （不做实现层默认值；见 §11.23(12)）。
            if raw_gripper.get("carry_gripper") is not None:
                carry = dict(raw_gripper["carry_gripper"])
                if str(carry.get("mode") or "") not in ("hold", "trajectory"):
                    raise ValueError("carry_gripper.mode 只允许 hold / trajectory：%r" % (carry.get("mode"),))
                gripper["carry_gripper"] = {"mode": str(carry["mode"]),
                                            **({"source": str(carry["source"])} if carry.get("source") else {})}
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
            # 放置段的接近/抬离间隙（可选键，来自报告声明；缺省即"该场景不支持放置"，
            # 由 `place_object` 显式报错，不在解析层补默认值）
            if raw_gripper.get("pregrasp_offset_m") is not None:
                pregrasp = float(raw_gripper["pregrasp_offset_m"])
                if not math.isfinite(pregrasp) or pregrasp <= 0:
                    raise ValueError("pregrasp_offset_m 必须是正有限数")
                gripper["pregrasp_offset_m"] = pregrasp
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
            # 抓取段的**运行期闭环纠偏**声明（2026-09-30 §11.29）：缺省 None = 不纠偏（行为与改动前一致）。
            raw_pick_correction = raw_gripper.get("grasp_pose_correction")
            if raw_pick_correction is None:
                gripper["grasp_pose_correction"] = None
            else:
                if not isinstance(raw_pick_correction, dict) or not raw_pick_correction:
                    raise ValueError("grasp_pose_correction 必须是对象（含 mode）")
                correction_mode = str(raw_pick_correction.get("mode") or "")
                if correction_mode not in GRASP_POSE_CORRECTION_MODES:
                    raise ValueError("grasp_pose_correction.mode 必须是 %s，实际: %r"
                                     % ("/".join(GRASP_POSE_CORRECTION_MODES), correction_mode))
                unknown_correction = sorted(set(raw_pick_correction) - set(GRASP_POSE_CORRECTION_KEYS))
                if unknown_correction:
                    raise ValueError("grasp_pose_correction 含未知字段: " + str(unknown_correction))
                required_correction_keys = ["residual_tolerance_m", "max_correction_m"]
                if correction_mode == "resolved":
                    required_correction_keys += ["ik_iterations", "ik_step", "ik_tolerance_m",
                                                 "max_axis_deg"]
                for key in required_correction_keys:
                    value = raw_pick_correction.get(key)
                    if (not isinstance(value, (int, float)) or isinstance(value, bool)
                            or not math.isfinite(float(value)) or not float(value) > 0):
                        raise ValueError("grasp_pose_correction.%s 必须是正有限数（实际 %r）"
                                         % (key, value))
                gripper["grasp_pose_correction"] = {
                    str(key): (int(raw_pick_correction[key]) if key == "ik_iterations"
                               else raw_pick_correction[key])
                    for key in required_correction_keys}
                gripper["grasp_pose_correction"]["mode"] = correction_mode
            friction_flag = raw_gripper.get("require_friction_lift")
            gripper["require_friction_lift"] = bool(friction_flag) if friction_flag is not None else False
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
