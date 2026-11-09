# 场景注册表使用手册（阶段 A，0.4.2）

> 口径：`simulation=true`。本文只讲"怎么加/查/排障"，契约细节见 `plans/iraf-single-controlplane/01-contracts.md`。

## 1. 事实来源

| 事实 | 位置 | 说明 |
|---|---|---|
| 场景 | `/etc/iraf/scenes/<scene_id>.yaml` | 一场景一文件 |
| Provider（主机） | `/etc/iraf/providers/iraf-<slug>.yaml` | 一 provider 一文件，文件名 = `provider_id` 的 `/`→`-` |
| 校验器 | `/root/iraf-instance/scripts/registry_check.py` | 或仓库 `scripts/registry_check.py` |
| 库 | `src/iraf_core/scene_registry.py` | 规则唯一实现 |

## 2. 查

```bash
# 全量校验（板卡）
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf \
  --profiles-root /opt/iraf-sdk/0.4.1/profiles
# OK scenes=4 providers=4

# 机器可读
python3 ... --json

# 运行期（MCP 工具）
iraf_scene_status(scene_id="piper-pick")
# -> mode / allowed_hosts / resolved{scene,provider,host,endpoint,resource} / scenes[] / providers[]
```

## 3. 加一个场景（只加 1 个文件）

1. 新文件 `/etc/iraf/scenes/<new-id>.yaml`：

```yaml
schema: iraf.scene/v1
scene_id: my-scene            # 必须等于文件名
provider_id: iraf/my-scene    # 必须已有同名 provider 文件
profile: profiles/piper_mujoco.yaml
safety_policy: profiles/safety/simulation_lab.yaml
simulation: true
max_concurrency: 1
lease_ttl_s: 60
resources:
  - id: piper
    profile: profiles/piper_mujoco.yaml
    capabilities: [move_joint, pick_object, visual_pick, place_object, stop, reset_scene]
```

2. 对应 provider `/etc/iraf/providers/iraf-my-scene.yaml`（`endpoint` 的 host 必须在 `allowed_hosts` 内）。
3. 原子写入：`写 .tmp → mv`（不要原地编辑），然后 `--seal` 重算 digest：

```bash
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --seal
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf
```

4. **不需要重启任何服务**：MCP 在下次调用时按 mtime/size 变化热加载（失败保留旧版本并在 `iraf_scene_status.registry.errors` 里报错）。

## 4. 加一台主机（多主机）

1. 在对应 provider 文件的 `hosts:` 增加一项（`id` 与 `priority` 唯一）：

```yaml
hosts:
  - id: x86-1
    endpoint: 10.203.247.121:50054
    capacity: {cores: 4, gpu: none}
    priority: 10
  - id: x86-2
    endpoint: 192.168.100.128:50054
    capacity: {cores: 16, gpu: none}
    priority: 20
```

2. 把新主机加进 `allowed_hosts`（**两个列表必须一致，否则校验失败**）；
3. 调度：`schedule: static`（priority 最小优先）/ `least-loaded`（按 Health 的 `capacity.load`，缺字段降级 static）；
4. 不允许扫描网段；新增主机必须走数据文件。

## 5. 三档开关（阶段 A 的灰度手段）

| 值 | 行为 | 用途 |
|---|---|---|
| `off`（默认） | 完全不介入，旧行为 | 生产默认 |
| `shadow` | 解析并写审计 `resolved_scene`，**不改变行为** | 灰度核对"解析是否与旧行为一致" |
| `on` | 解析 + 校验 env 与注册表一致（不一致返回 `IRAF-REGISTRY-MISMATCH`） | 阶段 B 之前的过渡 |

配置：各单元 env 里 `IRAF_SCENE_REGISTRY_MODE`；默认场景 `IRAF_DEFAULT_SCENE`。
**注意**：每个单元必须显式设置 `IRAF_ENV_FILE`（已在 ExecStart 前缀里），否则会回落到
`/etc/iraf/edge-intel.env` 造成"一个端点的开关泄漏到全部端点"（2026-11-09 实测）。

## 6. 排障对照

| 现象 | 先看 | 常见原因 |
|---|---|---|
| `IRAF-SCENE-NOT-FOUND` | `iraf_scene_status` 的 `scenes[]` | scene_id 拼错/文件没加载（digest 不符会拒绝） |
| `IRAF-REGISTRY-INVALID` | `registry.errors[]` 与 `registry_check` 输出 | schema/digest/交叉引用/白名单 |
| `IRAF-REGISTRY-MISMATCH` | env 与 `resolved.endpoint` | env 指向别处；以注册表为准修 env |
| `IRAF-PROVIDER-HOST-REJECTED` | `allowed_hosts` | endpoint 主机不在白名单 |
| 抓取 `distance=0.196m` 类失败 | 上一把是否已把目标搬走 | **场景有状态**：重跑前 `switch_scene.sh <scene>` 复位 Provider |

## 7. 当前已登记的 4 个场景

| scene_id | provider | endpoint | 能力 |
|---|---|---|---|
| `piper-pick` | `iraf/piper-pick` | `.121:50052` | piper 6 能力 |
| `piper-pick-y05` | `iraf/piper-pick-y05` | `.121:50055` | piper 6 能力（box Y+5mm） |
| `ur5-pick` | `iraf/ur5-pick` | `.121:50053` | ur5 6 能力 |
| `joint-handoff` | `iraf/handoff-lab` | `.121:50054` | piper/ur5/go2 资源（9 步链路） |