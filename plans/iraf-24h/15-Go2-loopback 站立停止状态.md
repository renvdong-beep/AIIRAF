# 15 Go2 loopback 站立/停止/状态

- 状态：TODO　　预估：2~3 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §8（U2）；ADR-0004 验证门禁 2

## 目标
让 Go2 在 MuJoCo 里真的站得住、停得下、状态读得到，并留下可复算的数字。

## 前置
步骤 13、14 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`scripts/verify_go2_loopback.py`
- 新增：`config/go2_loopback.yaml`
- 新增：`tests/unit/test_go2_loopback_contract.py`

## 步骤
1. `config/go2_loopback.yaml` 声明：目标躯干高度、站立时长、姿态容差、速度容差、PD 增益、站立姿态、采样频率。**增益与目标值只能在配置里**。
2. 控制器：按声明对 12 个 `<motor>` 施加 PD + 重力前馈（Go2 执行器是力矩型，`ctrlrange` 实测 `-23.7/23.7` 与 knee `-45.43/45.43`）；前馈与增益从模型/配置读取，不写死。
3. `stop`：控制量归零 + 速度衰减判定；`stop` 后必须报告是否进入静止（速度 < 容差）。
4. 状态读取：躯干位姿/速度、12 关节角速度、IMU 读数、力矩传感器读数，按固定频率采样并统计。
5. 输出 `build/acceptance/go2-loopback/report.json`：稳定段躯干高度均值/标准差、最大姿态偏差、末段速度、站立保持时长、`simulation: true`。
6. 判据来自配置；未达标即 FAILED（不得调容差凑数，只能在调试记录里分析原因）。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/verify_go2_loopback.py --config config/go2_loopback.yaml; echo "exit=$?"   # 期望 exit=0
python3 -c "import json;d=json.load(open('build/acceptance/go2-loopback/report.json'));print(d['passed'],d['stand']['height_mean_m'],d['stand']['height_std_m'],d['stand']['hold_seconds'],d['stop']['final_speed_mps'])"
PYTHONPATH=src python3 -m unittest tests.unit.test_go2_loopback_contract -v
```
报告必须包含真实数字（不得出现 null/NaN）；站立时长 ≥ 配置声明的 `hold_seconds`。

## 证据落盘
`build/iraf-24h/15/loopback.txt`

## 提交信息
`feat: 新增 Go2 MuJoCo loopback 站立与状态验收`

## 失败 / 阻塞处理
若 PD 参数调不稳：先在调试记录里量化（高度振荡幅度、发散时间），并把参数扫描结果写进证据，**不得**修改判据或删掉失败用例。
