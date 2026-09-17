"""engineering.* 能力命名空间：构建期工件生成类 skill 的集合。

本包的 Provider 只产出构建期工件（配置文件、适配骨架、校验报告），
不驱动机器人、不申请运动租约，因此与 iraf_skills.common 下的运动 Provider
在能力命名空间上严格隔离（见 AGENTS.md 铁律 2 与 skill.yaml 注释）。
"""
