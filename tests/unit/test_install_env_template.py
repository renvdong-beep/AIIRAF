# -*- coding: utf-8 -*-
"""edge-intel 运行装配单模板的渲染用例（2026-10-16）。

要证的三件事：
  ① 声明了 `env_template` 的角色 ⇒ 走模板渲染，部署期值（--env-set 对应的 env_values）被填入；
  ② 缺必需占位符 ⇒ **显式失败**（不写半个文件、不猜默认值）；
  ③ 未声明模板的角色 ⇒ 沿用原来的程序化生成（既有行为不回归）。
另外明确一条安全性质：**凭据绝不进 env 文件**（token 只由部署环境注入）。
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]

_spec = importlib.util.spec_from_file_location(
    "iraf_lib_install", REPO / "deploy" / "sdk" / "lib_install.py")
lib = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lib)


def _tokens(profile_id):
    effective = lib.effective_layout(REPO, profile_id)
    tokens = lib._unit_tokens(effective, Path("/"), python_bin="/opt/iraf-sdk/current/bin/python",
                              run_user="root")
    return effective, tokens


def test_declared_role_renders_from_template():
    effective, tokens = _tokens("edge-intel")
    assert effective.get("env_template"), "edge-intel 应当声明 env_template"
    text = lib.render_env_file(effective, Path("/"), tokens, repo_root=REPO,
                               env_values={"PROVIDER_ENDPOINT": "PROBE-HOST:50052"})
    assert "IRAF_BACKEND_ENTRYPOINT=iraf_adapters.remote.provider_backend:RemoteProviderBackend" in text
    assert '"endpoint":"PROBE-HOST:50052"' in text, text
    assert "IRAF_EVENT_STORE=/var/lib/iraf/iraf-runtime.db" in text


def test_missing_deployment_value_fails_closed():
    """缺部署期取值必须显式失败 —— 否则会渲染出一个"看起来能用"的半成品 env。"""
    effective, tokens = _tokens("edge-intel")
    with pytest.raises(lib.InstallError) as exc:
        lib.render_env_file(effective, Path("/"), tokens, repo_root=REPO, env_values={})
    assert "PROVIDER_ENDPOINT" in str(exc.value)


def test_role_without_template_keeps_programmatic_output():
    """未声明模板的角色（single-ubuntu2204）行为不变：仍由声明程序化生成。"""
    effective, tokens = _tokens("single-ubuntu2204")
    assert not effective.get("env_template")
    text = lib.render_env_file(effective, Path("/"), tokens, repo_root=REPO)
    assert "IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend" in text
    assert "IRAF_RUNTIME_PORT=8765" in text


def test_credentials_never_written_into_env_file():
    """凭据只由部署环境注入：渲染结果里不得出现 token 赋值。"""
    effective, tokens = _tokens("edge-intel")
    text = lib.render_env_file(effective, Path("/"), tokens, repo_root=REPO,
                               env_values={"PROVIDER_ENDPOINT": "PROBE-HOST:50052"})
    assert "IRAF_PROVIDER_TOKEN=" not in text
    assert "token_env" in text, "应当只声明 token 的**环境变量名**，而不是它的值"
