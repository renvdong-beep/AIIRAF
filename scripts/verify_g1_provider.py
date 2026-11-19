#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G1 单机 Provider 验收（无板卡）：stand + locomote（simulation=true）。

用法：PYTHONPATH=src python3 scripts/verify_g1_provider.py [endpoint]
token 从 ~/.config/iraf/runtime.secrets 的 IRAF_PROVIDER_TOKEN 读取。
"""
import re
import sys
from types import SimpleNamespace

from iraf_adapters.remote.provider_backend import RemoteProviderBackend

ENDPOINT = sys.argv[1] if len(sys.argv) > 1 else "10.203.247.145:50058"
raw = open("/home/coretek/.config/iraf/runtime.secrets", encoding="utf-8").read()
match = re.search(r"^IRAF_PROVIDER_TOKEN=(.*)$", raw, re.M)
token = (match.group(1).strip().strip("'\"") if match else "")
if not token:
    raise SystemExit("缺少 IRAF_PROVIDER_TOKEN")

backend = RemoteProviderBackend(ENDPOINT, token, resource_id="unitree_g1", timeout_s=180)
lease = SimpleNamespace(fencing_token=1, resource="unitree_g1")
stand = backend.stand(lease)
print("stand:", {k: stand.get(k) for k in ("passed", "height_m", "max_tilt_deg", "max_tracking_error_rad")})
walk = backend.locomote({"vx_mps": 0.5, "vy_mps": 0.0, "wz_rad_s": 0.0}, 2000, lease)
print("walk:", {k: walk.get(k) for k in ("passed", "distance_m", "command_forward_mps")})
if not stand.get("passed"):
    raise SystemExit(1)
