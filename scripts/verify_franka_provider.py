#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
from types import SimpleNamespace

from iraf_adapters.remote.provider_backend import RemoteProviderBackend

raw = open("/home/coretek/.config/iraf/runtime.secrets", encoding="utf-8").read()
m = re.search(r"^IRAF_PROVIDER_TOKEN=(.*)$", raw, re.M)
token = m.group(1).strip().strip("'\"")
backend = RemoteProviderBackend("10.203.247.145:50059", token, resource_id="franka", timeout_s=120)
lease = SimpleNamespace(fencing_token=1, resource="franka")
positions = {"joint1": 0.2, "joint2": 0.1, "joint3": 0.0, "joint4": -0.6,
             "joint5": 0.0, "joint6": 0.6, "joint7": 0.0}
out = backend.move_joint(positions, 2000, lease)
print("move_joint accepted", out is None)
