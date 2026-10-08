"""Canonical gRPC Runtime server entry point."""
import argparse
import os
from concurrent import futures
import grpc
from ..bootstrap import build_runtime_from_env
from .runtime_grpc import SkillRuntimeServicer, add_execute_get_servicer_to_server
from .events_grpc import EventServicer, add_event_servicer_to_server


def _supervisor_for(backend):
    """只为**真正需要连续步进**的 backend（MuJoCo）装配仿真监督器。

    为什么必须**惰性导入**（2026-10-08 双域容器实测，控制域）：
    本模块原先在 import 期就 `from ..mujoco.supervisor import MujocoSimulationSupervisor`，
    而 `iraf_adapters.mujoco.__init__` 又会 import `mujoco`（仿真依赖）。控制域按部署设计
    **只跑 Capability Provider、不装仿真依赖**（仿真在智能域/x86 侧，见
    docs/deploy-topology-and-domain-split.md），于是规范入口 `python3 -m iraf_adapters.grpc.server`
    在控制域直接 `ModuleNotFoundError: No module named 'mujoco'` —— 服务端起不来。
    改为按能力按需导入后，控制域无需任何仿真依赖即可起服务的规范入口；
    MuJoCo 路径的行为完全不变（enabled/挂载/停机顺序逐字保留）。
    """
    if not hasattr(backend, "start_continuous"):
        return None
    from ..mujoco.supervisor import MujocoSimulationSupervisor

    return MujocoSimulationSupervisor(backend)

def main(argv=None):
    parser = argparse.ArgumentParser(prog="iraf-runtime-grpc")
    parser.add_argument("--host", default=os.environ.get("IRAF_GRPC_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("IRAF_GRPC_PORT", "50051")))
    args = parser.parse_args(argv)
    token = os.environ.get("IRAF_GRPC_TOKEN")
    if not token:
        parser.error("IRAF_GRPC_TOKEN must be provided by deployment")
    if os.environ.get("IRAF_GRPC_DEVELOPMENT", "false").lower() != "true":
        parser.error("gRPC adapter requires IRAF_GRPC_DEVELOPMENT=true")
    if args.host not in {"127.0.0.1", "localhost"}:
        parser.error("development gRPC adapter only permits loopback")
    runtime = build_runtime_from_env()
    if not runtime.profile.simulation:
        parser.error("development gRPC adapter only permits simulation=true")
    supervisor = _supervisor_for(runtime.backend)
    if supervisor is not None: supervisor.start()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    subject = os.environ.get("IRAF_RUNTIME_SUBJECT", "agentos")
    add_execute_get_servicer_to_server(SkillRuntimeServicer(runtime, token, subject), server)
    add_event_servicer_to_server(EventServicer(runtime.store, token, subject), server)
    server.add_insecure_port(f"{args.host}:{args.port}")
    server.start()
    print(f"IRAF canonical gRPC Runtime (development simulation) listening on {args.host}:{args.port}", flush=True)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        server.stop(grace=2)
    finally:
        if supervisor is not None: supervisor.stop()

if __name__ == "__main__":
    main()
