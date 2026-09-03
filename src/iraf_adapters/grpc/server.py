"""Canonical gRPC Runtime server entry point."""
import argparse
import os
from concurrent import futures
import grpc
from ..bootstrap import build_runtime_from_env
from .runtime_grpc import SkillRuntimeServicer, add_execute_get_servicer_to_server
from .events_grpc import EventServicer, add_event_servicer_to_server
from ..mujoco.supervisor import MujocoSimulationSupervisor

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
    supervisor = MujocoSimulationSupervisor(runtime.backend) if hasattr(runtime.backend, "start_continuous") else None
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
