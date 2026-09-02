#!/usr/bin/env python3
"""启动 IRAF canonical gRPC Runtime；仅装配核心，不实现业务逻辑。"""
import argparse,os
from concurrent import futures
import grpc
from bootstrap import build_runtime_from_env
from runtime_grpc import SkillRuntimeServicer,add_execute_get_servicer_to_server

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--host",default=os.environ.get("IRAF_GRPC_HOST","127.0.0.1"))
    parser.add_argument("--port",type=int,default=int(os.environ.get("IRAF_GRPC_PORT","50051")))
    args=parser.parse_args()
    token=os.environ.get("IRAF_GRPC_TOKEN")
    if not token:
        parser.error("IRAF_GRPC_TOKEN 必须由部署环境提供")
    if os.environ.get("IRAF_GRPC_DEVELOPMENT","false").lower()!="true":
        parser.error("当前 gRPC 适配器仅允许 IRAF_GRPC_DEVELOPMENT=true")
    runtime=build_runtime_from_env()
    if not runtime.profile.simulation:
        parser.error("开发 gRPC 适配器仅允许 simulation=true")
    if args.host not in {"127.0.0.1","localhost"}:
        parser.error("开发 gRPC 适配器只允许回环地址")
    server=grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    add_execute_get_servicer_to_server(
        SkillRuntimeServicer(runtime,token,os.environ.get("IRAF_RUNTIME_SUBJECT","agentos")),
        server,
    )
    server.add_insecure_port("%s:%s"%(args.host,args.port))
    server.start()
    print("IRAF canonical gRPC Runtime (development simulation) listening on %s:%s"%(args.host,args.port),flush=True)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        server.stop(grace=2)

if __name__=="__main__": main()
