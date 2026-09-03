"""Submit a canonical Skill request through the runtime HTTP adapter."""
import argparse,json,os,time,uuid
from datetime import datetime,timezone
from urllib.request import Request,urlopen
from iraf_core.profile import load_robot_profile,load_safety_policy

def main(argv=None):
    parser=argparse.ArgumentParser(prog="iraf-task")
    parser.add_argument("--endpoint", default=os.environ.get("IRAF_RUNTIME_URL"))
    parser.add_argument("--token", default=os.environ.get("IRAF_RUNTIME_TOKEN"))
    parser.add_argument("--profile", required=True)
    parser.add_argument("--safety-policy", required=True)
    parser.add_argument("--skill", required=True)
    parser.add_argument("--skill-version", default="")
    parser.add_argument("--parameters", default="{}")
    parser.add_argument("--resource-id")
    parser.add_argument("--controller", default="iraf-cli")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    args=parser.parse_args(argv)
    if not args.endpoint or not args.token: parser.error("endpoint and token are required")
    if args.timeout_ms <= 0: parser.error("timeout-ms must be positive")
    profile=load_robot_profile(args.profile); safety=load_safety_policy(args.safety_policy); request_id=str(uuid.uuid4())
    deadline=datetime.fromtimestamp((time.time()*1000+args.timeout_ms)/1000,timezone.utc).isoformat().replace("+00:00","Z")
    body={"requestId":request_id,"idempotencyKey":request_id,"goal":{"correlationId":request_id,"skillName":args.skill,"skillVersionConstraint":args.skill_version,"inputs":json.loads(args.parameters),"deadline":deadline,"robotProfile":{"name":profile.name,"version":profile.version,"digest":profile.digest},"safetyPolicy":{"name":safety.name,"version":safety.version,"digest":safety.digest},"labels":{"resource_id":args.resource_id or profile.name,"controller":args.controller}}}
    request=Request(args.endpoint.rstrip("/")+"/v1/tasks",data=json.dumps(body,ensure_ascii=False).encode(),headers={"Content-Type":"application/json","Authorization":"Bearer "+args.token},method="POST")
    with urlopen(request,timeout=max(1,args.timeout_ms/1000+2)) as response: result=json.loads(response.read().decode())
    print(json.dumps(result,ensure_ascii=False)); return 0 if result.get("status")=="SUCCEEDED" else 1

if __name__ == "__main__": raise SystemExit(main())
