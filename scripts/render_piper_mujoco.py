import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

# Load the Ubuntu MuJoCo binding first so it owns the board EGL driver.
import mujoco
from PIL import Image

# The framework's pure-Python packages can come from the project venv without
# replacing the already loaded system MuJoCo binding.
try:
    import jsonschema  # noqa: F401
except ImportError:
    candidates = [Path("/home/coretek/miniconda3/envs/mujoco_graspnet/lib/python3.9/site-packages")]
    candidates.extend(sorted(Path("/home/coretek/miniconda3/envs").glob("*/lib/python*/site-packages")))
    for candidate in candidates:
        if not candidate.is_dir() or str(candidate) in sys.path:
            continue
        sys.path.insert(0, str(candidate))
        try:
            import jsonschema  # noqa: F401
            break
        except ImportError:
            continue

from iraf_adapters.bootstrap import build_runtime_from_env
from iraf_core.policy import AuthenticatedContext
from iraf_adapters.mujoco.supervisor import MujocoSimulationSupervisor


def _load_runtime_environment():
    required = ("IRAF_PROFILE", "IRAF_SAFETY_POLICY", "IRAF_SKILL_ROOT", "IRAF_BACKEND_ENTRYPOINT", "IRAF_BACKEND_CONFIG", "IRAF_EVENT_STORE")
    if all(os.environ.get(key) for key in required):
        return
    try:
        pids = subprocess.check_output(["pgrep", "-f", "iraf_adapters.http.runtime_http"], text=True).split()
    except (OSError, subprocess.CalledProcessError):
        pids = []
    for pid in reversed(pids):
        try:
            values = {}
            for item in Path("/proc", pid, "environ").read_bytes().split(b"\0"):
                if b"=" in item:
                    key, value = item.split(b"=", 1)
                    values[key.decode()] = value.decode()
            if all(values.get(key) for key in required):
                os.environ.update({key: values[key] for key in required})
                return
        except (OSError, UnicodeDecodeError):
            continue
    missing = [key for key in required if not os.environ.get(key)]
    raise RuntimeError("Runtime environment is missing: " + ", ".join(missing))


def _timestamp_ms():
    return int(time.time() * 1000)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/piper-mujoco-view"))
    parser.add_argument("--duration-ms", type=int, default=300)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--forever", action="store_true", help="keep the framework-owned simulation and live snapshot running")
    parser.add_argument("--render-interval", type=float, default=1.0)
    args = parser.parse_args(argv)
    if args.duration_ms < 1 or args.fps < 1 or args.render_interval <= 0:
        parser.error("duration-ms, fps, and render-interval must be positive")
    _load_runtime_environment()
    runtime = build_runtime_from_env()
    profile = runtime.profile
    safety = runtime.safety_policy
    backend = runtime.backend
    if not hasattr(backend, "render_frame") or not hasattr(backend, "model"):
        raise RuntimeError("configured backend does not provide framework rendering")
    args.output.mkdir(parents=True, exist_ok=True)
    supervisor = MujocoSimulationSupervisor(backend)
    supervisor.start()
    renderer = mujoco.Renderer(backend.model, height=480, width=640)
    initial_path = args.output / "piper-initial.png"
    final_path = args.output / "piper-final.png"
    gif_path = args.output / "piper-motion.gif"
    live_path = args.output / "piper-live.png"
    try:
        initial = Image.fromarray(backend.render_frame(renderer)).copy()
        request = {"request_id": "visualizer-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S"), "idempotency_key": "visualizer-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S%f"), "correlation_id": "visualizer-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d"), "skill": "move_joint", "skill_version_constraint": "1.0.0", "parameters": {"positions": {"joint1": 0.2}, "duration_ms": args.duration_ms}, "deadline_unix_ms": _timestamp_ms() + 30_000, "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest, "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest, "resource_id": runtime.resource_id, "controller": "visualizer"}
        context = AuthenticatedContext("visualizer", frozenset({"task.submit", "task.read"}), "local")
        result = runtime.execute(request, context)
        final = Image.fromarray(backend.render_frame(renderer)).copy()
        initial.save(initial_path)
        final.save(final_path)
        initial.save(gif_path, save_all=True, append_images=[final], duration=max(1, int(1000 / args.fps)), loop=0)
        report = {"schema_version": "iraf.piper-mujoco-view/v3", "framework_execution": result, "skill": "move_joint", "provider": result.get("provider", {}), "simulation_only": bool(profile.simulation), "continuous_supervisor": supervisor.running, "initial_png": str(initial_path), "final_png": str(final_path), "animation_gif": str(gif_path), "live_png": str(live_path)}
        report_path = args.output / "render-report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
        if args.forever:
            print(json.dumps({"status": "RUNNING", "live_png": str(live_path), "report": str(report_path)}, ensure_ascii=True), flush=True)
            while True:
                Image.fromarray(backend.render_frame(renderer)).save(live_path)
                time.sleep(args.render_interval)
        print(json.dumps(report, ensure_ascii=True, indent=2))
        return 0 if result.get("status") == "SUCCEEDED" else 1
    except KeyboardInterrupt:
        return 0
    finally:
        renderer.close()
        supervisor.stop()


if __name__ == "__main__":
    raise SystemExit(main())
