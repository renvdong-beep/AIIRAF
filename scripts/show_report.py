"""汇总展示验收 report 的关键结论（避免在 shell 里内联 Python）。"""

import argparse
import json
from pathlib import Path


def summarize(path, tail=None):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    execution = data.get("execution") or {}
    print("status :", execution.get("status"))
    print("reason :", execution.get("reason"))
    print("error  :", execution.get("error_code"))
    print("sequence:", execution.get("sequence"))
    if execution.get("status") == "FAILED":
        print("skill  :", execution.get("requested_skill"))
        return 1
    result = execution.get("result") or {}
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if tail:
        text = "\n".join(text.splitlines()[-int(tail):])
    print(text)
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report")
    parser.add_argument("--tail", type=int, default=None)
    args = parser.parse_args()
    return summarize(args.report, args.tail)


if __name__ == "__main__":
    raise SystemExit(main())
