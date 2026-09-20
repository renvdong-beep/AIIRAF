"""按行号区间打印文件内容（避免在 shell 里写复杂 sed/awk）。"""

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("start", type=int)
    parser.add_argument("end", type=int)
    args = parser.parse_args()
    lines = Path(args.path).read_text(encoding="utf-8").splitlines()
    for index in range(max(0, args.start - 1), min(len(lines), args.end)):
        print("%5d| %s" % (index + 1, lines[index]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
