#!/usr/bin/env python3
"""Compatibility entry point; canonical CLI is iraf_tools.run_task."""
from iraf_tools.run_task import main

if __name__ == "__main__":
    raise SystemExit(main())
