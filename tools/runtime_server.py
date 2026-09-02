#!/usr/bin/env python3
"""兼容入口；实现位于 adapters/http，生产接口应使用 gRPC。"""
from runtime_http import main
if __name__=="__main__": main()