# MCP 配置示例（给上位机 harness 直接使用）

三份文件对应三种宿主形态：

1. `mcp-http.yaml`
   跨机上位机首选。宿主配置支持 `url + headers` 时用这份。
   必须把 `EDGE_HOST`、`TOKEN` 换成实际值；URL 必须是边缘的 MCP 端点
   （默认 `https://<edge>:8765/mcp`）。

2. `mcp-http.claude.json`
   Claude Desktop / 通用 `mcpServers` JSON 形态。字段含义与上面相同。

3. `mcp-stdio.yaml`
   只在**上位机同机已安装 IRAF runtime** 时使用；跨机不可用。
   stdio 不需要 token 注册表，身份由 `IRAF_RUNTIME_SUBJECT` 给出。

## 鉴权要点

- HTTP 形态必须带 `Authorization: Bearer <token>`；token 由边缘侧
  `python3 -m iraf_tools.mcp_tokens issue ...` 签发。
- 只支持 `url`、不支持自定义 header 的宿主**不能直连**需要鉴权的边缘 MCP；
  两条路：改用 stdio，或在本机放一个注入 Authorization 的反向代理（不要关鉴权）。
- 不要使用 `IRAF_MCP_INSECURE=1` 交付；它只允许 loopback 调试。

## 自检

```bash
# 无 token 应 401
curl -s -o /dev/null -w '%{http_code}\n' -X POST https://EDGE_HOST:8765/mcp

# 有效 token 用 SDK 自检
export MCP_URL=https://EDGE_HOST:8765/mcp
export MCP_TOKEN=...
python3 ../examples/mcp_client_min.py
```
