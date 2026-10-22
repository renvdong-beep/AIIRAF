#!/usr/bin/env bash
# 只读校验：新 dashboard 进程是否拿到 key + 模型调用通路 + 登录方式摸底
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. dashboard 进程 env（只报长度/头部）==="
run 'PID=$(ss -lntp 2>/dev/null | awk "/:9119 /{print \$NF}" | sed "s/.*pid=\([0-9]*\).*/\1/" | head -1); echo "  pid=$PID cwd=$(readlink -f /proc/$PID/cwd)"; tr "\0" "\n" < /proc/$PID/environ | grep -E "^(CUSTOM_API_KEY|OPENAI_BASE_URL|OPENAI_API_KEY|INTEWELL_AGENT_HOME)=" | while IFS= read -r l; do k=${l%%=*}; v=${l#*=}; case "$k" in *API_KEY) echo "  $k len=${#v} head=$(printf %s "$v" | cut -c1-6)";; *) echo "  $k=$v";; esac; done'

echo "=== 2. 直接打模型（同一 env，deepseek-flash，最短请求）==="
run 'v=$(cat /etc/iraf/agentos.key); curl -s -o /tmp/cc.json -w "  chat/completions -> %{http_code}\n" --max-time 25 -H "Authorization: Bearer $v" -H "Content-Type: application/json" -d "{\"model\":\"deepseek-flash\",\"messages\":[{\"role\":\"user\",\"content\":\"只回一个字：好\"}],\"max_tokens\":8}" http://192.168.100.89:3000/v1/chat/completions; head -c 220 /tmp/cc.json; echo'

echo "=== 3. 登录方式摸底：cookie 登录 admin/123 ==="
run 'rm -f /tmp/ck.txt; curl -s -o /tmp/login_page.html -w "  POST /login -> %{http_code}\n" --max-time 8 -c /tmp/ck.txt -d "username=admin&password=123" http://127.0.0.1:9119/login; echo "  cookie: $(grep -c . /tmp/ck.txt 2>/dev/null) 行"; curl -s -o /dev/null -w "  带 cookie 访问 /api/version -> %{http_code}\n" --max-time 6 -b /tmp/ck.txt http://127.0.0.1:9119/api/version'

echo "=== 4. auth.json 的键名（不打值）==="
run 'python3 -c "import json;d=json.load(open(\"/root/.intewell/auth.json\"));print(\"  keys:\",list(d)) if isinstance(d,dict) else print(type(d))" 2>&1 | head -3'
