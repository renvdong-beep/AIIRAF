"""AgentOS northbound HTTP boundary."""
import json
import os
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

class AgentOSBridgeError(RuntimeError):
    pass

class AgentOSBridgeClient:
    def __init__(self, base_url=None, token=None, timeout=10):
        self.base_url = (base_url or os.getenv("IRAF_AGENTOS_URL", "http://127.0.0.1:8090")).rstrip("/")
        self.token = token or os.getenv("IRAF_AGENTOS_TOKEN", "")
        self.timeout = timeout

    def _request(self, method, path, payload=None):
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        request = Request(self.base_url + path, data=body, headers=headers, method=method)
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8")
                return json.loads(raw) if raw else {}
        except (HTTPError, URLError, TimeoutError) as exc:
            raise AgentOSBridgeError(f"AgentOS request failed: {exc}") from exc

    def submit_task(self, task):
        return self._request("POST", "/v1/tasks", task)

    def get_task(self, task_id):
        return self._request("GET", f"/v1/tasks/{task_id}")

    def cancel_task(self, task_id):
        return self._request("POST", f"/v1/tasks/{task_id}/cancel")
