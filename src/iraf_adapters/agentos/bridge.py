"""AgentOSBridge 将文字意图映射为 canonical IRAF Skill 请求。"""
import copy
import hashlib
import json
import threading

from ..model.openai_intent import IntentNoMatch, IntentProviderError


class AgentOSBridge:
    def __init__(self, provider, dispatcher):
        self.provider = provider
        self.dispatcher = dispatcher

    def _intent_contracts(self):
        """Bind generic Skill schemas to the active RobotProfile capabilities."""
        contracts = copy.deepcopy(self.dispatcher.registry.intent_contracts())
        joints = tuple(self.dispatcher.profile.joints)
        if not joints:
            return contracts
        for contract in contracts.values():
            schema = contract.get("inputSchema") or {}
            positions = (schema.get("properties") or {}).get("positions")
            if not isinstance(positions, dict):
                continue
            # Generic Skill schemas cannot name a robot's joints. The effective
            # AgentOS contract must expose only joints accepted by this profile.
            positions["properties"] = {joint: {"type": "number"} for joint in joints}
            positions["additionalProperties"] = False
            contract["robotJoints"] = list(joints)
        return contracts

    def _parse_and_build(self, request, context):
        """解析意图 + 组装任务（execute_intent 与 submit_intent 共用，避免复制粘贴）。
        返回 (task, metadata, intent)；解析失败时返回 (failure_result, None, None)。"""
        correlation_id = request.get("correlation_id", "")
        metadata = self._intent_metadata(request)
        try:
            if not request.get("request_id") or not request.get("idempotency_key") or not correlation_id:
                raise IntentProviderError("request_id、correlation_id 和 idempotency_key 必填")
            intent = self.provider.parse(request.get("text", ""), self._intent_contracts())
        except IntentNoMatch as exc:
            # 过滤（不是故障）：这句话与任何机器人技能都不匹配。给调用方一个**可判读**的码，
            # 不要混进"解析失败"——机器人开发模式下平台会把整段对话都转发过来。
            skills = ", ".join(exc.matches)
            return self.dispatcher.record_pre_dispatch_failure(
                request,
                context,
                "IRAF-NOT-ROBOT-TASK",
                "%s（可用技能：%s）" % (exc.reason, skills),
                metadata,
            ), None, None
        except IntentProviderError as exc:
            return self.dispatcher.record_pre_dispatch_failure(
                request,
                context,
                "IRAF-INTENT-PARSE-FAILED",
                str(exc),
                metadata,
            ), None, None
        profile = self.dispatcher.profile
        safety = self.dispatcher.safety_policy
        task = {"request_id": request["request_id"], "correlation_id": correlation_id, "idempotency_key": request["idempotency_key"], "resource_id": request.get("resource_id", self.dispatcher.resource_id), "controller": request.get("controller", "agentos-intent"), "skill": intent["skill"], "skill_version_constraint": request.get("skill_version_constraint", ""), "parameters": intent["parameters"], "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest, "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest, "deadline_unix_ms": request.get("deadline_unix_ms", 0)}
        return task, metadata, intent

    def execute_intent(self, request, context):
        """同步执行：CLI/本地整链用（阻塞到执行完成，返回最终结论）。"""
        task, metadata, intent = self._parse_and_build(request, context)
        if intent is None:
            return task  # 预调度失败结果
        result = self.dispatcher.dispatch(task, context)
        metadata["resolved_skill"] = intent["skill"]
        return self.dispatcher.annotate(result, metadata)

    def submit_intent(self, request, context):
        """异步提交：MCP 北向用。立即返回 execution_id + VALIDATING，执行在后台线程进行，
        客户端用 iraf_get_task(execution_id) 轮询最终结论（不阻塞物理执行，避免 SSE 空闲断连）。"""
        task, metadata, intent = self._parse_and_build(request, context)
        if intent is None:
            return task  # 预调度失败结果（同步返回，无需轮询）
        execution_id = self.dispatcher.runtime.execution_id_for(
            context.subject, request["idempotency_key"]
        )
        metadata["resolved_skill"] = intent["skill"]
        ready_event = threading.Event()

        def _run():
            try:
                result = self.dispatcher.dispatch(
                    task, context, execution_id=execution_id, ready_event=ready_event
                )
                self.dispatcher.annotate(result, metadata)
            except Exception:
                # 后台执行异常不拖垮 MCP 服务进程；结论由事件库承载，轮询方按 FAILED 处理。
                pass

        threading.Thread(target=_run, daemon=True).start()
        ready_event.wait(timeout=15)
        return {
            "execution_id": execution_id,
            "status": "VALIDATING",
            "resolved_skill": intent["skill"],
            "submitted": True,
        }

    def _intent_metadata(self, request):
        encoded = json.dumps(
            request,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode()
        return {
            "adapter": "agentos-intent",
            "intent_provider": {
                "name": getattr(self.provider, "name", "unverified"),
                "version": getattr(self.provider, "version", "unverified"),
                "model": getattr(self.provider, "model_id", "unverified"),
            },
            "intent_request_digest": hashlib.sha256(encoded).hexdigest(),
        }
