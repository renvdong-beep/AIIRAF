"""AgentOSBridge 将文字意图映射为 canonical IRAF Skill 请求。"""
import copy
import hashlib
import json

from ..model.openai_intent import IntentProviderError


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

    def execute_intent(self, request, context):
        correlation_id = request.get("correlation_id", "")
        metadata = self._intent_metadata(request)
        try:
            if not request.get("request_id") or not request.get("idempotency_key") or not correlation_id:
                raise IntentProviderError("request_id、correlation_id 和 idempotency_key 必填")
            intent = self.provider.parse(request.get("text", ""), self._intent_contracts())
        except IntentProviderError as exc:
            return self.dispatcher.record_pre_dispatch_failure(
                request,
                context,
                "IRAF-INTENT-PARSE-FAILED",
                str(exc),
                metadata,
            )
        profile = self.dispatcher.profile
        safety = self.dispatcher.safety_policy
        task = {"request_id": request["request_id"], "correlation_id": correlation_id, "idempotency_key": request["idempotency_key"], "resource_id": request.get("resource_id", self.dispatcher.resource_id), "controller": request.get("controller", "agentos-intent"), "skill": intent["skill"], "skill_version_constraint": request.get("skill_version_constraint", ""), "parameters": intent["parameters"], "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest, "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest, "deadline_unix_ms": request.get("deadline_unix_ms", 0)}
        result = self.dispatcher.dispatch(task, context)
        metadata["resolved_skill"] = intent["skill"]
        return self.dispatcher.annotate(result, metadata)

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
