"""Development EventService adapter backed by the canonical execution store."""
import base64

import grpc
from iraf.v1 import events_pb2, events_pb2_grpc
from .runtime_grpc import _context_from_grpc


def _decode_offset(token):
    if not token:
        return 0
    try:
        value = base64.urlsafe_b64decode(token.encode("ascii")).decode("ascii")
        offset = int(value)
    except (ValueError, UnicodeError, base64.binascii.Error):
        raise ValueError("invalid page token")
    if offset < 0:
        raise ValueError("invalid page token")
    return offset


def _encode_offset(offset):
    return base64.urlsafe_b64encode(str(offset).encode("ascii")).decode("ascii")


def _timestamp(message, millis):
    if not millis:
        return
    message.seconds = int(millis // 1000)
    message.nanos = int(millis % 1000) * 1_000_000


def _event_message(target, row):
    target.event_id = row["event_id"]
    target.object_id = row["object_id"]
    target.execution_id = row["execution_id"]
    target.sequence = row["sequence"]
    target.kind = row["kind"]
    target.status = row["status"]
    target.reason = row["reason"]
    target.recovered_by = row.get("recovered_by", "")
    _timestamp(target.occurred_at, row["occurred_at_ms"])
    _timestamp(target.recovered_at, row.get("recovered_at_ms", 0))


def _artifact_message(target, value):
    value = value or {}
    target.name = value.get("name", "")
    target.version = value.get("version", "")
    target.digest = value.get("digest", "")


class EventServicer(events_pb2_grpc.EventServiceServicer):
    """Read-only event query for development; no raw payloads or credentials."""
    def __init__(self, store, token, subject="iraf-grpc"):
        self.store = store
        self.token = token
        self.subject = subject

    def ListEvents(self, request, context):
        _context_from_grpc(context, self.token, self.subject)
        if request.limit > 1000:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "limit must be <= 1000")
        try:
            offset = _decode_offset(request.page_token)
        except ValueError as exc:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
        rows, next_offset = self.store.list_events(
            execution_id=request.execution_id,
            object_id=request.object_id,
            since_unix_ms=request.since_unix_ms,
            until_unix_ms=request.until_unix_ms,
            limit=request.limit or 100,
            offset=offset,
        )
        response = events_pb2.ListEventsResponse()
        for row in rows:
            _event_message(response.events.add(), row)
        if next_offset is not None:
            response.next_page_token = _encode_offset(next_offset)
        return response

    def GetReplayManifest(self, request, context):
        _context_from_grpc(context, self.token, self.subject)
        if not request.execution_id:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "execution_id is required")
        value = self.store.get_replay_manifest(request.execution_id)
        if value is None:
            context.abort(grpc.StatusCode.NOT_FOUND, "execution replay not found")
        requested_skill = value.get("requested_skill") or {}
        provider = value.get("provider") or {}
        response = events_pb2.ReplayManifest(
            schema_version=value["schema_version"],
            execution_id=value["execution_id"],
            request_digest=value["request_digest"],
            result_digest=value["result_digest"],
            subject_digest=value["subject_digest"],
            idempotency_key_digest=value["idempotency_key_digest"],
            correlation_id=value["correlation_id"],
            terminal_status=value["terminal_status"],
            terminal_sequence=value["terminal_sequence"],
            error_code=value["error_code"],
            reason=value["reason"],
            requested_skill_name=requested_skill.get("name", ""),
            requested_skill_version_constraint=requested_skill.get(
                "version_constraint", ""
            ),
            policy_decision_id=value["policy_decision_id"],
            policy_version=value["policy_version"],
            resource_id=value["resource_id"],
            controller=value["controller"],
            simulation=value["simulation"],
            event_digest=value["event_digest"],
        )
        _artifact_message(response.skill, value.get("skill", {}))
        _artifact_message(response.robot_profile, value.get("profile", {}))
        _artifact_message(
            response.safety_policy, value.get("safety_policy", {})
        )
        response.provider.name = provider.get("name", "")
        response.provider.type = provider.get("type", "")
        for row in value["events"]:
            _event_message(response.events.add(), row)
        _timestamp(response.completed_at, value["completed_at_ms"])
        return response


def add_event_servicer_to_server(servicer, server):
    handlers = {
        "ListEvents": grpc.unary_unary_rpc_method_handler(
            servicer.ListEvents,
            request_deserializer=events_pb2.ListEventsRequest.FromString,
            response_serializer=events_pb2.ListEventsResponse.SerializeToString,
        ),
        "GetReplayManifest": grpc.unary_unary_rpc_method_handler(
            servicer.GetReplayManifest,
            request_deserializer=events_pb2.GetReplayManifestRequest.FromString,
            response_serializer=events_pb2.ReplayManifest.SerializeToString,
        ),
    }
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("iraf.v1.EventService", handlers),))
