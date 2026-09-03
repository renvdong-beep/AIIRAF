import threading
import unittest
from concurrent import futures
from datetime import datetime, timezone, timedelta
from pathlib import Path

import grpc
from iraf.v1 import events_pb2, events_pb2_grpc, skill_pb2, runtime_pb2_grpc
from iraf_adapters.grpc.events_grpc import EventServicer, add_event_servicer_to_server
from iraf_adapters.grpc.runtime_grpc import (
    SkillRuntimeServicer,
    add_execute_get_servicer_to_server,
)
from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore

class Backend:
    def __init__(self, authority):
        self.authority = authority
        self.calls = []
    def runtime_inventory(self): return {"safety": {"estop": False}, "mode": "simulation"}
    def move_joint(self, positions, duration_ms, lease): self.authority.validate(lease); self.calls.append((positions, duration_ms))
    def stop(self, lease): self.authority.validate(lease); self.calls.append(("stop",))

class BlockingBackend(Backend):
    def __init__(self, authority):
        super().__init__(authority)
        self.started = threading.Event()
        self.release = threading.Event()
        self.stopped = threading.Event()
    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        self.calls.append((positions, duration_ms))
        self.started.set()
        if not self.release.wait(timeout=3):
            raise TimeoutError("test backend was not released")
    def stop(self, lease):
        self.authority.validate(lease)
        self.calls.append(("stop",))
        self.stopped.set()
        self.release.set()

class GrpcRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.authority = ControlAuthorityManager()
        cls.backend = Backend(cls.authority)
        cls.profile = RobotProfile("robot", "1.0.0", True, 100, ("j1",), {"j1": (-1, 1)}, frozenset({"move_joint", "stop"}), "development", "profile-digest")
        cls.safety = SafetyPolicy("lab", "1.0.0", "development", True, frozenset({"move_joint", "stop"}), 30000, "safety-digest")
        cls.registry = SkillRegistry().load_directory(Path(__file__).resolve().parents[2] / "skills")
        cls.store = SqliteExecutionStore(":memory:")
        cls.runtime = SkillRuntime(cls.profile, cls.safety, cls.backend, cls.registry, cls.authority, cls.store)
        cls.server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
        add_execute_get_servicer_to_server(SkillRuntimeServicer(cls.runtime, "token", "agentos"), cls.server)
        add_event_servicer_to_server(EventServicer(cls.store, "token", "agentos"), cls.server)
        cls.port = cls.server.add_insecure_port("127.0.0.1:0")
        cls.server.start()
        cls.channel = grpc.insecure_channel("127.0.0.1:%d" % cls.port)
        cls.stub = runtime_pb2_grpc.SkillRuntimeServiceStub(cls.channel)
        cls.event_stub = events_pb2_grpc.EventServiceStub(cls.channel)

    @classmethod
    def tearDownClass(cls):
        cls.channel.close()
        cls.server.stop(0)

    @classmethod
    def request(cls, key="grpc-key-1", duration_ms=10):
        req = skill_pb2.ExecuteSkillRequest(request_id="request-" + key, idempotency_key=key)
        req.goal.correlation_id = "correlation-" + key
        req.goal.skill_name = "move_joint"
        req.goal.skill_version_constraint = "1.0.0"
        req.goal.inputs.update({"positions": {"j1": 0.2}, "duration_ms": duration_ms})
        req.goal.robot_profile.name = "robot"
        req.goal.robot_profile.version = "1.0.0"
        req.goal.robot_profile.digest = "profile-digest"
        req.goal.safety_policy.name = "lab"
        req.goal.safety_policy.version = "1.0.0"
        req.goal.safety_policy.digest = "safety-digest"
        req.goal.labels["resource_id"] = "robot"
        req.goal.labels["controller"] = "grpc-test"
        req.goal.deadline.FromDatetime(datetime.now(timezone.utc) + timedelta(seconds=5))
        return req

    def test_execute_and_get(self):
        messages = list(self.stub.Execute(self.request(), metadata=(("authorization", "Bearer token"),)))
        terminal = messages[-1]
        self.assertEqual(skill_pb2.SKILL_STATE_SUCCEEDED, terminal.state)
        snapshot = self.stub.GetExecution(skill_pb2.GetExecutionRequest(execution_id=terminal.execution_id), metadata=(("authorization", "Bearer token"),))
        self.assertEqual(skill_pb2.SKILL_STATE_SUCCEEDED, snapshot.state)
        self.assertEqual(3, snapshot.last_sequence)
        page = self.event_stub.ListEvents(events_pb2.ListEventsRequest(execution_id=terminal.execution_id, limit=2), metadata=(("authorization", "Bearer token"),))
        self.assertEqual([0, 1], [event.sequence for event in page.events])
        self.assertTrue(page.next_page_token)
        tail = self.event_stub.ListEvents(events_pb2.ListEventsRequest(execution_id=terminal.execution_id, limit=2, page_token=page.next_page_token), metadata=(("authorization", "Bearer token"),))
        self.assertEqual([2, 3], [event.sequence for event in tail.events])
        self.assertFalse(tail.next_page_token)
        replay = self.event_stub.GetReplayManifest(events_pb2.GetReplayManifestRequest(execution_id=terminal.execution_id), metadata=(("authorization", "Bearer token"),))
        self.assertEqual("iraf.execution-replay/v1", replay.schema_version)
        self.assertEqual("SUCCEEDED", replay.terminal_status)
        self.assertEqual("move_joint", replay.skill.name)
        self.assertEqual("profile-digest", replay.robot_profile.digest)
        self.assertEqual([0, 1, 2, 3], [event.sequence for event in replay.events])
        self.assertEqual(64, len(replay.subject_digest))
        self.assertEqual(64, len(replay.result_digest))
        self.assertTrue(replay.simulation)

    def test_event_service_lists_recovered_safety_history(self):
        event = self.runtime.handle_safety_event("event-resource", "heartbeat timeout", "HEARTBEAT_TIMEOUT")
        self.runtime.recover_safety("event-resource", event.event_id, True, "operator")
        response = self.event_stub.ListEvents(events_pb2.ListEventsRequest(object_id="event-resource"), metadata=(("authorization", "Bearer token"),))
        self.assertEqual(1, len(response.events))
        self.assertEqual("HEARTBEAT_TIMEOUT", response.events[0].kind)
        self.assertEqual("operator", response.events[0].recovered_by)
        self.assertTrue(response.events[0].recovered_at.seconds > 0)

    def test_event_service_authentication_required(self):
        with self.assertRaises(grpc.RpcError) as cm:
            self.event_stub.ListEvents(events_pb2.ListEventsRequest(limit=1))
        self.assertEqual(grpc.StatusCode.UNAUTHENTICATED, cm.exception.code())
        with self.assertRaises(grpc.RpcError) as cm:
            self.event_stub.GetReplayManifest(events_pb2.GetReplayManifestRequest(execution_id="none"))
        self.assertEqual(grpc.StatusCode.UNAUTHENTICATED, cm.exception.code())

    def test_replay_manifest_missing_execution_is_not_found(self):
        with self.assertRaises(grpc.RpcError) as cm:
            self.event_stub.GetReplayManifest(events_pb2.GetReplayManifestRequest(execution_id="none"), metadata=(("authorization", "Bearer token"),))
        self.assertEqual(grpc.StatusCode.NOT_FOUND, cm.exception.code())

    def test_authentication_required(self):
        with self.assertRaises(grpc.RpcError) as cm:
            list(self.stub.Execute(self.request(key="unauthorized")))
        self.assertEqual(grpc.StatusCode.UNAUTHENTICATED, cm.exception.code())

    def test_cancel_unknown_execution_is_rejected(self):
        response = self.stub.Cancel(skill_pb2.CancelExecutionRequest(execution_id="none"), metadata=(("authorization", "Bearer token"),))
        self.assertFalse(response.accepted)
        self.assertEqual(skill_pb2.SKILL_STATE_UNSPECIFIED, response.state)

    def test_active_execution_cancels_and_stops_backend(self):
        authority = ControlAuthorityManager()
        backend = BlockingBackend(authority)
        runtime = SkillRuntime(self.profile, self.safety, backend, self.registry, authority, SqliteExecutionStore(":memory:"))
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
        add_execute_get_servicer_to_server(SkillRuntimeServicer(runtime, "token", "agentos"), server)
        port = server.add_insecure_port("127.0.0.1:0")
        server.start()
        channel = grpc.insecure_channel("127.0.0.1:%d" % port)
        stub = runtime_pb2_grpc.SkillRuntimeServiceStub(channel)
        try:
            stream = stub.Execute(self.request(key="cancel-active", duration_ms=1000), metadata=(("authorization", "Bearer token"),))
            accepted = next(stream)
            self.assertIn(accepted.state, {skill_pb2.SKILL_STATE_PENDING, skill_pb2.SKILL_STATE_VALIDATING, skill_pb2.SKILL_STATE_RUNNING})
            self.assertTrue(backend.started.wait(timeout=1))
            cancelled = stub.Cancel(skill_pb2.CancelExecutionRequest(execution_id=accepted.execution_id, reason="operator requested"), metadata=(("authorization", "Bearer token"),))
            self.assertTrue(cancelled.accepted)
            terminal = list(stream)[-1]
            self.assertEqual(skill_pb2.SKILL_STATE_CANCELLED, terminal.state)
            self.assertEqual("IRAF-CANCELLED", terminal.error.code)
            self.assertTrue(backend.stopped.is_set())
            snapshot = stub.GetExecution(skill_pb2.GetExecutionRequest(execution_id=accepted.execution_id), metadata=(("authorization", "Bearer token"),))
            self.assertEqual(skill_pb2.SKILL_STATE_CANCELLED, snapshot.state)
        finally:
            channel.close()
            server.stop(0)

if __name__ == "__main__":
    unittest.main()
