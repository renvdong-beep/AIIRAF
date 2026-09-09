"""可替换的 SQLite execution/event store。"""
import hashlib
import json
import sqlite3
import threading
import time


class SqliteExecutionStore:
    def __init__(self, path):
        self._db = sqlite3.connect(path, check_same_thread=False, timeout=30)
        self._lock = threading.RLock()
        self._db.execute("CREATE TABLE IF NOT EXISTS executions(execution_id TEXT PRIMARY KEY,subject TEXT NOT NULL,idempotency_key TEXT NOT NULL,request_digest TEXT NOT NULL,result_json TEXT NOT NULL,updated_at_ms INTEGER NOT NULL,UNIQUE(subject,idempotency_key))")
        self._db.execute("CREATE TABLE IF NOT EXISTS execution_events(execution_id TEXT NOT NULL,sequence INTEGER NOT NULL,status TEXT NOT NULL,reason TEXT NOT NULL,occurred_at_ms INTEGER NOT NULL,PRIMARY KEY(execution_id,sequence))")
        self._db.execute("CREATE TABLE IF NOT EXISTS safety_events(resource TEXT PRIMARY KEY,event_id TEXT UNIQUE NOT NULL,kind TEXT NOT NULL,reason TEXT NOT NULL,occurred_at_ns INTEGER NOT NULL)")
        self._db.execute("CREATE TABLE IF NOT EXISTS safety_event_history(event_id TEXT PRIMARY KEY,resource TEXT NOT NULL,kind TEXT NOT NULL,reason TEXT NOT NULL,occurred_at_ns INTEGER NOT NULL,recovered_at_ns INTEGER,recovered_by TEXT NOT NULL DEFAULT '')")
        self._db.execute("INSERT OR IGNORE INTO safety_event_history(event_id,resource,kind,reason,occurred_at_ns) SELECT event_id,resource,kind,reason,occurred_at_ns FROM safety_events")
        self._db.commit()

    def find_idempotent(self, subject, key):
        with self._lock:
            row = self._db.execute("SELECT request_digest,result_json FROM executions WHERE subject=? AND idempotency_key=?", (subject, key)).fetchone()
            return (row[0], json.loads(row[1])) if row else None

    def save_result(self, subject, key, digest, result):
        with self._lock:
            self._db.execute("INSERT INTO executions VALUES(?,?,?,?,?,?)", (result["execution_id"], subject, key, digest, json.dumps(result, ensure_ascii=True, sort_keys=True), int(time.time() * 1000)))
            self._db.commit()

    def annotate_result(self, execution_id, metadata):
        """只追加新的适配器元数据，不允许改写已有执行事实。"""
        with self._lock:
            row = self._db.execute(
                "SELECT result_json FROM executions WHERE execution_id=?",
                (execution_id,),
            ).fetchone()
            if row is None:
                return None
            result = json.loads(row[0])
            conflicts = sorted(
                key
                for key in set(result).intersection(metadata)
                if result[key] != metadata[key]
            )
            if conflicts:
                raise ValueError("执行元数据不得覆盖已有字段: " + str(conflicts))
            additions = {key: value for key, value in metadata.items() if key not in result}
            if not additions:
                return result
            result.update(additions)
            encoded = json.dumps(result, ensure_ascii=True, sort_keys=True)
            self._db.execute(
                "UPDATE executions SET result_json=?,updated_at_ms=? "
                "WHERE execution_id=?",
                (encoded, int(time.time() * 1000), execution_id),
            )
            self._db.commit()
            return result

    def append_event(self, execution_id, sequence, status, reason=""):
        with self._lock:
            self._db.execute("INSERT OR IGNORE INTO execution_events VALUES(?,?,?,?,?)", (execution_id, sequence, status, reason, int(time.time() * 1000)))
            self._db.commit()

    def get(self, execution_id):
        with self._lock:
            row = self._db.execute("SELECT result_json FROM executions WHERE execution_id=?", (execution_id,)).fetchone()
            return json.loads(row[0]) if row else None

    def get_replay_manifest(self, execution_id):
        """构建不包含原始请求、身份和密钥的确定性回放索引。"""
        with self._lock:
            row = self._db.execute(
                "SELECT subject,idempotency_key,request_digest,result_json,updated_at_ms "
                "FROM executions WHERE execution_id=?",
                (execution_id,),
            ).fetchone()
        if row is None:
            return None
        subject, idempotency_key, request_digest, result_json, completed_at_ms = row
        result = json.loads(result_json)
        events = []
        offset = 0
        while True:
            page, next_offset = self.list_events(
                execution_id=execution_id,
                limit=1000,
                offset=offset,
            )
            events.extend(page)
            if next_offset is None:
                break
            offset = next_offset
        event_payload = json.dumps(
            events, ensure_ascii=True, sort_keys=True, separators=(",", ":")
        ).encode()
        project = self._project_mapping
        return {
            "schema_version": "iraf.execution-replay/v1",
            "execution_id": execution_id,
            "request_digest": request_digest,
            "result_digest": hashlib.sha256(result_json.encode()).hexdigest(),
            "subject_digest": hashlib.sha256(subject.encode()).hexdigest(),
            "idempotency_key_digest": hashlib.sha256(
                idempotency_key.encode()
            ).hexdigest(),
            "correlation_id": result.get("correlation_id", ""),
            "terminal_status": result.get("status", ""),
            "terminal_sequence": int(result.get("sequence", 0)),
            "error_code": result.get("error_code", ""),
            "reason": result.get("reason", ""),
            "requested_skill": project(
                result.get("requested_skill"), ("name", "version_constraint")
            ),
            "skill": project(result.get("skill"), ("name", "version", "digest")),
            "provider": project(result.get("provider"), ("name", "type")),
            "profile": project(
                result.get("profile"), ("name", "version", "digest")
            ),
            "safety_policy": project(
                result.get("safety_policy"), ("name", "version", "digest")
            ),
            "policy_decision_id": result.get("policy_decision_id", ""),
            "policy_version": result.get("policy_version", ""),
            "resource_id": result.get("resource_id", ""),
            "controller": result.get("controller", ""),
            "simulation": bool(result.get("simulation", False)),
            "adapter": result.get("adapter", ""),
            "intent_provider": project(
                result.get("intent_provider"), ("name", "version", "model")
            ),
            "intent_request_digest": result.get("intent_request_digest", ""),
            "resolved_skill": result.get("resolved_skill", ""),
            "events": events,
            "event_digest": hashlib.sha256(event_payload).hexdigest(),
            "completed_at_ms": int(completed_at_ms),
        }

    @staticmethod
    def _project_mapping(value, fields):
        """只导出公共契约允许的嵌套字段，兼容历史或异常持久化数据。"""
        if not isinstance(value, dict):
            return {}
        return {field: value[field] for field in fields if field in value}

    @staticmethod
    def _safety_record(row):
        return {"resource": row[0], "event_id": row[1], "kind": row[2], "reason": row[3], "occurred_at_ns": row[4]}

    def save_safety_event(self, event):
        """Persist the first active event for a resource and return the winner."""
        with self._lock:
            cur = self._db.execute("INSERT OR IGNORE INTO safety_events(resource,event_id,kind,reason,occurred_at_ns) VALUES(?,?,?,?,?)", (event.resource, event.event_id, event.kind, event.reason, event.occurred_at_ns))
            if cur.rowcount == 1:
                self._db.execute("INSERT OR IGNORE INTO safety_event_history(event_id,resource,kind,reason,occurred_at_ns) VALUES(?,?,?,?,?)", (event.event_id, event.resource, event.kind, event.reason, event.occurred_at_ns))
            self._db.commit()
            row = self._db.execute("SELECT resource,event_id,kind,reason,occurred_at_ns FROM safety_events WHERE resource=?", (event.resource,)).fetchone()
            return self._safety_record(row) if row else None

    def get_safety_event(self, resource):
        with self._lock:
            row = self._db.execute("SELECT resource,event_id,kind,reason,occurred_at_ns FROM safety_events WHERE resource=?", (resource,)).fetchone()
            return self._safety_record(row) if row else None

    def delete_safety_event(self, resource, event_id, actor="", recovered_at_ns=None):
        with self._lock:
            recovered_at_ns = int(recovered_at_ns or time.monotonic_ns())
            self._db.execute("UPDATE safety_event_history SET recovered_at_ns=?,recovered_by=? WHERE event_id=? AND resource=?", (recovered_at_ns, str(actor or ""), event_id, resource))
            cur = self._db.execute("DELETE FROM safety_events WHERE resource=? AND event_id=?", (resource, event_id))
            self._db.commit()
            return cur.rowcount == 1

    def list_events(self, execution_id="", object_id="", since_unix_ms=0, until_unix_ms=0, limit=100, offset=0):
        """Return a bounded, deterministic view for the development EventService."""
        limit = max(1, min(int(limit or 100), 1000))
        offset = max(0, int(offset or 0))
        with self._lock:
            execution_where = []
            execution_args = []
            if execution_id:
                execution_where.append("execution_id=?")
                execution_args.append(execution_id)
            if object_id:
                execution_where.append("execution_id=?")
                execution_args.append(object_id)
            if since_unix_ms:
                execution_where.append("occurred_at_ms>=?")
                execution_args.append(int(since_unix_ms))
            if until_unix_ms:
                execution_where.append("occurred_at_ms<=?")
                execution_args.append(int(until_unix_ms))
            execution_sql = "SELECT execution_id,sequence,status,reason,occurred_at_ms FROM execution_events"
            if execution_where:
                execution_sql += " WHERE " + " AND ".join(execution_where)
            execution_rows = self._db.execute(execution_sql, execution_args).fetchall()

            safety_rows = []
            if not execution_id:
                safety_where = []
                safety_args = []
                if object_id:
                    safety_where.append("resource=?")
                    safety_args.append(object_id)
                if since_unix_ms:
                    safety_where.append("occurred_at_ns>=?")
                    safety_args.append(int(since_unix_ms) * 1_000_000)
                if until_unix_ms:
                    safety_where.append("occurred_at_ns<=?")
                    safety_args.append(int(until_unix_ms) * 1_000_000)
                safety_sql = "SELECT event_id,resource,kind,reason,occurred_at_ns,recovered_at_ns,recovered_by FROM safety_event_history"
                if safety_where:
                    safety_sql += " WHERE " + " AND ".join(safety_where)
                safety_rows = self._db.execute(safety_sql, safety_args).fetchall()

        events = [{"event_id": f"{row[0]}:{row[1]}", "object_id": row[0], "execution_id": row[0], "sequence": int(row[1]), "kind": "EXECUTION_STATE", "status": row[2], "reason": row[3], "occurred_at_ms": int(row[4]), "recovered_at_ms": 0, "recovered_by": ""} for row in execution_rows]
        events.extend({"event_id": row[0], "object_id": row[1], "execution_id": "", "sequence": 0, "kind": row[2], "status": "QUARANTINED", "reason": row[3], "occurred_at_ms": int(row[4] // 1_000_000), "recovered_at_ms": int(row[5] // 1_000_000) if row[5] else 0, "recovered_by": row[6] or ""} for row in safety_rows)
        events.sort(key=lambda item: (item["occurred_at_ms"], item["event_id"]))
        page = events[offset:offset + limit]
        next_offset = offset + limit if offset + limit < len(events) else None
        return page, next_offset
