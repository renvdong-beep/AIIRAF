"""可替换的 SQLite execution/event store。"""
import json,sqlite3,threading,time
class SqliteExecutionStore:
    def __init__(self,path):
        self._db=sqlite3.connect(path,check_same_thread=False); self._lock=threading.RLock(); self._db.execute("CREATE TABLE IF NOT EXISTS executions(execution_id TEXT PRIMARY KEY,subject TEXT NOT NULL,idempotency_key TEXT NOT NULL,request_digest TEXT NOT NULL,result_json TEXT NOT NULL,updated_at_ms INTEGER NOT NULL,UNIQUE(subject,idempotency_key))"); self._db.execute("CREATE TABLE IF NOT EXISTS execution_events(execution_id TEXT NOT NULL,sequence INTEGER NOT NULL,status TEXT NOT NULL,reason TEXT NOT NULL,occurred_at_ms INTEGER NOT NULL,PRIMARY KEY(execution_id,sequence))"); self._db.commit()
    def find_idempotent(self,subject,key):
        with self._lock:
            row=self._db.execute("SELECT request_digest,result_json FROM executions WHERE subject=? AND idempotency_key=?",(subject,key)).fetchone()
            return (row[0],json.loads(row[1])) if row else None
    def save_result(self,subject,key,digest,result):
        with self._lock:
            self._db.execute("INSERT INTO executions VALUES(?,?,?,?,?,?)",(result["execution_id"],subject,key,digest,json.dumps(result,ensure_ascii=True,sort_keys=True),int(time.time()*1000))); self._db.commit()
    def append_event(self,execution_id,sequence,status,reason=""):
        with self._lock:
            self._db.execute("INSERT OR IGNORE INTO execution_events VALUES(?,?,?,?,?)",(execution_id,sequence,status,reason,int(time.time()*1000))); self._db.commit()
    def get(self,execution_id):
        with self._lock:
            row=self._db.execute("SELECT result_json FROM executions WHERE execution_id=?",(execution_id,)).fetchone(); return json.loads(row[0]) if row else None