import json
import tempfile
import unittest
from pathlib import Path

from tools.task_planner import emit_ready, load_plan


class TaskPlannerTests(unittest.TestCase):
    def test_emits_root_then_dependent_task(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            (root / "root.patch").write_text("diff --git a/a b/a\n", encoding="utf-8")
            (root / "child.patch").write_text("diff --git a/b b/b\n", encoding="utf-8")
            plan = {
                "version": 1,
                "plan_id": "planner-test",
                "tasks": [
                    {"id": "root-task", "title": "test: root", "verification_profile": "unit", "patch": "root.patch"},
                    {"id": "child-task", "title": "test: child", "verification_profile": "unit", "patch": "child.patch", "depends_on": ["root-task"]},
                ],
            }
            plan_path = root / "plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            queue = root / "queue" / "inbox"
            evidence = root / "evidence"
            self.assertEqual(emit_ready(load_plan(plan_path), plan_path, queue, evidence), ["root-task"])
            (evidence / "root-task.json").parent.mkdir(parents=True, exist_ok=True)
            (evidence / "root-task.json").write_text('{"task_id":"root-task","passed":true}', encoding="utf-8")
            self.assertEqual(emit_ready(load_plan(plan_path), plan_path, queue, evidence), ["child-task"])

    def test_rejects_dependency_cycle(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "plan.json"
            path.write_text(json.dumps({"version": 1, "tasks": [
                {"id": "task-one", "title": "one", "verification_profile": "unit", "patch": "one.patch", "depends_on": ["task-two"]},
                {"id": "task-two", "title": "two", "verification_profile": "unit", "patch": "two.patch", "depends_on": ["task-one"]},
            ]}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "环"):
                load_plan(path)


if __name__ == "__main__":
    unittest.main()
