# -*- coding: utf-8 -*-
"""JointMujocoBackend 契约：能力并集 + resource_id 路由（P1）。"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_adapters.mujoco.joint_backend import JointMujocoBackend, JointBindingError  # noqa: E402


class _Lease:
    def __init__(self, resource_id):
        self.resource_id = resource_id


class JointBackendContractTest(unittest.TestCase):
    def _backend(self):
        backend = object.__new__(JointMujocoBackend)
        backend._resources = dict(JointMujocoBackend.DEFAULT_RESOURCES)
        return backend

    def test_capabilities_cover_both_robots(self):
        caps = set(JointMujocoBackend.IMPLEMENTED_CAPABILITIES)
        for name in ("pick_object", "place_object", "move_joint",
                     "stand", "stop", "locomote", "dock_for_handoff",
                     "accept_payload", "read_state"):
            self.assertIn(name, caps)

    def test_routes_by_resource_id(self):
        backend = self._backend()
        self.assertEqual("arm", backend._kind(_Lease("piper"))[0])
        self.assertEqual("arm", backend._kind(_Lease("ur5"))[0])
        self.assertEqual("quadruped", backend._kind(_Lease("go2"))[0])
        self.assertEqual("quadruped", backend._kind(_Lease("unitree_go2"))[0])

    def test_unknown_resource_fails_closed(self):
        backend = self._backend()
        with self.assertRaises(JointBindingError):
            backend._kind(_Lease("unknown_robot"))

    def test_missing_resource_id_fails_closed(self):
        backend = self._backend()
        with self.assertRaises(JointBindingError):
            backend._kind(_Lease(""))


if __name__ == "__main__":
    unittest.main()
