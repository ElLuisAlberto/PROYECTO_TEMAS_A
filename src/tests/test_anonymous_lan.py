"""LAN mode selection remains server-controlled without visible logins."""

import unittest

from teleop_cooperativo.gateway import CooperativeGateway


class AnonymousLanTest(unittest.TestCase):
    def setUp(self):
        credentials = {"operators": ("a" * 40,), "observers": ("b" * 40,), "admins": ("c" * 40,)}
        self.gateway = CooperativeGateway(credentials=credentials, anonymous_lan=True)
        self.gateway.mark_ready()

    def enter(self, name, role, *, remote=True, credential=None):
        self.gateway.open_connection(name, remote_control_allowed=remote)
        hello = {"type": "hello", "room_id": "default", "operator_name": name, "role": role}
        if credential is not None:
            hello["credential"] = credential
        return self.gateway.receive(name, hello)[0]

    def test_remote_availability_and_exclusive_capacity(self):
        self.assertEqual(self.enter("watch", "observer")["type"], "welcome")
        self.assertTrue(self.gateway.room_state()["modes"]["operator"])
        self.assertEqual(self.enter("op-a", "operator")["type"], "welcome")
        self.assertEqual(self.enter("op-b", "operator")["type"], "welcome")
        self.assertFalse(self.gateway.room_state()["modes"]["operator"])
        self.assertEqual(self.enter("op-c", "operator")["code"], "mode_unavailable")
        self.gateway.close_connection("op-a")
        self.assertTrue(self.gateway.room_state()["modes"]["operator"])
        self.assertEqual(self.enter("op-d", "operator")["type"], "welcome")
        self.assertEqual(self.enter("admin-a", "admin")["type"], "welcome")
        self.assertFalse(self.gateway.room_state()["modes"]["admin"])
        self.assertEqual(self.enter("admin-b", "admin")["code"], "mode_unavailable")
        self.gateway.close_connection("admin-a")
        self.assertTrue(self.gateway.room_state()["modes"]["admin"])

    def test_local_web_cannot_operate_or_open_admin(self):
        for role in ("operator", "admin", "observer"):
            self.assertEqual(self.enter("local-" + role, role, remote=False)["code"], "authentication_failed")
        self.assertEqual(self.enter("native", "observer", remote=False, credential="b" * 40)["role"], "observer")
        self.assertEqual(self.gateway.receive("native", {"type": "claim", "axes": ["X"]})[0]["code"], "forbidden")

    def test_atomic_claims_disconnect_and_fault(self):
        self.enter("a", "operator")
        self.enter("b", "operator")
        older_revision = self.gateway.room_state()["revision"]
        grant = self.gateway.receive("a", {"type": "claim", "axes": ["X", "Y"]})[0]
        self.assertEqual(grant["type"], "claim_granted")
        self.assertGreater(grant["revision"], older_revision)
        self.assertEqual(grant["revision"], self.gateway.room_state()["revision"])
        self.assertEqual(self.gateway.receive("b", {"type": "claim", "axes": ["X", "Z"]})[0]["code"], "claim_denied")
        self.assertFalse(self.gateway.room_state()["axes"]["Z"]["assigned"])
        self.gateway.close_connection("a")
        self.assertFalse(self.gateway.room_state()["axes"]["X"]["assigned"])
        self.assertEqual(self.gateway.receive("b", {"type": "claim", "axes": ["X"]})[0]["type"], "claim_granted")
        self.enter("admin", "admin")
        self.gateway.receive("admin", {"type": "stop_global"})
        self.assertFalse(self.gateway.room_state()["modes"]["operator"])
        self.assertFalse(self.gateway.room_state()["axes"]["X"]["assigned"])
        self.assertEqual(self.gateway.receive("admin", {"type": "rearm"})[0]["type"], "rearm_confirmed")
        self.assertTrue(self.gateway.room_state()["modes"]["operator"])

    def test_anonymous_disabled_by_default(self):
        gateway = CooperativeGateway(credentials=self.gateway.credentials)
        gateway.mark_ready()
        gateway.open_connection("remote", remote_control_allowed=True)
        reply = gateway.receive("remote", {"type": "hello", "room_id": "default", "operator_name": "remote", "role": "operator"})
        self.assertEqual(reply[0]["code"], "authentication_failed")


if __name__ == "__main__":
    unittest.main()
