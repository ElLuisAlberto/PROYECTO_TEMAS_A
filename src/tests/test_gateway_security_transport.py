"""Exercise authorization and origin checks over actual WebSockets."""

import asyncio
import json
import socket
import unittest

import websockets

from teleop_cooperativo.gateway import CooperativeGateway, serve_websocket


class SecureTransportTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.gateway = CooperativeGateway(credentials={"operators": ("a" * 40,), "observers": ("b" * 40,), "admins": ("c" * 40,)})
        self.gateway.mark_ready()
        self.task = asyncio.create_task(serve_websocket(self.gateway, host="127.0.0.1", port=self.port, allowed_origins=("https://192.168.1.10:8443",), trusted_proxy="127.0.0.1", base_local_ips=("192.168.1.10",)))
        for _ in range(100):
            try:
                reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
                writer.close()
                await writer.wait_closed()
                break
            except OSError:
                await asyncio.sleep(0.01)
        else:
            self.fail("gateway did not start")

    async def asyncTearDown(self):
        self.task.cancel()
        try:
            await self.task
        except asyncio.CancelledError:
            pass

    async def hello(self, ws, role, token):
        await ws.send(json.dumps({"type": "hello", "room_id": "default", "operator_name": "Test", "role": role, "credential": token}))
        return json.loads(await ws.recv())

    async def test_no_state_before_auth_and_local_base_cannot_operate(self):
        origin = "https://192.168.1.10:8443"
        async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws", origin=origin, additional_headers={"X-Forwarded-For": "192.168.1.10"}) as ws:
            with self.assertRaises(asyncio.TimeoutError):
                await asyncio.wait_for(ws.recv(), 0.08)
            reply = await self.hello(ws, "operator", "a" * 40)
            self.assertEqual(reply["code"], "authentication_failed")
        async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws", origin=origin, additional_headers={"X-Forwarded-For": "192.168.1.20"}) as ws:
            self.assertEqual((await self.hello(ws, "operator", "a" * 40))["type"], "welcome")
        async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws") as ws:
            self.assertEqual((await self.hello(ws, "observer", "b" * 40))["role"], "observer")

    async def test_browser_origin_rejected(self):
        with self.assertRaises(Exception):
            async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws", origin="https://attacker.invalid"):
                self.fail("untrusted origin connected")

    async def test_local_web_can_operate_only_when_explicitly_enabled(self):
        self.gateway.anonymous_lan = True
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        task = asyncio.create_task(serve_websocket(
            self.gateway, host="127.0.0.1", port=port,
            allowed_origins=("https://192.168.1.10:8443",),
            trusted_proxy="127.0.0.1", base_local_ips=("192.168.1.10",),
            allow_local_web=True,
        ))
        try:
            for _ in range(100):
                try:
                    reader, writer = await asyncio.open_connection("127.0.0.1", port)
                    writer.close()
                    await writer.wait_closed()
                    break
                except OSError:
                    await asyncio.sleep(0.01)
            async with websockets.connect(
                f"ws://127.0.0.1:{port}/ws", origin="https://192.168.1.10:8443",
                additional_headers={"X-Forwarded-For": "192.168.1.10"},
            ) as ws:
                self.assertEqual((await self.hello(ws, "operator", ""))["type"], "welcome")
                await ws.send(json.dumps({"type": "claim", "axes": ["X"]}))
                while True:
                    reply = json.loads(await asyncio.wait_for(ws.recv(), 1.0))
                    if reply["type"] == "claim_granted":
                        break
                self.assertTrue(self.gateway.room_state()["axes"]["X"]["assigned"])
            with self.assertRaises(Exception):
                async with websockets.connect(
                    f"ws://127.0.0.1:{port}/ws", origin="https://attacker.invalid",
                    additional_headers={"X-Forwarded-For": "192.168.1.10"},
                ):
                    self.fail("untrusted origin connected")
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def test_normal_close_releases_operator_axes(self):
        origin = "https://192.168.1.10:8443"
        async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws", origin=origin, additional_headers={"X-Forwarded-For": "192.168.1.20"}) as ws:
            self.assertEqual((await self.hello(ws, "operator", "a" * 40))["type"], "welcome")
            await ws.send(json.dumps({"type": "claim", "axes": ["X"]}))
            while True:
                reply = json.loads(await asyncio.wait_for(ws.recv(), 1.0))
                if reply["type"] == "claim_granted":
                    break
            self.assertTrue(self.gateway.room_state()["axes"]["X"]["assigned"])
        for _ in range(50):
            if not self.gateway.room_state()["axes"]["X"]["assigned"]:
                break
            await asyncio.sleep(0.01)
        self.assertFalse(self.gateway.room_state()["axes"]["X"]["assigned"])

    async def test_anonymous_mode_availability_is_broadcast_on_connect_and_close(self):
        self.gateway.anonymous_lan = True
        origin = "https://192.168.1.10:8443"
        url = f"ws://127.0.0.1:{self.port}/ws"
        headers = {"X-Forwarded-For": "192.168.1.20"}

        async def wait_for_state(ws, predicate):
            async def receive():
                while True:
                    message = json.loads(await ws.recv())
                    if message.get("type") == "room_state" and predicate(message):
                        return message
            return await asyncio.wait_for(receive(), 2)

        async with websockets.connect(url, origin=origin, additional_headers=headers) as watch:
            self.assertEqual((await self.hello(watch, "observer", ""))["type"], "welcome")
            async with websockets.connect(url, origin=origin, additional_headers=headers) as first:
                self.assertEqual((await self.hello(first, "operator", ""))["type"], "welcome")
                async with websockets.connect(url, origin=origin, additional_headers=headers) as second:
                    self.assertEqual((await self.hello(second, "operator", ""))["type"], "welcome")
                    await wait_for_state(watch, lambda state: not state["modes"]["operator"])
                await wait_for_state(watch, lambda state: state["modes"]["operator"])

    async def test_silent_operator_releases_mode_after_disconnect_timeout(self):
        self.gateway.anonymous_lan = True
        origin = "https://192.168.1.10:8443"
        headers = {"X-Forwarded-For": "192.168.1.20"}
        async with websockets.connect(f"ws://127.0.0.1:{self.port}/ws", origin=origin, additional_headers=headers) as ws:
            self.assertEqual((await self.hello(ws, "operator", ""))["type"], "welcome")
            await asyncio.wait_for(ws.wait_closed(), 3)
        self.assertFalse(any(session.role == "operator" for session in self.gateway._sessions.values()))
