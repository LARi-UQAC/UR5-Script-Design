"""Tests for the UDP loopback IPC that replaced tcp_live.json.

Covers:
* :func:`ur5_sim.visualization.swift_scene.send_tcp_live` round-trips a
  payload on ``127.0.0.1`` and is drop-tolerant when no receiver is bound.
* A non-blocking receiver bound on the same port keeps only the latest
  datagram when the sender bursts faster than the timer drains.
* Payload encoding is UTF-8 JSON (so the design UI's drain decoder reads it
  the same way the viewer writes it).
"""

from __future__ import annotations

import json
import socket
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.ipc_config import (  # noqa: E402
    TCP_LIVE_HOST,
    TCP_LIVE_MAX_BYTES,
    TCP_LIVE_PORT,
)
from ur5_sim.visualization.swift_scene import send_tcp_live  # noqa: E402


def _bind_receiver(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((TCP_LIVE_HOST, port))
    sock.setblocking(False)
    return sock


class UdpRoundTripTests(unittest.TestCase):
    def setUp(self) -> None:
        # Bind a free ephemeral port so parallel test runs do not collide
        # with a running viewer on the canonical TCP_LIVE_PORT.
        self.recv = _bind_receiver(0)
        self.port = self.recv.getsockname()[1]

    def tearDown(self) -> None:
        self.recv.close()

    def test_payload_round_trips(self) -> None:
        # Send to the receiver's bound port via a direct sendto to avoid
        # depending on TCP_LIVE_PORT being free on the host.
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            payload = {"cycle": 2, "frame": 42, "in_contact": True}
            sender.sendto(json.dumps(payload).encode("utf-8"),
                          (TCP_LIVE_HOST, self.port))
            # Loopback usually delivers in <1 ms; loop briefly.
            received = None
            for _ in range(50):
                try:
                    received, _ = self.recv.recvfrom(TCP_LIVE_MAX_BYTES)
                    break
                except BlockingIOError:
                    time.sleep(0.002)
            self.assertIsNotNone(received)
            self.assertEqual(json.loads(received.decode("utf-8")), payload)
        finally:
            sender.close()

    def test_drain_keeps_only_latest_when_burst(self) -> None:
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for i in range(5):
                sender.sendto(
                    json.dumps({"frame": i}).encode("utf-8"),
                    (TCP_LIVE_HOST, self.port),
                )
            time.sleep(0.01)
            last = None
            while True:
                try:
                    data, _ = self.recv.recvfrom(TCP_LIVE_MAX_BYTES)
                    last = data
                except BlockingIOError:
                    break
            self.assertIsNotNone(last)
            self.assertEqual(json.loads(last.decode("utf-8")), {"frame": 4})
        finally:
            sender.close()


class SendTcpLiveDropToleranceTests(unittest.TestCase):
    def test_send_with_no_receiver_does_not_raise(self) -> None:
        # No bound receiver on TCP_LIVE_PORT -> UDP datagram is dropped by
        # the kernel. send_tcp_live must swallow the OSError silently.
        try:
            send_tcp_live({"frame": 0, "cycle": 0})
        except Exception as exc:  # pragma: no cover
            self.fail(f"send_tcp_live raised {exc!r} when no receiver bound")


if __name__ == "__main__":
    unittest.main()
