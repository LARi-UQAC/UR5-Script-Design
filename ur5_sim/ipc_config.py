"""Shared IPC parameters between ``ur5_sim`` (writer) and
``ur5_etalementv6`` (reader).

UDP unicast on ``127.0.0.1`` replaces the previous ``tcp_live/tcp_live.json``
file IPC. Loopback latency is ~20-100 us versus ~1-10 ms (and 50-500 ms tail)
for the atomic file write the design UI used to poll. Drop-tolerant: the
overlay only needs the latest frame, so any UDP datagram loss is invisible
(the next frame 16 ms later supersedes it).
"""

from __future__ import annotations

# Loopback address. Binding the receiver here is exempted from the Windows
# firewall prompt and is reachable only from the same machine, so the
# telemetry channel has no external attack surface.
TCP_LIVE_HOST: str = "127.0.0.1"

# Ephemeral, unprivileged port (> 1024, IANA private/dynamic range
# 49152-65535 by spec but 47811 is commonly free in practice on dev
# workstations). Override via ``UR5_SIM_IPC_PORT`` env var if it clashes
# with another local service.
import os as _os  # noqa: E402

_DEFAULT_PORT = 47811
try:
    TCP_LIVE_PORT: int = int(_os.environ.get("UR5_SIM_IPC_PORT", _DEFAULT_PORT))
except ValueError:
    TCP_LIVE_PORT = _DEFAULT_PORT

# Generous safety margin over the current payload (~400 B JSON for a
# trajectory carrying ~50 trail points). 4 KB stays well below the typical
# loopback MTU and avoids datagram fragmentation.
TCP_LIVE_MAX_BYTES: int = 65_507  # max UDP IPv4 payload; recv buffer size
