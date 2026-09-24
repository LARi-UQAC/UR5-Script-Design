"""
venv_guard.py — fail fast when running outside this repo's .venv.

swift-sim (ur5_sim's 3D viewer) needs websockets<13; this repo's .venv pins
that in requirements.txt, but a global Python resolved first on PATH can
still run everything for minutes before dying deep inside a Swift
background thread with a cryptic RuntimeError (CLAUDE.md, "Dependency
pinning"). ensure_venv() is called at the top of ur5_sim/__init__.py and
design/__init__.py so the wrong interpreter is refused immediately instead.

Deliberately zero project imports (stdlib only), so it works before any
other module — including a partially-broken one under the wrong
interpreter — has a chance to import anything.
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent
_EXPECTED_VENV = _REPO_ROOT / ".venv"


def ensure_venv() -> None:
    exe = Path(sys.executable).resolve()
    if _EXPECTED_VENV not in exe.parents:
        print(
            "ERROR: not running from this project's .venv.\n"
            f"  Running under : {exe}\n"
            f"  Expected under: {_EXPECTED_VENV}\n"
            "  Run via validate.bat, or activate the venv first:\n"
            f"    {_EXPECTED_VENV}\\Scripts\\activate\n",
            file=sys.stderr,
        )
        sys.exit(1)
