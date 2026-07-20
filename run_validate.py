"""Thin entry point so the existing ``validate.bat`` keeps working."""

from __future__ import annotations

import sys

from ur5_sim.cli import main

if __name__ == "__main__":
    sys.exit(main())
