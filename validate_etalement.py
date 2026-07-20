"""Deprecated shim - use ``python -m ur5_sim`` or ``run_validate.py`` instead.

This module is kept so the original command line keeps working after the
refactor that split the monolithic file into the ``ur5_sim`` package.
"""

from __future__ import annotations

import sys

from ur5_sim.cli import main

if __name__ == "__main__":
    sys.exit(main())
