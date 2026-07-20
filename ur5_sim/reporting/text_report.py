"""Console-friendly report of the IK + surface validation outcome."""

from __future__ import annotations

from ur5_sim.config import P_ANCHOR_OLD_RAW, P_REF_RAW, SCRIPT_PATH

_SURFACE_KINDS = ("SURFACE_DEVIATION", "SURFACE_CLAMP")
_PROBE_KINDS = (
    "PROBE_OK",
    "PROBE_NO_CONTACT",
    "PROBE_TILT_EXCEEDED",
    "PROBE_RESIDUAL",
    "PROBE_UNREACHABLE",
    "PROBE_SKIPPED",
)
_SPEED_KINDS = ("SPEED_LIMIT_EXCEEDED",)
_SEGMENT_KINDS = ("SEGMENT_VELOCITY_EXCEEDED", "SEGMENT_VELOCITY_UNKNOWN")


def _format_detail(kind: str, detail: object) -> str:
    """Render the ``detail`` column for a given failure kind."""
    if kind in _SURFACE_KINDS:
        try:
            depth_mm = float(detail)
        except (TypeError, ValueError):
            return str(detail)
        if kind == "SURFACE_DEVIATION":
            return f"dz = {depth_mm:+7.3f} mm (snap on plane)"
        return f"penetration = {depth_mm:7.3f} mm (clamped above plane)"
    return str(detail)


def report(
    parsed: list[tuple[int, tuple[float, ...]]],
    failures: list[tuple[int, str, object]],
) -> None:
    """Print the validation summary in a fixed, easy-to-scan layout.

    The header always lists the active anchors so the reader can confirm
    which run they are looking at. Failures are split in two sections:
    surface constraints (``SURFACE_DEVIATION`` during ``force_mode``,
    ``SURFACE_CLAMP`` during transit) listed first so the reader can
    triage them before the IK / joint-limit defects.
    """
    print(f"Parsed {len(parsed)} pose(s) from {SCRIPT_PATH.name}")
    print(f"P_ANCHOR_OLD = {P_ANCHOR_OLD_RAW}")
    print(f"P_REF        = {P_REF_RAW}")
    print()

    speed_events = [f for f in failures if f[1] in _SPEED_KINDS]
    segment_events = [f for f in failures if f[1] in _SEGMENT_KINDS]
    probe_events = [f for f in failures if f[1] in _PROBE_KINDS]
    surface_fail = [f for f in failures if f[1] in _SURFACE_KINDS]
    other_fail = [
        f for f in failures
        if f[1] not in _SURFACE_KINDS
        and f[1] not in _PROBE_KINDS
        and f[1] not in _SPEED_KINDS
        and f[1] not in _SEGMENT_KINDS
    ]

    if speed_events:
        print(f"{len(speed_events)} TCP speed violation(s):")
        print(f"{'line':>6}  {'type':<26}  detail")
        print("-" * 72)
        for lineno, kind, detail in speed_events:
            print(f"{lineno:>6}  {kind:<26}  {detail}")
        print()

    if segment_events:
        print(f"{len(segment_events)} per-segment velocity event(s):")
        print(f"{'line':>6}  {'type':<26}  detail")
        print("-" * 72)
        for lineno, kind, detail in segment_events:
            print(f"{lineno:>6}  {kind:<26}  {detail}")
        print()

    if probe_events:
        print(f"{len(probe_events)} probe event(s):")
        print(f"{'line':>6}  {'type':<22}  detail")
        print("-" * 72)
        for lineno, kind, detail in probe_events:
            print(f"{lineno:>6}  {kind:<22}  {detail}")
        print()

    if surface_fail:
        print(f"{len(surface_fail)} surface constraint event(s):")
        print(f"{'line':>6}  {'type':<18}  detail")
        print("-" * 72)
        for lineno, kind, detail in surface_fail:
            print(f"{lineno:>6}  {kind:<18}  {_format_detail(kind, detail)}")
        print()
    else:
        print("Surface: tool stays on the 50x50 mm test plate "
              "throughout every force_mode block.")
        print()

    if not other_fail:
        print("IK / joint limits: all poses solved within joint limits.")
        return
    print(f"{len(other_fail)} IK / joint-limit failure(s):")
    print(f"{'line':>6}  {'type':<18}  detail")
    print("-" * 72)
    for lineno, kind, detail in other_fail:
        print(f"{lineno:>6}  {kind:<18}  {_format_detail(kind, detail)}")
