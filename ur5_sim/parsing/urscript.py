"""URScript parsing and pose-math helpers.

Reads ``etalement.script`` and extracts every ``movel(T(p[...]))`` /
``movej(T(p[...]))`` call so the trajectory can be reasoned about offline.
Also reimplements URScript ``pose_trans`` / ``pose_inv`` on top of
spatialmath SE3, so the on-robot wrapper ``T(p_orig) = pose_trans(P_REF,
pose_trans(pose_inv(P_ANCHOR_OLD), p_orig))`` can be evaluated in Python.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from spatialmath import SE3, SO3

# Forme historique du URScript : ``movel(T(p[...]))``. Conservee pour
# documenter le contrat d'origine et faire echec rapide sur les lignes qui ne
# transitent pas par le wrapper ``T(...)``.
POSE_RE = re.compile(
    r"move[lj]\(T\(p\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,"
    r"\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]\)"
)
# Forme generique : un literal ``p[x, y, z, rx, ry, rz]`` n'importe ou dans
# la ligne (couvre aussi bien ``movel(T(p[...]))``, ``movel(p[...])`` que
# ``movel(apply_correction(p[...], dx, dy))`` emis par les variantes
# recentes de ``generate_urscript``).
ANY_POSE_RE = re.compile(
    r"p\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,"
    r"\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]"
)
MOVE_LINE_RE = re.compile(r"^\s*move[lj]\b")

CYCLE_DEF_RE = re.compile(r"^\s*def\s+cycle_(\d+)\s*\(\s*\)\s*:")
# Bloc de sondage 3 points emis par ur5_etalementv6._build_urscript_lines.
# Les appels ``cp1 = probe_one(p[approach], p[floor], "P1")`` portent les deux
# poses absolues monde (haute = approche, basse = plancher). Le simulateur
# rejoue les descentes en y intersectant un plan virtuel parametre dans
# ``ur5_sim.config`` pour valider la reconstruction de ``MEAS_FRAME``.
PROBE_DEF_RE = re.compile(r"^\s*def\s+probe_surface_plane\s*\(\s*\)\s*:")
PROBE_ONE_RE = re.compile(
    r"^\s*cp(\d+)\s*=\s*probe_one\(\s*"
    r"p\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,"
    r"\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]"
    r"\s*,\s*"
    r"p\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,"
    r"\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]"
)
NOMINAL_FRAME_GLOBAL_RE = re.compile(
    r"^\s*global\s+NOMINAL_FRAME\s*=\s*"
    r"p\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,"
    r"\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]"
)
NHAT_GLOBAL_RE = re.compile(r"^\s*global\s+NHAT_([XYZ])\s*=\s*([+\-0-9.eE]+)")
# Vitesses TCP emises comme ``global <NAME> = <value>`` au preambule du
# script. Le simulateur lit ces globaux et compare a la limite PolyScope
# (URSCRIPT_MAX_TCP_SPEED, mirroir cote ur5_sim.config).
SPEED_GLOBAL_RE = re.compile(
    r"^\s*global\s+([A-Z_][A-Z0-9_]*)\s*=\s*([+\-0-9.eE]+)"
)
# Noms emis par ur5_etalementv6._build_urscript_lines qui sont des vitesses
# TCP (m/s), distinguees des autres globaux scalaires (force, blend, ...).
TCP_SPEED_GLOBAL_NAMES = frozenset({
    # URSCRIPT_TRANSIT_V est inline directement sur chaque movel de transit
    # (cf. design/export.py) ; il n'est plus declare comme global dans le
    # script. La verification cote sim s'appuie desormais sur les autres
    # vitesses TCP nommees et sur la valeur de design.params.URSCRIPT_TRANSIT_V
    # importee via ur5_sim.config.
    "URSCRIPT_CONTACT_V",
    "URSCRIPT_RECONTACT_V",
    "V_CIRC",
    "V_RECT",
    "PROBE_DESCENT_V",
})
# Argument ``v=<expr>`` sur une ligne movel/movej. L'exporter ne genere que
# deux formes : ``<NAME_or_literal>`` ou ``<NAME_or_literal>*<NAME_or_literal>``
# (cf. design/export.py). Le groupe capture l'expression brute ; l'evaluation
# se fait via :func:`_eval_speed_expr` apres lookup dans la table des globaux.
MOVE_V_RE = re.compile(r"\bv\s*=\s*([A-Za-z_][A-Za-z0-9_.]*|[+\-0-9.]+)(?:\s*\*\s*([A-Za-z_][A-Za-z0-9_.]*|[+\-0-9.]+))?")
# Discrimine ``movel`` (vitesse en m/s) vs ``movej`` (vitesse en rad/s).
MOVE_KIND_RE = re.compile(r"^\s*(move[lj])\b")


# ``force_mode(...)``/``end_force_mode()`` delimitent les phases de contact
# pendant lesquelles le regulateur de force impose Z constant sur la surface.
# Le simulateur les utilise pour activer le snap bidirectionnel (cf.
# ``ur5_sim.visualization.surface.apply_surface_constraint``).
FORCE_MODE_RE = re.compile(r"^\s*force_mode\s*\(")
END_FORCE_RE = re.compile(r"^\s*end_force_mode\s*\(")


def urscript_pose(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> SE3:
    """Convert a URScript pose tuple to a spatialmath SE3.

    URScript orientation is encoded as an axis-angle rotation vector :
    direction = rotation axis, magnitude = angle in radians. spatialmath's
    ``SO3.EulerVec`` implements this convention exactly.
    """
    return SE3.Rt(SO3.EulerVec([rx, ry, rz]), [x, y, z])


def parse_poses(
    script_path: Path,
) -> list[tuple[int, tuple[float, ...], int, bool]]:
    """Extract ``(line_number, pose_tuple, cycle_idx, in_contact)`` per move call.

    Every ``movel``/``movej`` line is matched, regardless of the helper that
    wraps the pose literal (``T(p[...])``, ``apply_correction(p[...], ...)``
    or the plain ``p[...]`` baked by recent versions of
    ``generate_urscript``). The first 6-tuple ``p[x, y, z, rx, ry, rz]`` found
    on the line is returned.

    ``cycle_idx`` (1-based) is the index of the enclosing ``def cycle_N():``
    block. Poses appearing before any ``def cycle_N():`` (e.g. the global
    init lines in ``def etalement():``) are tagged with ``0`` so they can be
    filtered out by the live-display layer.

    ``in_contact`` is ``True`` when the pose sits between a ``force_mode(...)``
    and the matching ``end_force_mode()`` inside the same ``def cycle_N():``
    block. It drives the kinematic surrogate of the 6 N regulation in
    :mod:`ur5_sim.visualization.surface`: contact poses get snapped onto the
    surface plane, transit poses are only clamped from below.
    """
    poses: list[tuple[int, tuple[float, ...], int, bool]] = []
    current_cycle = 0
    in_contact = False
    with script_path.open("r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, start=1):
            cycle_m = CYCLE_DEF_RE.match(line)
            if cycle_m:
                current_cycle = int(cycle_m.group(1))
                # Le regulateur de force est explicitement (re)demarre dans
                # chaque ``def cycle_N():`` ; un eventuel oubli de
                # ``end_force_mode()`` ne doit pas fuir d'un cycle a l'autre.
                in_contact = False
                continue
            if FORCE_MODE_RE.match(line):
                in_contact = True
                continue
            if END_FORCE_RE.match(line):
                in_contact = False
                continue
            if not MOVE_LINE_RE.match(line):
                continue
            match = ANY_POSE_RE.search(line)
            if match:
                pose = tuple(float(v) for v in match.groups())
                poses.append((lineno, pose, current_cycle, in_contact))
    return poses


def parse_poses_legacy(
    script_path: Path,
) -> list[tuple[int, tuple[float, ...], int]]:
    """Retro-compatible shim returning the historical 3-tuple shape.

    Drops the ``in_contact`` field added in 2026. Kept so any third-party
    caller importing ``parse_poses`` from before the surface-constraint work
    keeps compiling. New code must consume :func:`parse_poses` directly.
    """
    return [
        (lineno, pose, cycle)
        for lineno, pose, cycle, _in_contact in parse_poses(script_path)
    ]


def parse_probe_blocks(
    script_path: Path,
) -> list[tuple[int, int, tuple[float, ...], tuple[float, ...]]]:
    """Extract the 3 probe descents emitted by ``probe_surface_plane()``.

    DESACTIVE - A REVOIR (rework futur) : le sondage 3 points est INCORRECT et
    n'est plus emis par l'export (remplace par probe_surface_z, 1 point en Z).
    Sur le script actuel cette fonction retourne donc une liste vide ; elle est
    conservee (avec parse_nhat) pour le rework. Voir ur5_sim/probe.py.

    Returns
    -------
    list of ``(probe_idx, lineno, approach_pose, floor_pose)``
        ``probe_idx`` is the 1-based index of the probe point (cp1 -> 1, ...).
        ``approach_pose`` and ``floor_pose`` are 6-tuples ``(x, y, z, rx, ry, rz)``
        in absolute world coordinates (already baked by ``_abs_pose``).

    The list is empty when the script does not contain a ``def
    probe_surface_plane():`` block. Order follows the order of appearance in
    the script so caller can index by position (cp1, cp2, cp3).
    """
    blocks: list[tuple[int, int, tuple[float, ...], tuple[float, ...]]] = []
    inside_probe_def = False
    with script_path.open("r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, start=1):
            if PROBE_DEF_RE.match(line):
                inside_probe_def = True
                continue
            if not inside_probe_def:
                continue
            # End of probe_surface_plane(): a top-level ``end`` (column 0)
            # closes the URScript def; subsequent ``end`` tokens are nested.
            if line.startswith("end"):
                inside_probe_def = False
                continue
            m = PROBE_ONE_RE.match(line)
            if not m:
                continue
            idx = int(m.group(1))
            approach = tuple(float(v) for v in m.groups()[1:7])
            floor = tuple(float(v) for v in m.groups()[7:13])
            blocks.append((idx, lineno, approach, floor))
    return blocks


def parse_nominal_frame(script_path: Path) -> tuple[float, ...] | None:
    """Return the absolute ``NOMINAL_FRAME`` pose tuple declared in the script.

    Used by the simulator to reconstruct ``MEAS_FRAME`` in Python the same way
    URScript would on the controller. Returns ``None`` if the global is absent
    (older scripts without the probe block).
    """
    with script_path.open("r", encoding="utf-8") as f:
        for line in f:
            m = NOMINAL_FRAME_GLOBAL_RE.match(line)
            if m:
                return tuple(float(v) for v in m.groups())
    return None


def parse_nhat(script_path: Path) -> tuple[float, float, float] | None:
    """Return the nominal plate normal ``(NHAT_X, NHAT_Y, NHAT_Z)`` from globals.

    DESACTIVE - A REVOIR (rework futur) : les globals NHAT_* etaient propres au
    sondage 3 points (INCORRECT), retire de l'export. Retourne donc ``None`` sur
    le script actuel. Conserve pour le rework. Voir ur5_sim/probe.py.
    """
    components: dict[str, float] = {}
    with script_path.open("r", encoding="utf-8") as f:
        for line in f:
            m = NHAT_GLOBAL_RE.match(line)
            if m:
                components[m.group(1)] = float(m.group(2))
                if len(components) == 3:
                    return (components["X"], components["Y"], components["Z"])
    if len(components) == 3:
        return (components["X"], components["Y"], components["Z"])
    return None


def parse_tcp_speed_globals(script_path: Path) -> dict[str, float]:
    """Extract TCP-speed global declarations from the script preamble.

    Returns a mapping ``{var_name: value_mps}`` covering every name in
    :data:`TCP_SPEED_GLOBAL_NAMES` that the script declares. Missing names
    are simply absent from the mapping (caller decides whether that's an
    error).
    """
    speeds: dict[str, float] = {}
    with script_path.open("r", encoding="utf-8") as f:
        for line in f:
            m = SPEED_GLOBAL_RE.match(line)
            if not m:
                continue
            name = m.group(1)
            if name not in TCP_SPEED_GLOBAL_NAMES:
                continue
            try:
                speeds[name] = float(m.group(2))
            except ValueError:
                continue
    return speeds


@dataclass(frozen=True)
class MotionSegment:
    """One ``movel`` / ``movej`` call with the velocity argument resolved.

    ``v_value`` is in m/s for ``movel`` (``v_unit == "m/s"``), rad/s for
    ``movej`` (``v_unit == "rad/s"``). When the ``v=`` expression could not
    be resolved against the symbol table, ``v_value`` is ``None`` and the
    caller must apply a fallback (typically the PolyScope cap).
    """

    lineno: int
    pose: tuple[float, ...]
    cycle_idx: int
    in_contact: bool
    kind: str       # "movel" or "movej"
    v_value: Optional[float]
    v_unit: str     # "m/s" or "rad/s"


def parse_speed_symbol_table(script_path: Path) -> dict[str, float]:
    """Extract every ``global NAME = scalar`` declaration from the preamble.

    Returns a mapping ``{NAME: float}`` covering every numeric scalar
    declared as a URScript global. Includes SPEED_FACTOR, ACCEL_FACTOR
    and every TCP-speed name (URSCRIPT_CONTACT_V, URSCRIPT_RECONTACT_V,
    V_CIRC, V_RECT, V_INIT, PROBE_DESCENT_V, ...). Pose literals
    (``global NOMINAL_FRAME = p[...]``) and joint arrays
    (``global Q_SAFE_JOINTS = [...]``) are filtered out by the
    ``SPEED_GLOBAL_RE`` (numeric scalar only).
    """
    symtab: dict[str, float] = {}
    with script_path.open("r", encoding="utf-8") as f:
        for line in f:
            m = SPEED_GLOBAL_RE.match(line)
            if not m:
                continue
            try:
                symtab[m.group(1)] = float(m.group(2))
            except ValueError:
                continue
    return symtab


def _eval_speed_expr(
    lhs: str,
    rhs: Optional[str],
    symtab: dict[str, float],
) -> Optional[float]:
    """Resolve a captured ``v=`` expression against the symbol table.

    Handles only the two forms emitted by ``design/export.py``:
    ``<NAME_or_literal>`` (single token) and ``<NAME_or_literal> *
    <NAME_or_literal>`` (binary product). Returns ``None`` when any token
    is neither a numeric literal nor a known global.
    """
    def _resolve(tok: str) -> Optional[float]:
        try:
            return float(tok)
        except ValueError:
            return symtab.get(tok)

    a = _resolve(lhs)
    if a is None:
        return None
    if rhs is None:
        return a
    b = _resolve(rhs)
    if b is None:
        return None
    return a * b


def parse_motion_segments(script_path: Path) -> list[MotionSegment]:
    """Walk every ``movel``/``movej`` line and resolve the v= argument.

    Same cycle / force_mode state machine as :func:`parse_poses` so each
    segment carries the enclosing ``cycle_idx`` and the ``in_contact``
    flag. The first ``p[x, y, z, rx, ry, rz]`` literal on the line is
    the target pose. ``movel(approach_pose, ...)`` lines without a pose
    literal (probe bodies) are skipped here exactly as they are in
    :func:`parse_poses`.
    """
    symtab = parse_speed_symbol_table(script_path)
    segments: list[MotionSegment] = []
    current_cycle = 0
    in_contact = False
    with script_path.open("r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, start=1):
            cycle_m = CYCLE_DEF_RE.match(line)
            if cycle_m:
                current_cycle = int(cycle_m.group(1))
                in_contact = False
                continue
            if FORCE_MODE_RE.match(line):
                in_contact = True
                continue
            if END_FORCE_RE.match(line):
                in_contact = False
                continue
            kind_m = MOVE_KIND_RE.match(line)
            if not kind_m:
                continue
            pose_m = ANY_POSE_RE.search(line)
            if not pose_m:
                continue
            kind = kind_m.group(1)
            v_unit = "m/s" if kind == "movel" else "rad/s"
            v_m = MOVE_V_RE.search(line)
            v_value: Optional[float] = None
            if v_m:
                v_value = _eval_speed_expr(v_m.group(1), v_m.group(2), symtab)
            pose = tuple(float(v) for v in pose_m.groups())
            segments.append(MotionSegment(
                lineno=lineno,
                pose=pose,
                cycle_idx=current_cycle,
                in_contact=in_contact,
                kind=kind,
                v_value=v_value,
                v_unit=v_unit,
            ))
    return segments


def transform(p_orig: SE3, p_anchor_old: SE3, p_ref: SE3) -> SE3:
    """Reproduce the URScript ``T`` wrapper used in ``etalement.script``.

    ``T(p) = pose_trans(P_REF, pose_trans(pose_inv(P_ANCHOR_OLD), p))``.
    With ``P_REF == P_ANCHOR_OLD`` the result is the identity transform.
    """
    return p_ref * p_anchor_old.inv() * p_orig
