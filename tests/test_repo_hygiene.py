"""
tests/test_repo_hygiene.py - Repo-hygiene guards for F5 (dead artifacts
tracked in git) and F9 (no end-of-line policy), see
docs/superpower/plans/erreur_hors_datalogger.md.

Both concerns read the git index (`git ls-files`, `git check-attr`) rather
than the working tree or a hand-parsed copy of `.gitattributes`: the index is
the state that actually ships, and asking git to resolve its own pattern
matching is the only way to be sure a rule "matches" in the sense git means
it. Every test in this module is skipped, not failed, when git is not on
PATH, so the rest of the suite still runs on a checkout without git.
"""

from __future__ import annotations

import re
import subprocess
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
UR5_SIM_DIR = REPO_ROOT / "ur5_sim"

# Extensions whose end-of-line policy is deliberately owned by a separate,
# already-scoped fix (F2 rewrites the export writer, F10 decides what the
# committed etalement.script should be). .gitattributes carries no rule for
# them on purpose, so they are the one documented exception to "every
# extension actually tracked is matched by some rule" below.
# F10 a tranche le cas de .script le 2026-08-16 : la regle est desormais
# `*.script text eol=lf`, parce que les tests de reproductibilite comparent une
# chaine generee au fichier suivi octet pour octet, et que core.autocrlf=true
# rendrait cette comparaison fausse sur un clone Windows neuf. Seul .urp reste
# en attente : il est retouche a la main par l'operateur, donc le renormaliser
# est une decision d'operateur, pas un effet de bord.
_EOL_POLICY_DEFERRED_EXTENSIONS = {".urp"}

# Comments that point at another file for context, in either language this
# repo mixes ("see ...", "cf. ...", "voir ..."), followed by something that
# reads as a relative path (must contain a slash and a dot, so a plain
# English word right after "see" is not mistaken for one).
_SEE_COMMENT_RE = re.compile(
    r"(?:\bsee\b|\bcf\.|\bvoir\b)\s+([\w./-]*/[\w./-]+\.[A-Za-z0-9]+)",
    re.IGNORECASE,
)


def _run_git(*args: str) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise unittest.SkipTest(f"git unavailable or failed: {exc}")
    return result.stdout


def _tracked_files() -> list[str]:
    return [line for line in _run_git("ls-files").splitlines() if line]


class DeadArtifactsRemovedTests(unittest.TestCase):
    """F5 - etalement.script.bak, ur5_etalementv6.py.original, tcp_live.json
    and validate_etalement.py must never come back as tracked files."""

    def setUp(self):
        self.tracked = _tracked_files()

    def test_no_bak_original_or_orig_file_is_tracked(self):
        offenders = [
            f for f in self.tracked
            if f.endswith((".bak", ".original", ".orig"))
        ]
        self.assertEqual(
            offenders, [],
            "a .bak/.original/.orig file is tracked again - these are dead "
            f"artifacts per F5, git history already holds them: {offenders}",
        )

    def test_tcp_live_json_and_directory_are_absent(self):
        offenders = [
            f for f in self.tracked
            if f == "tcp_live.json" or f.startswith("tcp_live/")
        ]
        self.assertEqual(
            offenders, [],
            "tcp_live.json / tcp_live/ are the retired file-IPC artifacts "
            f"(replaced by the UDP loopback IPC) and must stay deleted: {offenders}",
        )

    def test_validate_etalement_shim_is_gone(self):
        self.assertNotIn(
            "validate_etalement.py", self.tracked,
            "validate_etalement.py was deleted as a deprecated shim "
            "duplicating run_validate.py (F5 decision: delete, not keep)",
        )

    def test_see_comments_inside_ur5_sim_resolve_on_disk(self):
        """Catches both the fixed ik_multisolve.py:27 reference and any
        future comment that points at a path which does not exist."""
        broken: list[str] = []
        for py_file in sorted(UR5_SIM_DIR.rglob("*.py")):
            text = py_file.read_text(encoding="utf-8")
            for match in _SEE_COMMENT_RE.finditer(text):
                target = match.group(1)
                # Repo comments use two conventions: a path relative to the
                # repo root ("ur5_sim/probe.py", "design/export.py") and,
                # historically, one relative to the file itself
                # ("../../tcp_live"). Accept either.
                candidates = (REPO_ROOT / target, py_file.parent / target)
                if not any(c.exists() for c in candidates):
                    rel = py_file.relative_to(REPO_ROOT)
                    broken.append(f"{rel}: '{target}' resolves to neither "
                                   f"{candidates[0]} nor {candidates[1]}")
        self.assertEqual(
            broken, [],
            "a 'see/cf./voir' comment inside ur5_sim/ points at a path that "
            "does not exist on disk:\n" + "\n".join(broken),
        )


class GitattributesPolicyTests(unittest.TestCase):
    """F9 - line endings are a recorded, per-extension decision, not
    whatever core.autocrlf happens to be on the workstation that commits."""

    def setUp(self):
        self.gitattributes = REPO_ROOT / ".gitattributes"
        self.tracked = _tracked_files()

    def test_gitattributes_file_exists(self):
        self.assertTrue(
            self.gitattributes.is_file(),
            ".gitattributes is absent - line endings are decided per "
            "workstation with no recorded policy (F9)",
        )

    def test_bat_files_have_an_explicit_crlf_rule(self):
        bat_files = [f for f in self.tracked if f.endswith(".bat")]
        self.assertTrue(bat_files, "expected at least one tracked .bat file")
        values = self._check_attr("eol", bat_files)
        for path in bat_files:
            self.assertEqual(
                values.get(path), "crlf",
                f"{path}: expected eol=crlf (cmd.exe needs CRLF), "
                f"got {values.get(path)!r}",
            )

    def test_py_and_c_files_are_lf(self):
        text_files = [f for f in self.tracked if f.endswith((".py", ".c"))]
        self.assertTrue(text_files, "expected tracked .py/.c files")
        values = self._check_attr("eol", text_files)
        offenders = {p: v for p, v in values.items() if v != "lf"}
        self.assertEqual(
            offenders, {},
            f".py/.c files must resolve to eol=lf per F9: {offenders}",
        )

    def test_every_tracked_extension_is_matched_by_some_rule(self):
        """A new extension added later must be a deliberate .gitattributes
        decision, not a silent fallback to whatever git guesses.

        .script and .urp are the one documented exception: their policy is
        owned by F2/F10, not by this fix (see the module docstring).
        """
        by_extension: dict[str, list[str]] = {}
        for f in self.tracked:
            suffix = Path(f).suffix or f"/{Path(f).name}"
            by_extension.setdefault(suffix, []).append(f)

        unmatched: list[str] = []
        for suffix, files in by_extension.items():
            if suffix in _EOL_POLICY_DEFERRED_EXTENSIONS:
                continue
            sample = files[0]
            result = subprocess.run(
                ["git", "check-attr", "--all", "--", sample],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                check=True,
            )
            if not result.stdout.strip():
                unmatched.append(f"{suffix} (e.g. {sample})")

        self.assertEqual(
            unmatched, [],
            "these tracked extensions match no .gitattributes rule at all, "
            "so their line-ending policy silently falls back to git's "
            "per-workstation default instead of a recorded decision: "
            + ", ".join(unmatched),
        )

    def test_script_and_urp_deliberately_carry_no_rule(self):
        """Pin the deferral itself: if F2/F10 land a rule for these, this
        assertion is meant to break so the exception list above is updated
        in the same commit, not silently left stale."""
        script_or_urp = [
            f for f in self.tracked
            if Path(f).suffix in _EOL_POLICY_DEFERRED_EXTENSIONS
        ]
        self.assertTrue(script_or_urp, "expected tracked .script/.urp files")
        result = subprocess.run(
            ["git", "check-attr", "--all", "--", *script_or_urp],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertEqual(
            result.stdout.strip(), "",
            "a .gitattributes rule now matches .script/.urp; update "
            "_EOL_POLICY_DEFERRED_EXTENSIONS in this test if that was a "
            "deliberate F2/F10 change:\n" + result.stdout,
        )

    @staticmethod
    def _check_attr(attr: str, paths: list[str]) -> dict[str, str]:
        result = subprocess.run(
            ["git", "check-attr", attr, "--", *paths],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        pattern = re.compile(r"^(.*): " + re.escape(attr) + r": (.*)$")
        values: dict[str, str] = {}
        for line in result.stdout.splitlines():
            m = pattern.match(line)
            if m:
                values[m.group(1)] = m.group(2)
        return values


if __name__ == "__main__":
    unittest.main()
