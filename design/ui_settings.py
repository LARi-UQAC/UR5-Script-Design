"""
design/ui_settings.py - Fenetre de reglages du protocole d'etalement
(phase 4 du plan docs/superpower/plans/plan_variables_UI.md).

Coquille du contrat fige pour la phase 5 (design/app.py) : la signature de
open_settings_window et les attributs/methodes publics de SettingsWindow ne
doivent pas changer sans mettre a jour la phase 5 en meme temps. La
construction des onglets Tk (ttk.Notebook, une ligne par
design.settings_spec.FieldSpec, aucune construction repetee par champ), les
callbacks des boutons, et le coeur d'apply() vivent dans
design/ui_widgets.py, pour que ce fichier reste sous le plafond de 4096
jetons (.claude/rules/code-style.md).

Tk peut etre absent de l'environnement : open_settings_window() rend alors
None et imprime un message, sans jamais lever.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

try:
    import tkinter as tk
    from tkinter import ttk
    _TK_AVAILABLE = True
except ImportError:  # pragma: no cover - environnement sans Tk
    tk = None
    ttk = None
    _TK_AVAILABLE = False

from design.settings import Settings, get_settings
from design.settings_spec import GROUP_LABELS, GROUP_ORDER, SPECS, spec_by_name, \
    specs_for_group
from design.ui_widgets import (DEFAULT_EXPORT_STEM, build_bottom_bar,
                                build_group_tab, capture_working_settings,
                                ensure_error_styles, field_widget_state,
                                format_value_text, get_widget_text,
                                mark_widget_error, set_widget_state,
                                set_widget_text)


def open_settings_window(
    settings: Optional[Settings] = None,
    on_apply: Optional[Callable[[Settings], None]] = None,
    on_export: Optional[Callable[[Settings, str], None]] = None,
    master: Optional[Any] = None,
) -> Optional["SettingsWindow"]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Ouvre la fenetre de reglages du protocole. Point d'entree que la
        phase 5 (design/app.py) appelle depuis le bouton "Parametres".

    Inputs:
        settings (Settings | None): reglages a editer ; get_settings() si
            None (singleton rechargeable de design/settings.py).
        on_apply (Callable[[Settings], None] | None): rappel invoque apres un
            Appliquer valide.
        on_export (Callable[[Settings, str], None] | None): rappel du bouton
            Exporter, avec le nom de fichier de sortie sans extension.
        master (tk widget | None): racine ou fenetre parente. None reutilise
            la racine Tk implicite deja ouverte par matplotlib (backend
            TkAgg en cree une), ou en cree une si aucune n'existe.

    Outputs:
        window (SettingsWindow | None): instance ouverte, ou None si Tk est
            indisponible dans cet environnement (message imprime, aucune
            exception levee).
    --------------------------------------------------------------------------
    """
    if not _TK_AVAILABLE:
        print("Tk indisponible : fenetre de reglages non ouverte.")
        return None
    if master is None:
        master = tk._default_root or tk.Tk()
    if settings is None:
        settings = get_settings()
    return SettingsWindow(master, settings, on_apply=on_apply,
                           on_export=on_export)


class SettingsWindow:
    """Toplevel Tk : cinq onglets engendres depuis design.settings_spec."""

    def __init__(self, master: Any, settings: Settings,
                 on_apply: Optional[Callable[[Settings], None]] = None,
                 on_export: Optional[Callable[[Settings, str], None]] = None):
        self.settings = settings
        self._on_apply = on_apply
        self._on_export = on_export
        self.widgets: dict[str, Any] = {}
        self._default_labels: dict[str, Any] = {}

        self.top = tk.Toplevel(master)
        self.top.title("Parametres du protocole d'etalement")
        ensure_error_styles(self.top)
        self._unlock_var = tk.BooleanVar(
            master=self.top, value=self.settings.calibration_unlocked)

        self.notebook = ttk.Notebook(self.top)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=6)
        for group in GROUP_ORDER:
            frame = ttk.Frame(self.notebook)
            frame.columnconfigure(1, weight=1)
            self.notebook.add(frame, text=GROUP_LABELS[group])
            build_group_tab(self, frame, group)

        build_bottom_bar(self)
        self._refresh_status()

    # -- API de capture, contrat fige pour la phase 5 --------------------

    def read_field(self, name: str) -> str:
        """Texte courant du widget de saisie `name`."""
        return get_widget_text(self.widgets[name])

    def set_field(self, name: str, value: Any) -> None:
        """Ecrit `value` dans le widget `name`. Une chaine est ecrite telle
        quelle (utile pour injecter volontairement un texte invalide en
        test) ; toute autre valeur est d'abord formatee selon spec.kind."""
        spec = spec_by_name(name)
        text = value if isinstance(value, str) else format_value_text(spec, value)
        set_widget_text(self.widgets[name], text)

    def set_unlocked(self, value: bool) -> None:
        """
        --------------------------------------------------------------------
        Purpose:
            Bascule le verrou de l'onglet Calibration sans dialogue de
            confirmation : c'est l'API programmatique, appelee par la case a
            cocher une fois deja confirmee (voir ui_widgets.prompt_unlock_
            toggle) et directement par les tests, qui ne doivent jamais
            declencher de boite de dialogue Tk bloquante en environnement
            headless.

        Inputs:
            value (bool): True deverrouille l'onglet, False le reverrouille.
        --------------------------------------------------------------------
        """
        self.settings.calibration_unlocked = value
        self._unlock_var.set(value)
        for spec in specs_for_group("calibration"):
            set_widget_state(self.widgets[spec.name],
                              field_widget_state(spec, value))

    def apply(self) -> bool:
        """
        --------------------------------------------------------------------
        Purpose:
            Lit tous les champs modifiables, convertit, et valide sur une
            copie de Settings (design.ui_widgets.capture_working_settings).
            Application tout ou rien : la moindre erreur laisse
            self.settings entierement intact.

        Outputs:
            ok (bool): True si applique (self.settings mis a jour champ par
                champ, on_apply appele). False si des erreurs ont ete
                trouvees : elles s'affichent dans status_var et les widgets
                fautifs recoivent le style d'erreur.
        --------------------------------------------------------------------
        """
        working, errors = capture_working_settings(self)

        for spec in SPECS:
            is_faulty = any(e.startswith(f"{spec.name} :") for e in errors)
            mark_widget_error(self.widgets[spec.name], is_faulty)

        if errors:
            self.status_var.set(" | ".join(errors))
            return False

        for spec in SPECS:
            setattr(self.settings, spec.name, getattr(working, spec.name))
        self.settings.calibration_unlocked = working.calibration_unlocked

        self._refresh_status(
            extra=self.settings.clamps() + self.settings.warnings())
        if self._on_apply is not None:
            self._on_apply(self.settings)
        return True

    def reset(self, group: Optional[str] = None) -> None:
        """settings.reset(group) puis recharge les widgets touches ; les
        widgets verrouilles reevaluent leur etat au passage."""
        self.settings.reset(group)
        specs = SPECS if group is None else specs_for_group(group)
        for spec in specs:
            self.set_field(spec.name, getattr(self.settings, spec.name))
            mark_widget_error(self.widgets[spec.name], False)
            set_widget_state(self.widgets[spec.name], field_widget_state(
                spec, self.settings.calibration_unlocked))
        self._refresh_status()

    def save(self) -> None:
        """apply() d'abord ; si valide, ecrit SETTINGS_PATH."""
        if self.apply():
            self.settings.save()

    def export(self) -> None:
        """apply() d'abord ; si valide, appelle on_export(settings, stem)
        avec le nom saisi dans la barre du bas (defaut : "etalement")."""
        if not self.apply():
            return
        if self._on_export is not None:
            stem = self.export_stem_var.get().strip() or DEFAULT_EXPORT_STEM
            self._on_export(self.settings, stem)

    def default_text(self, name: str) -> str:
        """Texte 'defaut ... [lo-hi]' actuellement affiche pour le champ
        `name` (utilise par le test de non-regression sur design/params.py)."""
        return self._default_labels[name].cget("text")

    def _refresh_status(self, extra: Optional[list[str]] = None) -> None:
        """Recalcule la ligne d'etat depuis len(settings.to_overrides())."""
        n = len(self.settings.to_overrides())
        text = f"Etat : {n} valeur(s) differente(s) des defauts"
        if extra:
            text += " | " + " | ".join(extra)
        self.status_var.set(text)
