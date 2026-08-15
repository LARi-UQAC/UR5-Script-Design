"""
design/ui_widgets.py - Moteur IHM de la fenetre de reglages : construction
des widgets, conversion texte/valeur, et le gros de la logique de
design/ui_settings.py (phase 4 du plan
docs/superpower/plans/plan_variables_UI.md).

design/ui_settings.py reste la coquille du contrat fige pour la phase 5
(open_settings_window, classe SettingsWindow) ; ce module porte tout ce qui
peut s'exprimer sans dependre de cette classe precise, pour que les deux
fichiers restent sous le plafond de 4096 jetons
(.claude/rules/code-style.md). Les fonctions qui prennent `window` en
parametre agissent sur une SettingsWindow par duck typing (widgets, settings,
notebook, top) sans l'importer : aucun cycle d'import entre les deux fichiers.

Trois etats Tk suffisent pour toute la fenetre : 'normal' (saisissable),
'readonly' (visible, non saisissable : combobox de choix, et tout champ
kind in (points, vector) qui reste un texte affiche seulement) et 'disabled'
(grise : editable=False, enabled=False, ou onglet calibration verrouille).
"""

from __future__ import annotations

import copy
from typing import Any

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
    _TK_AVAILABLE = True
except ImportError:  # pragma: no cover - environnement sans Tk
    tk = None
    messagebox = None
    ttk = None
    _TK_AVAILABLE = False

import design.params as params
from design.settings_spec import (FieldSpec, GROUP_LABELS, GROUP_ORDER, SPECS,
                                   specs_for_group)

ERROR_STYLE_ENTRY = "Error.TEntry"
ERROR_STYLE_COMBOBOX = "Error.TCombobox"
DEFAULT_EXPORT_STEM = "etalement"


# --- Styles et conversion texte/valeur --------------------------------------

def ensure_error_styles(widget: Any) -> None:
    """Declare les styles ttk de signalement d'erreur (fond rouge pale) sur
    l'interpreteur Tk de `widget`. Idempotent, a rappeler sans risque."""
    style = ttk.Style(widget)
    style.configure(ERROR_STYLE_ENTRY, fieldbackground="#f8d7da")
    style.configure(ERROR_STYLE_COMBOBOX, fieldbackground="#f8d7da")


def is_combobox(widget: Any) -> bool:
    """True si `widget` est le ttk.Combobox d'un champ kind == choice."""
    return _TK_AVAILABLE and isinstance(widget, ttk.Combobox)


def format_value_text(spec: FieldSpec, value: Any) -> str:
    """Convertit une valeur Settings en texte pour un widget de saisie."""
    if spec.kind in ("points", "vector"):
        return repr(value)
    if spec.kind == "int":
        return str(int(value))
    if spec.kind == "float":
        return f"{float(value):g}"
    return str(value)


def parse_value_text(spec: FieldSpec, text: str) -> Any:
    """
    --------------------------------------------------------------------------
    Purpose:
        Convertit le texte lu dans un widget vers le type que Settings
        attend pour ce champ. N'est jamais appele pour kind in
        (points, vector) : ces champs restent en lecture seule dans l'IHM,
        capture_working_settings ne les lit donc jamais au moment d'Appliquer.

    Inputs:
        spec (FieldSpec): metadonnee du champ.
        text (str): texte brut lu dans le widget.

    Outputs:
        value (Any): int, float ou str selon spec.kind.

    Raises:
        ValueError: texte non convertible dans le type attendu ; l'appelant
            transforme cela en message d'erreur nomme par champ.
    --------------------------------------------------------------------------
    """
    text = text.strip()
    if spec.kind == "int":
        return int(text)
    if spec.kind == "float":
        return float(text)
    return text


def default_bounds_text(spec: FieldSpec, default: Any) -> str:
    """Texte 'defaut <valeur> [<lo>-<hi>]' affiche a droite de chaque ligne
    (quatrieme information de la maquette 5.2 du plan)."""
    text = f"defaut {format_value_text(spec, default)}"
    if spec.lo is not None and spec.hi is not None:
        # Separateur " a " et non "-" : une borne basse negative donnait
        # "[-2-2]", illisible sur l'onglet calibration.
        text += f" [{spec.lo:g} a {spec.hi:g}]"
    elif spec.kind == "choice":
        # Les choix indisponibles restent visibles ici, marques, pour dire ce
        # qui existera sans laisser croire qu'on peut le selectionner.
        shown = [c if c not in spec.disabled_choices else f"{c} (indisponible)"
                 for c in spec.choices]
        text += f" {{{', '.join(shown)}}}"
    return text


# --- Widgets bas niveau ------------------------------------------------------

def build_field_widget(parent: Any, spec: FieldSpec) -> Any:
    """Cree le widget de saisie adapte a spec.kind, non peuple : Combobox en
    lecture seule pour un choix, Entry pour tout le reste."""
    if spec.kind == "choice":
        # Seuls les choix reellement utilisables sont proposes. Le libelle de
        # droite documente les autres (plan, section 8).
        usable = [c for c in spec.choices if c not in spec.disabled_choices]
        return ttk.Combobox(parent, values=usable,
                             state="readonly", width=18)
    return ttk.Entry(parent, width=18)


def set_widget_state(widget: Any, state: str) -> None:
    """Applique un etat Tk ('normal', 'readonly' ou 'disabled') a un widget
    de saisie, Entry ou Combobox indifferemment : les deux l'acceptent."""
    widget.configure(state=state)


def set_widget_text(widget: Any, text: str) -> None:
    """Ecrit `text` dans un widget de saisie quel que soit son etat courant
    (un Entry desactive ou en lecture seule refuse insert/delete, donc on
    repasse temporairement a 'normal' le temps de l'ecriture)."""
    previous_state = str(widget["state"])
    if previous_state in ("disabled", "readonly"):
        widget.configure(state="normal")
    if is_combobox(widget):
        widget.set(text)
    else:
        widget.delete(0, tk.END)
        widget.insert(0, text)
    if previous_state in ("disabled", "readonly"):
        widget.configure(state=previous_state)


def get_widget_text(widget: Any) -> str:
    """Lit le texte courant d'un widget de saisie."""
    return widget.get()


def mark_widget_error(widget: Any, is_error: bool) -> None:
    """Applique ou retire le fond rouge pale d'un champ signale fautif par
    apply(). N'affecte jamais l'etat 'state' du widget, seulement son style."""
    if is_combobox(widget):
        widget.configure(style=ERROR_STYLE_COMBOBOX if is_error else "TCombobox")
    else:
        widget.configure(style=ERROR_STYLE_ENTRY if is_error else "TEntry")


# --- Etats derives des drapeaux de FieldSpec --------------------------------

def field_enabled(spec: FieldSpec, calibration_unlocked: bool) -> bool:
    """Le champ peut-il etre modifie en ce moment, tous drapeaux confondus
    (editable, enabled, verrouillage calibration) ?"""
    if not spec.editable or not spec.enabled:
        return False
    if spec.locked and not calibration_unlocked:
        return False
    return True


def field_widget_state(spec: FieldSpec, calibration_unlocked: bool) -> str:
    """Etat Tk du widget : 'disabled' si field_enabled est faux, 'readonly'
    pour un choix ou un champ kind points/vector (jamais saisissable, meme
    deverrouille), 'normal' sinon."""
    if not field_enabled(spec, calibration_unlocked):
        return "disabled"
    if spec.kind in ("choice", "points", "vector"):
        return "readonly"
    return "normal"


def field_writable(spec: FieldSpec, calibration_unlocked: bool) -> bool:
    """apply() doit-il lire et convertir ce champ ? Faux pour points et
    vector, qui restent des textes affiches, jamais repris a l'ecriture."""
    return (field_enabled(spec, calibration_unlocked)
            and spec.kind not in ("points", "vector"))


# --- Construction des onglets et de la barre du bas -------------------------

def build_field_row(window: Any, parent: Any, row: int, spec: FieldSpec) -> int:
    """Une ligne : libelle, champ de saisie, unite, defaut + bornes, et la
    note sous la ligne quand spec.note n'est pas vide. Peuple
    window.widgets et window._default_labels au passage."""
    ttk.Label(parent, text=spec.label).grid(
        row=row, column=0, sticky="w", padx=4, pady=2)

    widget = build_field_widget(parent, spec)
    widget.grid(row=row, column=1, sticky="we", padx=4, pady=2)
    window.widgets[spec.name] = widget
    set_widget_text(widget, format_value_text(
        spec, getattr(window.settings, spec.name)))
    set_widget_state(widget, field_widget_state(
        spec, window.settings.calibration_unlocked))

    ttk.Label(parent, text=spec.unit).grid(
        row=row, column=2, sticky="w", padx=4, pady=2)

    default = getattr(params, spec.const)
    default_label = ttk.Label(parent, text=default_bounds_text(spec, default))
    default_label.grid(row=row, column=3, sticky="w", padx=4, pady=2)
    window._default_labels[spec.name] = default_label
    row += 1

    if spec.note:
        note = ttk.Label(parent, text=spec.note, wraplength=560,
                          foreground="#555555", justify="left")
        note.grid(row=row, column=0, columnspan=4, sticky="w",
                  padx=4, pady=(0, 6))
        row += 1
    return row


def build_group_tab(window: Any, parent: Any, group: str) -> None:
    """Engendre toutes les lignes d'un onglet depuis specs_for_group ; ajoute
    la case de deverrouillage en tete de l'onglet Calibration."""
    row = 0
    if group == "calibration":
        checkbox = ttk.Checkbutton(
            parent,
            text="Deverrouiller la calibration (modifie l'ancrage robot)",
            variable=window._unlock_var,
            command=lambda: prompt_unlock_toggle(window))
        checkbox.grid(row=row, column=0, columnspan=4, sticky="w",
                      padx=4, pady=(4, 8))
        row += 1
    for spec in specs_for_group(group):
        row = build_field_row(window, parent, row, spec)


def build_bottom_bar(window: Any) -> None:
    """Ligne d'etat, nom de fichier de sortie, puis les quatre boutons dans
    l'ordre impose : Reinitialiser, Appliquer, Enregistrer, Exporter."""
    window.status_var = tk.StringVar(master=window.top)
    ttk.Label(window.top, textvariable=window.status_var, anchor="w").pack(
        fill="x", padx=6)

    export_row = ttk.Frame(window.top)
    export_row.pack(fill="x", padx=6, pady=(2, 0))
    ttk.Label(export_row, text="Nom du fichier de sortie").pack(side="left")
    window.export_stem_var = tk.StringVar(master=window.top,
                                           value=DEFAULT_EXPORT_STEM)
    ttk.Entry(export_row, textvariable=window.export_stem_var,
              width=24).pack(side="left", padx=(4, 0))

    buttons = ttk.Frame(window.top)
    buttons.pack(fill="x", padx=6, pady=6)
    ttk.Button(buttons, text="Reinitialiser",
               command=lambda: prompt_reset_scope(window)).pack(side="left")
    ttk.Button(buttons, text="Appliquer", command=window.apply).pack(
        side="left", padx=(6, 0))
    ttk.Button(buttons, text="Enregistrer", command=window.save).pack(
        side="left", padx=(6, 0))
    ttk.Button(buttons, text="Exporter", command=window.export).pack(
        side="left", padx=(6, 0))


# --- Callbacks interactifs (dialogues de confirmation) ----------------------

def prompt_reset_scope(window: Any) -> None:
    """Callback du bouton Reinitialiser : demande tout ou seulement l'onglet
    courant, les deux chemins existent (section 5.3 du plan). N'est jamais
    exerce par les tests, qui appellent window.reset(group) directement."""
    current = GROUP_ORDER[window.notebook.index(window.notebook.select())]
    reset_all = messagebox.askyesno(
        "Reinitialiser",
        "Reinitialiser TOUS les onglets aux defauts ?\n"
        "Non : reinitialise seulement l'onglet courant "
        f"({GROUP_LABELS[current]}).")
    window.reset(None if reset_all else current)


def prompt_unlock_toggle(window: Any) -> None:
    """Callback de la case Calibration : confirmation avant deverrouillage.
    window.set_unlocked() reste l'API sans dialogue, utilisee ici une fois
    confirmee, et directement par les tests (jamais de boite bloquante en
    environnement headless)."""
    if window._unlock_var.get():
        confirmed = messagebox.askokcancel(
            "Deverrouiller la calibration",
            "Ceci autorise la modification de l'ancrage robot (origines, "
            "rotations, P_REF). Continuer ?")
        if confirmed:
            window.set_unlocked(True)
        else:
            window._unlock_var.set(False)
    else:
        window.set_unlocked(False)


# --- Coeur d'apply() ---------------------------------------------------------

def capture_working_settings(window: Any) -> tuple[Any, list[str]]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Lit tous les champs modifiables de la fenetre, convertit, et valide
        sur une copie de window.settings. Coeur d'apply(), isole ici pour
        garder design/ui_settings.py sous le plafond de jetons.

    Inputs:
        window (SettingsWindow): fenetre source (widgets, settings courant).

    Outputs:
        working (Settings): copie de window.settings, champs modifiables mis
            a jour depuis les widgets.
        errors (list[str]): messages d'erreur ; liste vide si tout est valide.
    --------------------------------------------------------------------------
    """
    working = copy.deepcopy(window.settings)
    unlocked = window.settings.calibration_unlocked
    errors: list[str] = []
    for spec in SPECS:
        if not field_writable(spec, unlocked):
            continue
        text = get_widget_text(window.widgets[spec.name])
        try:
            value = parse_value_text(spec, text)
        except (TypeError, ValueError):
            errors.append(f"{spec.name} : '{text}' n'est pas une valeur "
                           f"valide ({spec.label}).")
            continue
        setattr(working, spec.name, value)
    errors.extend(working.validate())
    return working, errors
