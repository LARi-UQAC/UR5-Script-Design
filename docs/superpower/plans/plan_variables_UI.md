# Plan : interface de réglage des variables du protocole

> Créé le 29 juillet 2026. Complémentaire de
> [plan_optimisation_urscript.md](plan_optimisation_urscript.md) : celui-ci
> traite la génération du mouvement, celui-ci traite la saisie des valeurs par
> l'opérateur. Les deux se rejoignent sur la section 5 de l'autre plan (force Z
> exposée en global URScript) et sur sa section 6 (option de sondage).

## 0. Contexte pour une session à froid

Cette section existe pour qu'une session Claude Code démarrée sans historique
puisse exécuter le plan sans rien redécouvrir. Elle est volontairement redondante
avec `CLAUDE.md` et `ARCHITECTURE.md` sur les points qui font échouer une reprise.

### 0.1 Le projet en un paragraphe

Outillage autour du protocole d'étalement cosmétique ISO/COLIPA exécuté sur un
Universal Robots UR5 (contrôleur CB3, PolyScope 3.x, version robot rapportée par
l'opérateur : URSoftware 3.11.0.82155) équipé d'un capteur d'effort Robotiq
FT-300 et d'une pince 2F-85 qui sert de **support passif** à un doigt silicone
hémisphérique. Deux outils Python coopèrent : `design/` (UI matplotlib de réglage
des 6 cycles sur une plaque 50 x 50 mm, export `etalement.script` URScript et
`etalement.urp` XML PolyScope) et `ur5_sim/` (validateur hors ligne : parse le
`.script`, rejoue en cinématique inverse, visualise dans Swift).

Protocole : 3 cycles circulaires (boustrophédon porteur plus épicycloïde) puis
3 cycles rectilignes, contact régulé à 6 N en Z par `force_mode`.

### 0.2 Environnement d'exécution

- Windows, PowerShell principal, Bash disponible. Préfixer les commandes par
  `rtk` (wrapper d'économie de tokens, sûr en toute circonstance).
- Environnement virtuel local : `.venv` à la racine du dépôt.
- Tests : `unittest` de la bibliothèque standard. **`pytest` n'est pas installé**
  dans ce `.venv`.
- Langue de travail : français pour les documents de ce dépôt, code et
  commentaires en français ou anglais selon le fichier existant.
- Règles de style à respecter dans tout texte produit : pas de tiret cadratin,
  guillemets droits, pas de caractère d'ellipse, pas de caractères invisibles.

```bash
.\validate.bat                       # menu interactif Windows
python -m ur5_sim --check            # validation hors ligne, sans GUI
python -m ur5_sim --visualize        # Swift 3D plus panneaux matplotlib
python ur5_etalementv6.py            # UI de design
python ur5_etalementv6.py --export   # ecrit etalement.script
python ur5_etalementv6.py --export-urp  # ecrit etalement.urp (voir 0.6)
python -m unittest discover -s tests -p "test_*.py"
pip-audit -r requirements.txt
```

### 0.3 Carte du code

| Chemin | Rôle | Taille |
| :--- | :--- | :--- |
| `design/params.py` | source unique des constantes du protocole | 133 lignes |
| `design/trajectory.py` | générateurs de trajectoire, `get_waypoint_indices` | 223 |
| `design/export.py` | émission URScript et URP, `_build_urscript_lines` | 505 |
| `design/app.py` | UI matplotlib (6 sous-graphiques, 5 curseurs, 3 boutons) | 444 |
| `design/geometry.py` | `plate_to_robot`, `_abs_pose`, formatage des poses | 124 |
| `design/live_ipc.py` | réception UDP des trames du simulateur | 264 |
| `ur5_sim/parsing/urscript.py` | lecture du `.script`, extraction des poses | 403 |
| `ur5_sim/cli.py` | point d'entrée `--check` / `--visualize` | 380 |
| `ur5_sim/config.py` | constantes du simulateur, importées de `design.params` | 121 |
| `ur5_sim/probe.py` | rejeu du sondage 3 points, inerte | 209 |
| `ur5_etalementv6.py` | shim de 2 lignes vers `design.app.main` | 3 |

Documents : `CLAUDE.md` (résumé opérationnel), `ARCHITECTURE.md` (invariants,
frames, contraintes), `plan_optimisation_urscript.md` (génération du mouvement),
ce fichier (saisie des valeurs).

### 0.4 Invariants à ne pas casser

**Chaîne de repères.** Trois repères coexistent : plaque (mm, +Z sortant),
base robot (m, via `plate_to_robot` qui applique
`ROBOT_BASE_ROTATION_DEG = 225` puis les origines), monde absolu (m, via
`_abs_pose`). Toute nouvelle géométrie doit parcourir **exactement la même
chaîne** que les waypoints, sinon elle atterrit dans un autre repère. Recette de
référence : `ur5_sim/visualization/surface._plate_corner_world`.

**Contraintes CB3 / PolyScope 3.x.** Pas de `movel` ni `stopl` dans un thread
secondaire. Pas de tranche de liste (`[0:3]`). Pas d'indexation chaînée
`a[i][j]`. Pas de `for`, uniquement `while`. Budget mémoire vérifié par
`_validate_script_memory` (200 000 octets). Vitesses plafonnées à 0.250 m/s par
`_clamp_tcp_speed`. Le programme généré **n'actionne jamais** la pince : aucun
`rq_*`, aucun `set_payload`, aucun RS485 outil.

**Sémantique de `force_mode(task_frame, selection_vector, wrench, type, limits)`.**
Source d'erreur classique : le vecteur `limits` n'est pas en newtons et ses
unités changent par axe. Pour un axe **compliant** (sélection à 1), la valeur est
une **vitesse TCP maximale** (m/s ou rad/s). Pour un axe **non compliant**
(sélection à 0), c'est un **écart de position maximal toléré** entre la pose
réelle et la pose commandée (m ou rad), au-delà duquel le contrôleur déclenche
« Force mode: Maximum position deviation exceeded ». Dans ce projet, la sélection
est `[0, 0, 1, 0, 0, 0]` : seul Z est compliant, donc `FORCE_LIMIT_Z = 0.040` est
une vitesse en m/s tandis que `FORCE_LIMIT_XY = 0.008` est une distance en
mètres. Seul l'argument `wrench` est en newtons.

**Plancher du capteur.** FT-300 postérieur à octobre 2017 : bruit de signal sur
Fz de 0.1 N (écart type sur 1 s), seuil minimal recommandé de 1 N une fois monté
sur le robot, plage +/- 300 N, flux de données à 100 Hz. Sur CB3, `force_mode`
régule sur l'estimation interne du contrôleur (courants articulaires) **sauf** si
le URCap FT 300 est installé et l'option d'utilisation du signal du capteur en
force mode activée. Vérification : `zero_ftsensor()` puis enregistrement de
`get_tcp_force()` pendant 10 s sans contact ; écart type proche de 0.1 N =
lecture du capteur, de l'ordre du newton = estimation interne. Références :
[manuel FT 300](https://assets.robotiq.com/website-assets/support_documents/document/online/FT_Sensor_Instruction_Manual_Web_20190322.zip/FT_Sensor_Instruction_Manual_Web/Content/Specifications.htm),
[manuel URScript](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-scriptmanual/G5/force_mode_task_frame_selection_vector.htm).

**Sondage de surface.** Actif : sondage Z 1 point `probe_surface_z()`
(`design/export.py:244-274`), qui descend en `speedl` jusqu'à
`PROBE_FORCE_THR = 4.0` N et écrit la composante Z de `MEAS_FRAME`. Parqué :
sondage 3 points (`probe_one`, `probe_surface_plane`), conservé inerte, incorrect
car figé en Z. Conséquence à connaître : `MEAS_FRAME` porte la hauteur mesurée
mais garde l'orientation nominale, donc les 6 N sont perpendiculaires au plan
**nominal**, pas au plan réel si la plaque est inclinée. `NHAT` n'existe plus.

**Liaison par valeur des constantes.** `from design.params import X` copie la
valeur à l'import : modifier `design.params.X` ensuite ne change rien pour
`design/export.py` ni `ur5_sim/config.py`. C'est l'obstacle central de ce plan,
détaillé en section 2.

### 0.5 État des travaux, au 29 juillet 2026

Rien des deux plans n'est implémenté. Le code est propre et fonctionnel dans son
mode actuel.

| Sujet | État |
| :--- | :--- |
| Descente au contact sous force active | fait, dans `design/export.py:394-408` |
| Blend sous `force_mode` | ouvert, `r=URSCRIPT_BLEND` sur 642 `movel` |
| Force Z en global URScript | à faire, plan_optimisation section 5, phase 1 |
| Mode paramétrique cycles 1-3 | à faire, plan_optimisation sections 3 à 10 |
| Interface de réglage des variables | à faire, ce plan |
| Option de sondage `PROBE_MODE` | amélioration ultérieure, code conservé inerte |

`etalement.script` courant : 818 lignes, 113 131 octets (57 % du budget), 661
`movel`, 203 par cycle circulaire, 17 à 18 par cycle rectiligne, 6 paires
`force_mode` / `end_force_mode`, `FORCE_LIMIT_XY = 0.008`.

### 0.6 Pièges concrets, à lire avant d'agir

- **`etalement.urp` a été modifié à la main par l'opérateur pour des essais
  robot.** Il porte `FORCE_LIMIT_XY = 0.002` et les anciens globaux du sondage 3
  points (`NHAT_*`, `NOMINAL_P*`, `PROBE_TILT_MAX_RAD`). Ce n'est **pas** une
  incohérence à corriger d'office. Ne pas lancer `--export-urp` sans accord
  explicite : la commande écrase le fichier sans avertissement.
- `FORCE_LIMIT_XY` est passée de 0.002 à 0.008 m dans `design/params.py` après un
  arrêt de protection réel sur le robot (le doigt silicone traîne latéralement et
  écarte le TCP du chemin commandé). Ne pas revenir à 0.002.
- Le sondage 3 points ne doit être ni supprimé, ni réactivé, ni réécrit. Il
  attend un rework décrit dans l'autre plan, section 6.
- Deux chemins d'export produisent des densités différentes : l'UI exporte tous
  les points du curseur, le mode headless sous-échantillonne à
  `URSCRIPT_N_WAYPOINTS_CIRCULAR = 80`. Le simulateur ne valide donc pas
  forcément ce que l'opérateur livre au robot.
- Dépôt git : branche `main`, remote `LARi-UQAC/UR5-Script-Design`. Le prénom de
  l'étudiante associée au projet ne doit apparaître dans aucun artefact public
  (issues, README, messages de commit).
- `requirements.txt` contient des invariants de dépendances à ne pas relâcher
  (`swift-sim==1.1.0` impose `websockets<13`, correctifs locaux de
  `roboticstoolbox-python==1.1.1` sur Python 3.13).

### 0.7 Comment reprendre

1. Lire `ARCHITECTURE.md` sections 3 à 6, puis `design/params.py` en entier.
2. Lire la section 2 de ce plan (obstacle des imports par valeur) avant d'écrire
   la moindre ligne.
3. Exécuter `python -m ur5_sim --check` et
   `python -m unittest discover -s tests -p "test_*.py"` pour établir la
   référence avant modification.
4. Lire la sous-section 7.0 : elle fixe qui exécute quoi, le budget, et ce qui
   est volontairement laissé de côté.
5. Suivre les phases de la section 7.1, dans l'ordre, avec la sélection réduite
   de compétences Superpowers de la sous-section 7.2. La phase 2 a un critère
   d'acceptation strict : à réglages par défaut, le `.script` généré doit être
   identique octet pour octet à la référence.
6. Mettre à jour `README.md`, `ARCHITECTURE.md` et `CLAUDE.md` à la phase 8 :
   aucun livrable n'est complet sans cette mise à jour.

---

## 1. Objectif et périmètre

L'UI actuelle (`design/app.py`) règle la **forme du chemin** : nombre de points
de discrétisation, rayon des cercles, nombre de cycles, de passes, de tours.
Tout le reste du protocole est figé dans le code et ne peut être changé qu'en
éditant `design/params.py` puis en réexportant, ou en modifiant à la main le
fichier généré (ce qui est perdu au prochain export).

Objectif : une **seconde interface**, dédiée aux valeurs numériques du
protocole, permettant à l'opérateur de modifier avant export la force Z, les
limites de `force_mode`, le mode de sondage, les vitesses, les accélérations,
les hauteurs, avec les valeurs actuelles comme valeurs par défaut codées en dur
et restaurables en un clic.

Hors périmètre : la forme du chemin (déjà couverte par les curseurs existants,
à unifier seulement), la génération paramétrique du mouvement (autre plan), la
remise en service du sondage 3 points (autre plan, section 6).

---

## 2. Obstacle technique à lever en premier

Ce point conditionne toute la suite : sans lui, une interface qui modifie
`design/params.py` en mémoire n'aura **aucun effet** sur le script exporté.

`design/export.py:33-65` importe les constantes **par valeur** :

```python
from design.params import (
    FORCE_CONTACT_DEPTH,
    FORCE_LIMIT_ROT,
    FORCE_LIMIT_XY,
    ...
)
```

En Python, `from X import Y` copie la référence au moment de l'import. Modifier
ensuite `design.params.FORCE_LIMIT_XY` ne change pas le `FORCE_LIMIT_XY` déjà
lié dans `design/export.py`. Le code actuel connaît déjà le problème et le
contourne au cas par cas :

- `design/trajectory.py:82-85` : `import design.params as _P` **dans** la
  fonction, avec le commentaire « Read at call time so that app.py slider
  patches to design.params take effect » ;
- `design/app.py:185-196` : `_build_circular_with_params` écrit dans
  `_P.CIRC_R_CIRCLE`, appelle, puis restaure les anciennes valeurs dans un
  `finally`.

Ce contournement ne monte pas à l'échelle de trente champs. Il faut une couche
de réglages lue à l'exécution.

`ur5_sim/config.py:13-20` fait le même import par valeur depuis `design.params`
(`FORCE_Z_TARGET`, `FORCE_CONTACT_DEPTH`, `URSCRIPT_MAX_TCP_SPEED`,
`PROBE_TILT_MAX_RAD`, `P_REF`, `TCP_Z`). Si l'opérateur change une valeur dans
l'UI, le simulateur continuera de valider avec les valeurs par défaut : il
faut que les deux processus lisent la même source.

---

## 3. Architecture proposée

Trois niveaux, du plus stable au plus volatil :

| Niveau | Rôle | Fichier |
| :--- | :--- | :--- |
| Défauts codés en dur | valeurs de référence du protocole, sous git, jamais écrites par l'UI | `design/params.py` (inchangé) |
| Réglages effectifs | objet lu à l'exécution par l'exporteur, l'UI et le simulateur | `design/settings.py` (nouveau) |
| Surcharges persistées | ce que l'opérateur a changé, uniquement les écarts | `etalement_settings.json` (nouveau, non versionné) |

### 3.1 `design/settings.py`

```python
@dataclass
class Settings:
    force_z_target: float = params.FORCE_Z_TARGET
    force_limit_xy: float = params.FORCE_LIMIT_XY
    ...
    probe_mode: str = 'z1'

    def to_overrides(self) -> dict:      # uniquement les champs != defaut
    @classmethod
    def from_file(cls, path) -> Settings # defauts + surcharges du JSON
    def save(self, path) -> None
    def reset(self) -> None              # retour aux defauts params.py
    def validate(self) -> list[str]      # messages d'erreur, vide si tout est bon
```

Accès global par `get_settings()` (singleton simple), rechargeable.

Règle d'écriture pour tout le code appelant : **jamais**
`from design.settings import force_z_target`, toujours
`s = get_settings()` puis `s.force_z_target` au moment de l'usage.

### 3.2 Adaptation de `design/export.py`

Remplacer le bloc d'import par valeur par une lecture au début de
`_build_urscript_lines` :

```python
def _build_urscript_lines(cycles, settings=None):
    s = settings or get_settings()
    ...
```

et substituer chaque constante par `s.<champ>`. Le nombre d'occurrences est
modéré (les constantes force, probe, vitesses et Z apparaissent une à quatre
fois chacune). Les chemins (`SCRIPT_PATH`, `URP_PATH`) et les listes de sécurité
restent dans `params.py`.

### 3.3 Adaptation de `ur5_sim/config.py`

Lire `etalement_settings.json` s'il existe, sinon les défauts. Le fichier étant
au niveau du dépôt, aucun couplage nouveau n'est créé : `ur5_sim` dépend déjà de
`design.params`. Ajouter au rapport `--check` une ligne indiquant les valeurs
actives et si elles proviennent du JSON ou des défauts.

---

## 4. Inventaire des variables à exposer

Quatre groupes, plus un groupe verrouillé. Les valeurs par défaut sont celles de
`design/params.py` au 29 juillet 2026.

### 4.1 Groupe « Force » (onglet 1)

| Champ | Défaut | Unité | Bornes proposées | Effet |
| :--- | :--- | :--- | :--- | :--- |
| `FORCE_Z_TARGET` | 6.0 | N | 2.0 à 20.0 | consigne de force ; devient un global URScript éditable aussi sur le pendant |
| `FORCE_LIMIT_XY` | 0.008 | m (déviation max, axes non compliants) | 0.002 à 0.020 | au-delà, arrêt de protection « Maximum position deviation exceeded » |
| `FORCE_LIMIT_Z` | 0.040 | m/s (vitesse max, axe compliant) | 0.005 à 0.100 | vitesse de la compliance Z |
| `FORCE_LIMIT_ROT` | 0.35 | rad (déviation max) | 0.05 à 0.60 | 0.35 rad = 20 deg |
| `FORCE_CONTACT_DEPTH` | 0.005 | m | 0.001 à 0.015 | profondeur visée sous le plan nominal à la descente de recontact |

Rappels à afficher dans l'IHM à côté du champ de force :

- cible protocole 6.0 +/- 0.5 N ;
- plancher matériel : sur un FT 300 postérieur à octobre 2017, bruit Fz de 0.1 N
  (écart type sur 1 s) et seuil minimal recommandé de 1 N monté sur robot ; une
  consigne sous 2 N n'est pas régulable de façon stable ;
- `FORCE_LIMIT_Z` est une **vitesse**, pas une distance, contrairement à ses
  voisines dans le même vecteur.

### 4.2 Groupe « Sondage » (onglet 2)

| Champ | Défaut | Unité | Bornes | Note |
| :--- | :--- | :--- | :--- | :--- |
| `PROBE_MODE` | `z1` | choix | `z1` / `plane3` | `plane3` grisé, non sélectionnable (voir plan_optimisation_urscript.md section 6) |
| `PROBE_FORCE_THR` | 4.0 | N | 1.5 à 10.0 | seuil de détection du contact, norme des 3 composantes de force |
| `PROBE_DESCENT_V` | 0.004 | m/s | 0.001 à 0.020 | vitesse de descente du sondage |
| `PROBE_ACCEL` | 0.05 | m/s^2 | 0.01 à 0.50 | |
| `PROBE_MAX_TRAVEL` | 0.15 | m | 0.02 à 0.30 | course max avant échec, sécurité anti-collision |
| `PROBE_APPROACH_MM` | 30.0 | mm | 5 à 100 | utilisé par le mode `plane3` seulement, grisé en `z1` |
| `PROBE_TILT_MAX_RAD` | 0.0873 | rad | 0.01 à 0.30 | idem, grisé en `z1` |
| `PROBE_RETRY_MAX` | 1 | entier | 0 à 3 | idem |
| `PROBE_POINTS_PLATE_MM` | (5,5) (45,5) (25,45) | mm | dans la plaque | idem, saisie à trois lignes |

Les champs propres à `plane3` restent visibles mais désactivés : ils documentent
ce que la future option demandera, sans laisser croire qu'elle est disponible.

### 4.3 Groupe « Mouvement et URScript » (onglet 3)

| Champ | Défaut | Unité | Bornes | Note |
| :--- | :--- | :--- | :--- | :--- |
| `URSCRIPT_ACCEL` | 0.8 | m/s^2 | 0.1 à 2.0 | |
| `URSCRIPT_TRANSIT_V` | 0.3 | m/s | 0.02 à 0.25 après clamp | plafonné par `URSCRIPT_MAX_TCP_SPEED`, avertissement affiché |
| `URSCRIPT_CONTACT_V` | 0.05 | m/s | 0.005 à 0.25 | |
| `URSCRIPT_RECONTACT_V` | 0.01 | m/s | 0.002 à 0.05 | descente au contact |
| `URSCRIPT_BLEND` | 0.002 | m | 0.0 à 0.010 | blend hors contact |
| `URSCRIPT_BLEND_CONTACT` | 0.0005 | m | 0.0 à 0.005 | blend sous `force_mode`, introduit par l'autre plan |
| `URSCRIPT_N_WAYPOINTS_CIRCULAR` | 80 | entier | 20 à 2000 | à unifier avec le curseur de discrétisation, voir 6.3 |
| `CIRC_SPEED` | 36.0 | mm/s | 5 à 250 | |
| `LIN_SPEED` | 80.0 | mm/s | 5 à 250 | |
| `URSCRIPT_MAX_TCP_SPEED` | 0.250 | m/s | lecture seule | limite PolyScope, ne pas rendre modifiable |
| `URSCRIPT_MAX_BYTES` | 200000 | octets | lecture seule | budget mémoire du contrôleur |

### 4.4 Groupe « Surface et hauteurs » (onglet 4)

| Champ | Défaut | Unité | Bornes | Note |
| :--- | :--- | :--- | :--- | :--- |
| `SURFACE_W` | 50.0 | mm | 10 à 200 | change la géométrie, recalcul du chemin |
| `SURFACE_H` | 50.0 | mm | 10 à 200 | idem |
| `MARGIN` | 4.0 | mm | 0 à 20 | |
| `Z_TRANSIT` | 10.0 | mm | 2 à 50 | hauteur de dégagement entre cycles |
| `Z_RETREAT_END` | 30.0 | mm | 5 à 100 | retrait final |
| `CIRC_Y_START` | 5.0 | mm | 0 à 25 | |
| `CIRC_DURATION`, `LIN_DURATION_ODD`, `LIN_DURATION_EVEN` | 11.0 / 7.5 / 6.0 | s | 1 à 60 | durées cibles du protocole, informatives pour l'affichage |

### 4.5 Groupe « Calibration robot » (onglet 5, verrouillé)

Ces valeurs positionnent la plaque dans le repère du robot. Une erreur ici
envoie l'outil ailleurs que sur la plaque. Elles restent **affichées en lecture
seule**, l'édition étant conditionnée à une case à cocher explicite
« Déverrouiller la calibration (modifie l'ancrage robot) » et à une confirmation.

`ROBOT_X_ORIGIN`, `ROBOT_Y_ORIGIN`, `ROBOT_Z_SURFACE`, `ROBOT_RX/RY/RZ`,
`ROBOT_BASE_ROTATION_DEG`, `P_REF`, `TCP_FT300_Z`, `TCP_COUPLING_Z`,
`TCP_GRIPPER_Z`, `TCP_FINGER_Z`, `SAFE_APPROACH_RADIUS_M`, `Q_SAFE_JOINTS_RAD`.

`TCP_Z` reste **calculé** (somme des quatre longueurs) et affiché en lecture
seule : ne jamais le rendre saisissable indépendamment de ses composantes.

---

## 5. Choix de la technologie d'interface

| Option | Description | Pour | Contre |
| :--- | :--- | :--- | :--- |
| A - widgets matplotlib | `TextBox` et `RadioButtons` dans une seconde figure | aucune dépendance nouvelle, cohérent avec l'existant | ingérable au-delà d'une dizaine de champs : pas d'onglets, pas de défilement, placement manuel de chaque axe, validation pauvre |
| B - fenêtre Tkinter | `Toplevel` avec onglets `ttk.Notebook`, un `Entry` par champ | stdlib, vrais widgets de formulaire, onglets, défilement, validation par champ, boutons OK / Annuler | code IHM à écrire, cohabitation avec la boucle matplotlib à soigner |
| C - fichier JSON édité à la main | l'opérateur édite `etalement_settings.json` dans VS Code | trivial à implémenter, versionnable, revue facile | ce n'est pas une interface ; pas de bornes, pas d'unités, pas de garde-fou |
| D - page web locale servie par `http.server` | formulaire HTML sur `127.0.0.1`, ouvert dans le navigateur | testable par Playwright, précédent dans le projet avec le viewer Swift | deux processus et un port de plus à gérer pour une simple saisie de valeurs, l'opérateur quitte la fenêtre matplotlib pour un onglet ; écarté, voir 5.1 |

Recommandation : **B, avec la persistance de C**. Le backend matplotlib par
défaut sous Windows est TkAgg, donc une racine Tk existe déjà quand la figure
est ouverte : la fenêtre de réglages est un `Toplevel` de cette racine, sans
seconde `mainloop`, sans dépendance ajoutée. Le JSON reste le format de
sauvegarde, éditable à la main pour un cas pressé.

### 5.1 Option D et Playwright, écartées

Une validation de l'IHM par Playwright a été envisagée le 15 août 2026, puis
abandonnée le même jour. Playwright ne pilote que des navigateurs (Chromium,
Firefox, WebKit) : il n'existe aucun pilote Tk. Le rendre applicable aurait
imposé l'option D, c'est-à-dire remplacer la fenêtre Tk par une page web servie
en local, donc un serveur, un port et un onglet de navigateur pour saisir une
trentaine de nombres. Le rapport coût-bénéfice ne le justifie pas pour une
interface de saisie qui n'est pas un système critique.

La validation de l'IHM se fait donc par `unittest`, en instanciant la fenêtre
sans la montrer et en pilotant les widgets par leur API (`insert`, `get`,
appel direct des callbacks). Cela couvre ce qui compte ici, la **capture des
valeurs** et la propagation vers `Settings`, sans couvrir le rendu visuel, qui
reste vérifié à l'oeil par l'opérateur. Voir `tests/test_ui_settings.py` dans la
table des tests.

Note pour une session future : ne pas reproposer Playwright pour cette fenêtre.
La question a été tranchée.

### 5.2 Maquette

```text
+----------------------------------------------------------------------+
|  Parametres du protocole d'etalement                           [ X ]  |
+----------------------------------------------------------------------+
| [ Force ] [ Sondage ] [ Mouvement ] [ Surface ] [ Calibration (v) ]   |
+----------------------------------------------------------------------+
|                                                                      |
|  Force cible Z          [   6.0    ] N       defaut 6.0   [2.0-20.0] |
|  Deviation max XY       [   0.008  ] m       defaut 0.008 [.002-.02] |
|  Vitesse max Z          [   0.040  ] m/s     defaut 0.040 [.005-0.1] |
|  Deviation max rotation [   0.35   ] rad     defaut 0.35  [.05-0.6 ] |
|  Profondeur de contact  [   0.005  ] m       defaut 0.005 [.001-.015]|
|                                                                      |
|  Note : cible protocole 6.0 +/- 0.5 N. Sous 2 N, la regulation n'est |
|  pas stable avec le FT-300 (bruit 0.1 N, seuil recommande 1 N).      |
|                                                                      |
+----------------------------------------------------------------------+
|  Etat : 2 valeurs differentes des defauts                            |
|  [ Reinitialiser ]  [ Appliquer ]  [ Enregistrer ]  [ Exporter ... ] |
+----------------------------------------------------------------------+
```

Chaque ligne porte quatre informations : le libellé, la valeur saisissable,
l'unité, et le défaut codé en dur avec les bornes. L'opérateur voit donc en
permanence de combien il s'écarte de la référence.

### 5.3 Comportement des boutons

- **Appliquer** : valide tous les champs, met à jour l'objet `Settings` en
  mémoire, rafraîchit la figure de trajectoire si un champ géométrique a changé.
  En cas d'erreur, aucun champ n'est appliqué et les lignes fautives sont
  signalées.
- **Réinitialiser** : recharge les défauts de `design/params.py`, onglet courant
  ou tout, au choix.
- **Enregistrer** : écrit `etalement_settings.json` avec les seules surcharges.
- **Exporter** : déclenche `generate_urscript` et `generate_urp` avec les
  réglages courants, et affiche le compte rendu (nombre de lignes, mémoire,
  clamps appliqués).

---

## 6. Points de conception à trancher explicitement

### 6.1 Validation et clamps

Chaque champ porte un type, des bornes et une unité, déclarés dans une table au
même endroit que le dataclass. Deux niveaux :

- **refus** si hors bornes dures (par exemple force négative, `PROBE_MAX_TRAVEL`
  nul) : l'export est bloqué ;
- **avertissement plus clamp** pour ce que le contrôleur plafonne de toute façon
  (vitesses TCP, `_clamp_tcp_speed` existant, `design/export.py:69-78`) : la
  valeur est corrigée et l'IHM affiche ce qui a été appliqué.

Ajouter `_clamp_force_target()` sur le même modèle, avec `FORCE_Z_MIN_N = 2.0`
et `FORCE_Z_MAX_N = 20.0`.

### 6.2 Traçabilité des réglages utilisés

Exigence de protocole expérimental : savoir avec quelles valeurs un essai a été
produit. Le script généré porte déjà un en-tête ; y ajouter un bloc :

```urscript
# === REGLAGES UTILISES ===
# genere le 2026-07-29 14:22, source : etalement_settings.json
# valeurs modifiees par rapport aux defauts :
#   FORCE_Z_TARGET   6.0 -> 8.0 N
#   PROBE_FORCE_THR  4.0 -> 2.5 N
# empreinte des reglages : 3f9a1c7
```

L'empreinte est un condensé court des surcharges ; elle permet de rapprocher un
fichier `.script` ou `.urp` d'un jeu de réglages archivé.

### 6.3 Unification avec les curseurs existants

Les cinq curseurs de `design/app.py:149-182` (discrétisation, `CIRC_R_CIRCLE`,
`N_CIRCULAR_CYCLES`, `CIRC_N_PASSES`, `CIRC_N_CIRCLES`) doivent devenir des vues
sur le même objet `Settings`, sinon deux sources de vérité coexistent. Le
mécanisme de sauvegarde et restauration de `_build_circular_with_params`
disparaît alors.

Cette unification règle au passage la divergence signalée dans l'autre plan
(section 2.6) : l'export UI utilise tous les points du curseur alors que
l'export headless utilise `URSCRIPT_N_WAYPOINTS_CIRCULAR = 80`. Décider une
règle unique et l'exposer comme un champ, par exemple « densité des waypoints
circulaires » avec deux modes, `tous les points du tracé` ou `sous-échantillonner
à N`.

### 6.4 Articulation avec la force devenue globale URScript

Une fois la section 5 de l'autre plan livrée, la force existe à deux endroits :
dans l'UI avant export, et comme `global FORCE_Z_TARGET` éditable sur le
pendant. Règle proposée, à écrire dans l'en-tête du script :

- la valeur saisie dans l'UI est la valeur **initiale** du programme ;
- une modification sur le pendant vaut pour la session en cours et n'est pas
  répercutée dans le JSON ;
- au prochain export, la valeur de l'UI reprend la main.

### 6.5 Cas des fichiers modifiés à la main

Le `.urp` courant a été ajusté manuellement pour des essais robot. Une fois
cette interface en place, ce n'est plus nécessaire, et cela redevient risqué :
`generate_urp` écrit sur `URP_PATH` sans avertissement
(`design/export.py:500-503`). Deux mesures :

- avertir avant écrasement si le fichier existant diffère du dernier fichier
  généré (comparaison d'empreinte enregistrée à l'export précédent) ;
- proposer dans l'UI un champ « nom du fichier de sortie » pour produire des
  variantes d'essai (`etalement_test.script`) sans toucher au fichier de
  référence.

---

## 7. Étapes d'implémentation

### 7.0 Cadre d'exécution, arrêté le 15 août 2026

**Livraison en une seule session.** Les huit phases sont exécutées dans la même
session Claude Code, sans reprise ultérieure. C'est ce qui dicte les arbitrages
ci-dessous.

**Répartition modèle par phase.**

| Phase | Exécutant | Motif |
| :--- | :--- | :--- |
| 1 - Couche de réglages | Opus, en session, sans sous-agent | fonde le contrat que lisent toutes les autres phases |
| 2 - Exporteur | Opus, en session, sans sous-agent | lève l'obstacle de la section 2 sous une contrainte d'identité octet pour octet ; une trentaine de constantes à tracer dans un fichier de 505 lignes |
| 3 - Simulateur | Opus, en session, sans sous-agent | second consommateur des réglages, même piège d'import par valeur |
| 4 - Fenêtre de réglages | sous-agent Sonnet | code IHM mécanique, engendré depuis la table de métadonnées, périmètre clos |
| 5 - Intégration | Opus, en session, sans sous-agent | supprime `_build_circular_with_params` et unifie cinq curseurs avec `Settings` ; état partagé, terrain à bogues subtils |
| 6 - Traçabilité | Opus, en session, sans sous-agent | touche l'en-tête émis, donc l'invariant d'identité de la phase 2 |
| 7 - Persistance | sous-agent Sonnet | `.gitignore`, fichier d'exemple, chargement au démarrage, bannière |
| 8 - Documentation | sous-agent Sonnet | mise à jour de trois documents selon la table 7.3 |

Les trois sous-agents Sonnet sont lancés une fois les phases 1 à 3 terminées,
c'est-à-dire une fois le contrat `Settings` figé. La phase 4 dépend de la table
de métadonnées, la phase 8 dépend du code livré : la 8 part en dernier.

**Aucun sous-agent d'exploration.** Pas de sous-agent chargé de chercher des
solutions de rechange ni de contester la conception. Ce document est la
conception retenue ; sa remise en cause n'est pas au programme de cette session.
Un désaccord constaté pendant l'exécution se signale en une phrase et se tranche
avec l'opérateur, il ne déclenche pas une étude parallèle.

**Budget de jetons contraint, et proportionné à l'enjeu.** Il s'agit d'une
interface de saisie de valeurs, pas d'un système critique. Conséquences
assumées :

- discipline de test différenciée : le test d'abord (`test-driven-development`)
  s'applique aux phases 1, 2 et 3, là où une régression est silencieuse et
  coûteuse ; les phases 4, 7 et 8 sont testées après coup, ce qui suffit pour du
  code d'IHM et de la documentation ;
- quatre fichiers de tests au lieu de sept, par regroupement, sans perte de
  couverture (voir la table des tests) ;
- pas de revue croisée multi-modèles, pas de délibération, pas de recherche
  bibliographique ;
- les valeurs de sécurité gardent en revanche leur garde-fou complet : la
  lecture seule sur `URSCRIPT_MAX_TCP_SPEED` et `URSCRIPT_MAX_BYTES`, le
  verrouillage de l'onglet calibration et le refus hors bornes ne sont pas des
  variables d'ajustement du budget.

**Exigence d'IHM rappelée.** L'interface doit permettre de saisir la valeur
souhaitée **et** afficher en permanence la valeur par défaut correspondante,
avec son unité et ses bornes, sur la même ligne. C'est la maquette 5.2, et c'est
un critère de recette, pas une suggestion de présentation.

### 7.1 Les huit phases

**Phase 1 - Couche de réglages** (Opus). `design/settings.py` : dataclass, table
des métadonnées (unité, bornes, libellé, groupe), `from_file`, `save`, `reset`,
`validate`, `get_settings`. Aucun changement de comportement à ce stade.

**Phase 2 - Exporteur** (Opus). `_build_urscript_lines(cycles, settings=None)`
lit `get_settings()` ; suppression des imports par valeur dans
`design/export.py`. Test de non-régression : à réglages par défaut, le `.script`
généré est strictement identique à l'actuel, octet pour octet.

**Phase 3 - Simulateur** (Opus). `ur5_sim/config.py` lit le même fichier de
réglages ; `--check` affiche la source et les écarts aux défauts.

**Phase 4 - Fenêtre de réglages** (sous-agent Sonnet). `design/ui_settings.py` :
`Toplevel` Tk, `ttk.Notebook` à cinq onglets, génération des lignes à partir de
la table de métadonnées (pas de code répété par champ), validation, boutons.
Chaque ligne affiche libellé, champ de saisie, unité, défaut et bornes. Livre
aussi `tests/test_ui_settings.py`.

**Phase 5 - Intégration** (Opus). Bouton « Paramètres » dans la barre de
`design/app.py`, unification des cinq curseurs existants avec `Settings`,
option de nom de fichier de sortie, avertissement d'écrasement.

**Phase 6 - Traçabilité** (Opus). Bloc « réglages utilisés » et empreinte dans
l'en-tête du script et du `.urp`.

**Phase 7 - Persistance** (sous-agent Sonnet). `etalement_settings.json` ajouté
au `.gitignore`, `etalement_settings.example.json` versionné, chargement au
démarrage avec bannière listant les écarts aux défauts.

**Phase 8 - Documentation** (sous-agent Sonnet ; obligatoire, pas optionnelle) :
mettre `README.md`, `ARCHITECTURE.md` et `CLAUDE.md` au niveau du code livré,
détail en 7.3. Une phase n'est terminée que lorsque la documentation
correspondante est à jour ; sinon la prochaine session à froid repart sur une
carte du code fausse.

### 7.2 Exécution avec le plugin Superpowers

Sélection réduite au strict nécessaire, conformément au budget arrêté en 7.0.
Chaque compétence est invoquée avec l'outil `Skill` et annoncée avant usage.

| Moment | Compétence | Usage ici |
| :--- | :--- | :--- |
| Au démarrage | `superpowers:executing-plans` | dérouler les huit phases avec un point de contrôle par phase |
| Phases 1, 2, 3 | `superpowers:test-driven-development` | test en échec d'abord ; l'identité octet pour octet de la phase 2 et la régression sur les imports par valeur sont les deux cas d'école. Non appliqué aux phases 4, 7 et 8 |
| Sur tout écart de comportement | `superpowers:systematic-debugging` | notamment si le script généré change alors que les réglages sont aux défauts |
| Avant toute annonce d'achèvement | `superpowers:verification-before-completion` | aucune affirmation de succès sans sortie de commande à l'appui |
| Fin de branche | `superpowers:finishing-a-development-branch` | décision d'intégration, fusion validée par l'humain |

Écartées pour cette session, et pourquoi : `using-git-worktrees` (une simple
branche `feat/settings-ui` suffit, les trois sous-agents écrivent dans des
fichiers disjoints, voir 7.2.1) ; `subagent-driven-development` et
`dispatching-parallel-agents` (la répartition est déjà fixée en 7.0, inutile de
la redériver) ; `requesting-code-review` et `receiving-code-review` (revue
croisée coûteuse, sans commune mesure avec l'enjeu d'une fenêtre de saisie) ;
`brainstorming` (ce document en est déjà le produit, ne le relancer que si le
périmètre change).

#### 7.2.1 Propriété des fichiers, pour éviter les écritures concurrentes

Les sous-agents Sonnet travaillent dans le même arbre de travail. Chaque fichier
a donc un propriétaire unique par phase.

| Fichier | Propriétaire |
| :--- | :--- |
| `design/settings.py` | phase 1 (Opus), puis phase 7 pour la seule fonction de chargement au démarrage |
| `design/export.py` | phases 2 et 6 (Opus) |
| `ur5_sim/config.py`, `ur5_sim/cli.py` | phase 3 (Opus) |
| `design/ui_settings.py`, `tests/test_ui_settings.py` | phase 4 (Sonnet) |
| `design/app.py` | phase 5 (Opus) **exclusivement** |
| `.gitignore`, `etalement_settings.example.json` | phase 7 (Sonnet) |
| `README.md`, `ARCHITECTURE.md`, `CLAUDE.md` | phase 8 (Sonnet) |

Point de friction identifié : la bannière d'écarts au démarrage relève de la
phase 7 par son contenu mais de `design/app.py` par son emplacement. Le
sous-agent de la phase 7 livre la fonction qui calcule la bannière et **ne
touche pas** à `design/app.py` ; c'est la phase 5 qui pose l'appel.

### 7.3 Documentation à mettre à jour

| Fichier | Section | Mise à jour attendue |
| :--- | :--- | :--- |
| `README.md` | « What is in this repository » | mentionner l'interface de réglage des variables et le fichier `etalement_settings.json` |
| `README.md` | « Starting the software » | bouton « Paramètres » de l'UI, drapeau CLI éventuel, emplacement du fichier de réglages et de son exemple |
| `README.md` | nouvelle sous-section « Settings » | ce qui est réglable, ce qui est verrouillé (limites de sécurité, calibration), et comment revenir aux défauts |
| `ARCHITECTURE.md` | section 2, table `design/` | lignes `settings.py` et `ui_settings.py`, et mention que `export.py` lit les réglages à l'exécution au lieu d'importer les constantes par valeur |
| `ARCHITECTURE.md` | section 2, table `ur5_sim/` | `config.py` lit le fichier de réglages quand il existe |
| `ARCHITECTURE.md` | section 8 ou nouvelle section | invariant « ne jamais faire `from design.params import X` dans un module qui doit voir les réglages utilisateur » |
| `ARCHITECTURE.md` | section 9, tests | nouveaux fichiers de tests de la table ci-dessous |
| `ARCHITECTURE.md` | section 10, règles pour la suite | remplacer le prochain chantier par ce qui reste ouvert |
| `CLAUDE.md` | « Common commands », « High-level architecture » | mêmes ajouts, format condensé, sans dupliquer `ARCHITECTURE.md` |

Vérifier que les liens des documents modifiés résolvent et que les numéros de
ligne cités dans ce plan sont corrigés s'ils ont bougé.

### 7.4 Tests

Quatre fichiers au lieu des sept d'abord envisagés. Le regroupement suit la
frontière des modules, pas celle des cas de test : la couverture est identique,
seul le nombre de fichiers et d'entêtes baisse.

| Test | Phase | Objet |
| :--- | :--- | :--- |
| `tests/test_settings.py` | 1 | trois classes. `RoundTrip` : `save` puis `from_file` restitue les mêmes valeurs, et seules les surcharges sont écrites. `Defaults` : chaque défaut du dataclass est égal à la constante de `design/params.py` correspondante. `Validation` : valeurs hors bornes refusées, clamps appliqués et signalés |
| `tests/test_export_settings.py` | 2, 6 | deux classes. `Identity` : à réglages par défaut, le `.script` généré est identique octet pour octet à la référence courante. `UsesSettings` : régression sur l'obstacle de la section 2, modifier `Settings` change réellement le `.script` généré. Le bloc de traçabilité de la phase 6 est neutralisé pendant la comparaison d'identité, sa date et son empreinte variant par construction |
| `tests/test_sim_reads_settings.py` | 3 | `ur5_sim/config` reflète le JSON quand il existe, les défauts sinon |
| `tests/test_ui_settings.py` | 4 | **capture des valeurs**, sans Playwright et sans afficher la fenêtre. Instancie la fenêtre sur une racine Tk non mappée (`withdraw()`), écrit dans les widgets par `insert`, appelle les callbacks des boutons, puis vérifie l'objet `Settings`. Cas couverts : une valeur valide se propage ; une valeur hors bornes est refusée sans qu'aucun champ ne soit appliqué ; le défaut affiché à côté de chaque champ correspond bien à `design/params.py` ; « Réinitialiser » ramène tous les champs aux défauts ; les champs en lecture seule et l'onglet calibration verrouillé refusent l'édition. Encadré par `unittest.skipUnless` sur la disponibilité de Tk |

Exécution : `python -m unittest discover -s tests -p "test_*.py"`.

Un test manque volontairement : le **rendu visuel** de la fenêtre. Il est
vérifié à l'oeil par l'opérateur au point 9 de la vérification. Automatiser un
contrôle de rendu Tk coûterait plus que ce que vaut le risque ici.

---

## 8. Risques et points de vigilance

- **Le champ qui ne sert à rien.** Exposer une variable que l'exporteur
  n'utilise pas donne une fausse impression de contrôle. La table de
  métadonnées doit être vérifiée par un test qui confirme que chaque champ
  exposé apparaît bien dans le script généré ou dans le calcul de trajectoire.
  Ce test appartient à la classe `UsesSettings` de
  `tests/test_export_settings.py` : il boucle sur la table plutôt que
  d'énumérer les champs à la main, sinon il vieillit mal.
- **Les valeurs de sécurité.** `URSCRIPT_MAX_TCP_SPEED` et `URSCRIPT_MAX_BYTES`
  sont des limites du contrôleur, pas des préférences : lecture seule.
- **La calibration.** Onglet verrouillé, confirmation explicite, et jamais de
  valeurs de site poussées dans git.
- **Le sondage `plane3`.** Reste grisé tant que les cinq conditions de la
  section 6.5 de l'autre plan ne sont pas remplies. Un menu déroulant qui
  propose une option cassée est pire que pas de menu.
- **Deux processus, un fichier.** Si le simulateur tourne pendant qu'on
  enregistre de nouveaux réglages, il travaille encore avec les anciens.
  Afficher la date de lecture des réglages dans le rapport `--check`.
- **Cohérence physique.** Rien n'empêche de saisir une force de 20 N avec une
  profondeur de contact de 1 mm. Un contrôle croisé minimal
  (`FORCE_CONTACT_DEPTH` compatible avec `FORCE_LIMIT_Z` et la vitesse de
  recontact) évite les combinaisons manifestement dangereuses.
- **Le contrat `Settings` figé trop tard.** Les trois sous-agents de 7.0
  dépendent tous de la table de métadonnées. La lancer avant la fin de la phase
  1 produirait trois implémentations d'un contrat encore mouvant, donc trois
  reprises. Le point de contrôle de fin de phase 3 est le verrou : rien ne part
  avant.
- **Écriture concurrente sur `design/app.py`.** Deux phases ont une raison
  légitime d'y toucher, la 5 et la 7. La table 7.2.1 tranche : seule la phase 5
  y écrit. Ne pas la contourner sous prétexte que la modification est d'une
  ligne.
- **La livraison en une session.** Contrainte posée en 7.0. Si le temps manque,
  couper par la fin : les phases 6, 7 et 8 sont les moins couplées au reste et
  se reportent le mieux. Ne jamais couper dans les phases 1 à 3, qui laisseraient
  le dépôt à moitié converti, avec un exporteur qui lit les réglages et un
  simulateur qui lit encore les défauts.

---

## 9. Vérification

1. À réglages par défaut : `python ur5_etalementv6.py --export --no-show` produit
   un `.script` identique à la référence, et `python -m ur5_sim --check` donne le
   même verdict qu'avant la refonte.
2. Modifier la force à 8 N dans l'UI, exporter, vérifier que
   `global FORCE_Z_TARGET = 8.0` apparaît une fois et que le bloc « réglages
   utilisés » liste l'écart.
3. Saisir une valeur hors bornes : l'export est refusé, le message nomme le
   champ et les bornes.
4. Enregistrer, fermer, rouvrir : les surcharges sont rechargées et l'IHM
   signale les écarts aux défauts.
5. « Réinitialiser » : tous les champs reviennent aux valeurs de
   `design/params.py`.
6. `python -m ur5_sim --check` reflète les réglages du JSON, pas les défauts.
7. Tests unitaires au vert, les quatre fichiers de la table 7.4 compris.
8. `README.md`, `ARCHITECTURE.md` et `CLAUDE.md` décrivent le code livré
   (table 7.3), et les liens résolvent.
9. Contrôle visuel par l'opérateur, seul point non automatisé : ouvrir la
   fenêtre, parcourir les cinq onglets, et vérifier que chaque ligne porte bien
   ses quatre informations, libellé, champ de saisie, unité, défaut avec bornes.
   Vérifier aussi que l'onglet calibration s'ouvre verrouillé et que
   `URSCRIPT_MAX_TCP_SPEED`, `URSCRIPT_MAX_BYTES` et `TCP_Z` sont affichés sans
   être saisissables.

---

## 10. Journal de révision

| Date | Modification |
| :--- | :--- |
| 29 juillet 2026 | Création. Périmètre : interface de saisie des variables du protocole, distincte de l'interface de tracé existante. Obstacle des imports par valeur identifié comme préalable. Inventaire en cinq groupes, onglet calibration verrouillé, Tkinter retenu avec persistance JSON, articulation avec la force devenue globale URScript et avec l'option de sondage |
| 29 juillet 2026 (suite) | Section 0 de contexte pour reprise à froid. Phase 8 « Documentation » séparée de la persistance, avec la table 7.2, devenue 7.3 (`README.md`, `ARCHITECTURE.md`, `CLAUDE.md`) et le critère de vérification associé. Sous-section 7.1, devenue 7.2 : exécution pilotée par le plugin Superpowers |
| 15 août 2026 | Cadre d'exécution arrêté (nouvelle sous-section 7.0) : livraison en une seule session ; phases 1, 2, 3, 5, 6 exécutées par Opus en session sans sous-agent, phases 4, 7, 8 déléguées à trois sous-agents Sonnet lancés après le gel du contrat `Settings` ; aucun sous-agent d'exploration ou de contestation de la conception ; budget de jetons contraint et proportionné, l'interface n'étant pas un système critique. Nouvelle table 7.2.1 de propriété des fichiers, `design/app.py` réservé à la phase 5. Renumérotation : 7.1 devient 7.2, 7.2 devient 7.3, les tests deviennent 7.4 |
| 15 août 2026 (suite) | Validation par Playwright envisagée puis écartée le jour même : Playwright ne pilote que des navigateurs, l'appliquer aurait imposé de remplacer la fenêtre Tk par une page web locale (option D, ajoutée à la table de la section 5 pour mémoire). Tkinter conservé. La capture des valeurs est vérifiée par `tests/test_ui_settings.py`, en `unittest`, sur une racine Tk non mappée. Motif consigné en 5.1 pour éviter que la question soit reposée. Tests regroupés de sept fichiers à quatre, à couverture égale (table 7.4). Trois risques ajoutés en section 8 : contrat figé trop tard, écriture concurrente sur `design/app.py`, ordre de coupe si le temps manque |
