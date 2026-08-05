# Plan d'Optimisation : Génération URScript par Fonctions Embarquées

> **Révision du 29 juillet 2026** - plan réécrit sur l'état réel du code après le
> découpage de `ur5_etalementv6.py` en package `design/`. La version initiale
> (mai 2026) visait un fichier monolithique et des numéros de ligne qui n'existent
> plus. Les correctifs déjà appliqués sont marqués FAIT, les points ouverts
> OUVERT, et deux conclusions de la version initiale sont corrigées (sections 2.4
> et 3.4).

## 0. Contexte pour une session à froid

Section autonome : une session Claude Code démarrée sans historique doit pouvoir
exécuter ce plan en ne lisant que ce fichier et les fichiers qu'il cite. Le même
contexte figure dans [plan_variables_UI.md](plan_variables_UI.md) section 0 ; les
deux plans sont indépendants l'un de l'autre.

### 0.1 Le projet en un paragraphe

Outillage autour du protocole d'étalement cosmétique ISO/COLIPA exécuté sur un
Universal Robots UR5 (contrôleur CB3, PolyScope 3.11.0.82155) équipé d'un capteur
d'effort Robotiq FT-300 et d'une pince 2F-85 qui sert de **support passif** à un
doigt silicone hémisphérique. Deux outils Python coopèrent : `design/` (UI
matplotlib de réglage des 6 cycles sur une plaque 50 x 50 mm, export
`etalement.script` URScript et `etalement.urp` XML PolyScope) et `ur5_sim/`
(validateur hors ligne : parse le `.script`, rejoue en cinématique inverse,
visualise dans Swift). Protocole : 3 cycles circulaires (boustrophédon porteur
plus épicycloïde) puis 3 cycles rectilignes, contact régulé à 6 N en Z par
`force_mode`.

### 0.2 Environnement d'exécution

- Windows, PowerShell principal, Bash disponible. Préfixer les commandes par
  `rtk`.
- Environnement virtuel local : `.venv` à la racine du dépôt.
- Tests : `unittest` de la bibliothèque standard. **`pytest` n'est pas installé.**
- Documents de ce dépôt en français. Style : pas de tiret cadratin, guillemets
  droits, pas de caractère d'ellipse, pas de caractères invisibles.

```bash
.\validate.bat                          # menu interactif Windows
python -m ur5_sim --check               # validation hors ligne, sans GUI
python -m ur5_sim --visualize           # Swift 3D plus panneaux matplotlib
python ur5_etalementv6.py               # UI de design
python ur5_etalementv6.py --export      # ecrit etalement.script
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

Documents : `README.md` (point d'entrée pratique), `ARCHITECTURE.md`
(invariants, repères, contraintes), `CLAUDE.md` (résumé opérationnel),
`plan_variables_UI.md` (interface de saisie des variables).

### 0.4 Invariants à ne pas casser

**Chaîne de repères.** Trois repères coexistent : plaque (mm, +Z sortant), base
robot (m, via `plate_to_robot` qui applique `ROBOT_BASE_ROTATION_DEG = 225` puis
les origines), monde absolu (m, via `_abs_pose`). Toute nouvelle géométrie doit
parcourir **exactement la même chaîne** que les waypoints. Recette de référence :
`ur5_sim/visualization/surface._plate_corner_world`.

**Contraintes CB3 / PolyScope 3.x.** Pas de `movel` ni `stopl` dans un thread
secondaire. Pas de tranche de liste (`[0:3]`). Pas d'indexation chaînée
`a[i][j]`. Pas de `for`, uniquement `while`. Budget mémoire vérifié par
`_validate_script_memory` (200 000 octets). Vitesses plafonnées à 0.250 m/s par
`_clamp_tcp_speed`. Le programme généré **n'actionne jamais** la pince : aucun
`rq_*`, aucun `set_payload`, aucun RS485 outil.

**Sémantique de `force_mode(task_frame, selection_vector, wrench, type, limits)`.**
Le vecteur `limits` n'est pas en newtons et ses unités changent par axe. Axe
**compliant** (sélection à 1) : vitesse TCP maximale (m/s ou rad/s). Axe **non
compliant** (sélection à 0) : écart de position maximal toléré entre pose réelle
et pose commandée (m ou rad), au-delà duquel le contrôleur déclenche « Force
mode: Maximum position deviation exceeded ». Ici la sélection est
`[0, 0, 1, 0, 0, 0]` : `FORCE_LIMIT_Z = 0.040` est une vitesse en m/s,
`FORCE_LIMIT_XY = 0.008` une distance en mètres. Seul `wrench` est en newtons.

**Plancher du capteur.** FT-300 postérieur à octobre 2017 : bruit Fz de 0.1 N
(écart type sur 1 s), seuil minimal recommandé de 1 N monté sur robot, plage
+/- 300 N, flux à 100 Hz. Sur CB3, `force_mode` régule sur l'estimation interne
du contrôleur (courants articulaires) **sauf** si le URCap FT 300 est installé et
l'option d'utilisation du signal du capteur en force mode activée. Vérification :
`zero_ftsensor()` puis `get_tcp_force()` pendant 10 s sans contact ; écart type
proche de 0.1 N = capteur, de l'ordre du newton = estimation interne.
Références : [manuel FT 300](https://assets.robotiq.com/website-assets/support_documents/document/online/FT_Sensor_Instruction_Manual_Web_20190322.zip/FT_Sensor_Instruction_Manual_Web/Content/Specifications.htm),
[manuel URScript](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-scriptmanual/G5/force_mode_task_frame_selection_vector.htm).

**Liaison par valeur des constantes.** `from design.params import X` copie la
valeur à l'import : modifier `design.params.X` ensuite ne change rien pour
`design/export.py` ni `ur5_sim/config.py`. `design/trajectory.py:82-85` et
`design/app.py:185-196` contournent déjà le problème au cas par cas. C'est le
sujet central de `plan_variables_UI.md`, à connaître ici pour ne pas introduire
de nouvel import par valeur.

### 0.5 État des travaux, au 29 juillet 2026

Rien de ce plan n'est implémenté. Le code est fonctionnel dans son mode actuel.

| Sujet | État |
| :--- | :--- |
| Descente au contact sous force active | fait (section 2.1) |
| Blend sous `force_mode` | ouvert (section 2.2) |
| Force Z en global URScript | à faire, section 5, phase 1 |
| Mode paramétrique cycles 1-3 | à faire, sections 3 à 10 |
| Interface de réglage des variables | autre plan |
| Option de sondage `PROBE_MODE` | amélioration ultérieure, code conservé inerte |

### 0.6 Pièges concrets, à lire avant d'agir

- **`etalement.urp` a été modifié à la main par l'opérateur pour des essais
  robot.** Il porte `FORCE_LIMIT_XY = 0.002` et les anciens globaux du sondage 3
  points. Ce n'est pas une incohérence à corriger d'office ; ne pas lancer
  `--export-urp` sans accord explicite, la commande écrase le fichier sans
  avertissement.
- `FORCE_LIMIT_XY` est passée de 0.002 à 0.008 m après un arrêt de protection
  réel sur le robot. Ne pas revenir à 0.002.
- Le sondage 3 points ne doit être ni supprimé, ni réactivé, ni réécrit
  (section 6).
- Le simulateur ne voit que les poses littérales `p[...]` des lignes `movel` :
  toute émission paramétrique rend la validation hors ligne aveugle (section 7).
- La géométrie des cycles 1-3 est un porteur composite dont la phase
  épicycloïdale suit l'abscisse curviligne cumulée (section 3.1).
- Supprimer le blend ne rend pas le mouvement continu et casse les durées du
  protocole (section 3.4) : trancher par une mesure, pas par un raisonnement.
- Dépôt git : branche `main`, remote `LARi-UQAC/UR5-Script-Design`. Le prénom de
  l'étudiante associée au projet ne doit apparaître dans aucun artefact public.
- `requirements.txt` porte des invariants de dépendances à ne pas relâcher
  (`swift-sim==1.1.0` impose `websockets<13`, correctifs locaux de
  `roboticstoolbox-python==1.1.1` sur Python 3.13).

### 0.7 Comment reprendre

1. Lire `ARCHITECTURE.md` sections 3 à 6, puis `design/params.py` en entier.
2. Établir la référence avant toute modification : `python -m ur5_sim --check` et
   `python -m unittest discover -s tests -p "test_*.py"`.
3. Suivre les phases de la section 9 dans l'ordre, avec le plugin Superpowers
   (section 9, sous-section « Exécution avec le plugin Superpowers »).
4. Mettre à jour `README.md`, `ARCHITECTURE.md` et `CLAUDE.md` à la phase 8 :
   aucun livrable n'est complet sans cette mise à jour.

---

## 1. Objectif

Réduire la taille du `.script` exporté et garantir un maintien continu des 6 N
perpendiculaires au plan de la plaque pendant toute la trajectoire de contact
(cycles 1-3 circulaires, cycles 4-6 rectilignes), en déportant le calcul de la
trajectoire épicycloïdale du Python vers le contrôleur UR5.

Fichiers concernés dans l'état actuel :

| Rôle | Fichier |
| :--- | :--- |
| Constantes protocole | `design/params.py` |
| Génération URScript / URP | `design/export.py` (`_build_urscript_lines`, lignes 99-459) |
| Générateurs de trajectoire | `design/trajectory.py` (`circular_cycle`, lignes 73-152) |
| UI et chemin d'export interactif | `design/app.py` (`export_current_urscript`, lignes 313-343) |
| Validation hors ligne | `ur5_sim/parsing/urscript.py`, `ur5_sim/cli.py` |

---

## 2. État réel du mode discret (mesuré)

### 2.0 Métriques du `etalement.script` courant

Fichier exporté depuis l'UI (mode Triangulé actif sur les cycles 4-6) :

| Mesure | Valeur |
| :--- | :--- |
| Lignes | 818 |
| Taille | 113 131 octets, soit 56.6 % de `URSCRIPT_MAX_BYTES` (200 000) |
| `movel` total | 661 |
| `movel` par cycle circulaire (1-3) | 203 |
| `movel` par cycle rectiligne (4-6) | 17 à 18 |
| `movel` portant `r=URSCRIPT_BLEND` | 642 |
| Paires `force_mode` / `end_force_mode` | 6 (une par cycle) |

Le budget mémoire n'est donc pas le facteur limitant aujourd'hui : la
justification première du mode paramétrique n'est plus la taille, mais la
qualité du mouvement et la lisibilité du programme.

### 2.1 Lacune 1 (descente au contact sans force) - FAIT

La séquence corrigée est en place dans `design/export.py`, lignes 394-408 :

```urscript
movel(apply_correction(pose_transit_in), v=transit)   # 394-395
sleep(0.2) ; zero_ftsensor() ; sleep(0.2)             # 396-398
force_mode(MEAS_FRAME, [0,0,1,0,0,0], [0,0,-6.0,...]) # 403-406
movel(apply_correction(pose_contact_deep),
      v=URSCRIPT_RECONTACT_V*SPEED_FACTOR)            # 407-408
```

`pose_contact_deep` vise `ROBOT_Z_SURFACE - FORCE_CONTACT_DEPTH`
(`FORCE_CONTACT_DEPTH = 0.005` m, `design/params.py:98`) : le régulateur de force
arrête la descente au contact avant d'atteindre la cible. La descente se fait à
`URSCRIPT_RECONTACT_V = 0.01` m/s, plus lente que le `URSCRIPT_CONTACT_V = 0.05`
m/s envisagé initialement.

`ur5_sim` connaît cette descente volontairement trop profonde et filtre
l'événement `SURFACE_DEVIATION` correspondant (`ur5_sim/cli.py:67`,
`_is_force_target_depth`, couvert par `tests/test_force_target_filter.py`).

### 2.2 Lacune 2 (blend radius sous force_mode) - OUVERT

`design/export.py:416-417` émet toujours :

```python
lines.append(f'  movel(apply_correction({pose_wp}), '
             f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR, r=URSCRIPT_BLEND)')
```

`URSCRIPT_BLEND = 0.002` m (`design/params.py:104`), présent sur 642 `movel` du
script courant. C'est le seul des trois correctifs d'origine qui reste entier.
Sa résolution est traitée en section 3.4, qui corrige l'analyse initiale.

### 2.3 Signature de `apply_correction` - CHANGÉE

La fonction émise ne prend plus qu'un argument (`design/export.py:377-379`) :

```urscript
def apply_correction(p_world):
  return pose_trans(MEAS_FRAME, pose_trans(pose_inv(NOMINAL_FRAME), p_world))
end
```

Les exemples URScript de la version initiale du plan (`apply_correction(p, dx, dy)`)
sont caducs. Toute nouvelle fonction embarquée doit appeler la forme à un
argument.

### 2.4 Lacune 3 (frame de référence) - PARTIELLEMENT RÉSOLUE, HYPOTHÈSE INITIALE INVALIDE

`force_mode` reçoit désormais `MEAS_FRAME` et non plus `get_actual_tcp_pose()`
(`design/export.py:403`). La dérive du TCP pendant le cycle n'affecte donc plus
la direction de la force : ce point est réglé.

En revanche, la solution proposée en mai (passer la normale sondée `NHAT`) n'est
plus applicable : le sondage 3 points qui produisait `NHAT` est désactivé
(`design/export.py:130-150` et 277-375, bloc conservé inerte ; voir
ARCHITECTURE.md section 6). Le sondage actif est le sondage Z 1 point
`probe_surface_z()` (`design/export.py:244-274`), qui écrit uniquement la
composante Z de `MEAS_FRAME` :

```urscript
MEAS_FRAME = p[NOMINAL_FRAME[0], NOMINAL_FRAME[1], touch[2],
               NOMINAL_FRAME[3], NOMINAL_FRAME[4], NOMINAL_FRAME[5]]
```

Conséquence à assumer : l'orientation de `MEAS_FRAME` reste nominale
(`ROBOT_RX = pi`, `RY = RZ = 0`). Si la plaque est inclinée, les 6 N sont
perpendiculaires au plan **nominal**, pas au plan réel. Le mode paramétrique
n'y change rien : il hérite du même `MEAS_FRAME`. La correction de ce biais
appartient au rework du sondage (mémoire projet : sondage 3 points à refaire),
pas à ce plan. Ne pas réintroduire `NHAT_*` dans les fonctions embarquées tant
que le sondage de plan n'est pas rétabli.

### 2.5 Limites `force_mode` - VALEURS CHANGÉES

`design/params.py:89-98`, avec la sémantique documentée sur place :

| Constante | Valeur | Sens réel |
| :--- | :--- | :--- |
| `FORCE_Z_TARGET` | 6.0 N | force cible |
| `FORCE_LIMIT_XY` | 0.008 m | déviation max tolérée en XY (axes NON compliants) |
| `FORCE_LIMIT_Z` | 0.040 m/s | vitesse max de l'axe compliant Z |
| `FORCE_LIMIT_ROT` | 0.35 rad | déviation angulaire max |

`FORCE_LIMIT_XY` est passée de 0.002 à 0.008 m parce que 2 mm déclenchaient
« Force mode: Maximum position deviation exceeded » : le doigt silicone traîne
latéralement et écarte le TCP du chemin commandé. Les exemples URScript qui
codaient `[0.002, 0.002, 0.040, 0.35, 0.35, 0.35]` en dur sont donc faux et sont
corrigés en section 10.

### 2.6 Densité de waypoints - DEUX CHEMINS D'EXPORT DIVERGENTS

C'est un point que la version initiale ignorait.

| Chemin | Waypoints des cycles 1-3 | Source |
| :--- | :--- | :--- |
| `python ur5_etalementv6.py --export` (headless) | `URSCRIPT_N_WAYPOINTS_CIRCULAR = 80` | `design/trajectory.py:50` |
| Bouton « Exporter URScript » de l'UI | **tous** les points du slider (50 à 2000, 203 dans le script courant) | `design/app.py:325`, `'waypoint_indices': list(range(len(pts)))` |

La recommandation initiale « passer 80 à 200 » est donc déjà satisfaite de fait
sur le chemin UI, et jamais sur le chemin headless. Il faut aligner les deux :
soit l'UI respecte `get_waypoint_indices`, soit `URSCRIPT_N_WAYPOINTS_CIRCULAR`
devient la valeur par défaut du slider. Sans cet alignement, `ur5_sim --check`
ne valide pas la même trajectoire que celle réellement exportée par l'opérateur.

### 2.7 Cycles 4-6 : le gain paramétrique est nul

`get_waypoint_indices(..., 'linear')` (`design/trajectory.py:40-48`) n'exporte
que les coins : 26 waypoints théoriques pour 13 passes, 17 à 18 dans le script
courant (mode Triangulé, `design/trajectory.py:172-199`). Une fonction embarquée
`run_linear_with_force` remplacerait 18 `movel` littéraux par une boucle plus une
table de coordonnées de même taille : aucun gain de mémoire, perte de lisibilité,
et perte de la validation `ur5_sim` (section 7).

**Décision : le mode paramétrique ne concerne que les cycles 1-3.** Pour les
cycles 4-6, seul le traitement du blend (section 3.4) s'applique.

---

## 3. Analyse du mouvement sous force_mode

### 3.1 Géométrie réelle des cycles 1-3

`circular_cycle` (`design/trajectory.py:73-152`) n'est pas « une droite porteuse
plus des cercles ». Le porteur est composite :

1. section droite initiale `y_start -> y_bot` sur `x = xs[0]` (si `CIRC_Y_START < y_bot`) ;
2. `CIRC_N_PASSES` passes verticales alternées `y_bot <-> y_top` sur `x = xs[i]` ;
3. entre deux passes, un demi-tour de rayon `r_turn = spacing/2` centré entre `xs[i]` et `xs[i+1]` ;
4. section droite finale `y_bot -> y_start` sur `x = xs[-1]`.

La phase des petits cercles est ensuite calculée sur l'**abscisse curviligne
cumulée du porteur entier** (`design/trajectory.py:139-147`) :

```python
s_norm = s / s[-1]
theta  = 2*pi * (CIRC_N_CIRCLES * n_passes) * s_norm
x = carrier_x + R*cos(theta)
y = carrier_y + R*sin(theta)
```

Toute fonction embarquée doit reproduire cette continuité de phase d'un segment
porteur au suivant. Une signature `execute_circular_pass(p_start, p_end, ...)`
telle que proposée en mai est insuffisante : elle ne peut pas décrire les
demi-tours ni transporter `s` d'un segment à l'autre. La décomposition correcte
est donnée en section 4.2.

### 3.2 Impact du blend sur les cycles 1-3

Avec `R = CIRC_R_CIRCLE = 5` mm, la circonférence d'un petit cercle est
31.4 mm. À 203 waypoints par cycle pour environ 2 700 mm de trajet, l'espacement
moyen est de 13 mm, soit 2 à 3 waypoints par petit cercle : la discrétisation est
déjà grossière par rapport au motif. Un blend de 2 mm sur des segments de 13 mm
« coupe » donc une fraction notable de chaque arc, et le régulateur de force
n'est pas garanti pendant l'interpolation de blend.

### 3.3 Impact du blend sur les cycles 4-6

`ΔY = (SURFACE_H - 2*MARGIN) / (LIN_N_PASSES - 1) = (50 - 8)/12 = 3.5` mm.
Avec `r = 2` mm, la transition inter-passes est en zone de blend sur environ
57 % de sa longueur, sur 12 transitions par cycle. Les passes droites (42 mm sans
waypoint intermédiaire) ne posent pas de problème.

### 3.4 CORRECTION de l'analyse initiale : supprimer le blend ne suffit pas et coûte cher

La version initiale concluait « supprimer `r=` et augmenter la densité ». Cette
conclusion est incomplète sur deux points :

1. **Le mode paramétrique ne rend pas le mouvement continu.** Une boucle `while`
   qui enchaîne des `movel` sans `r` produit exactement le même profil qu'une
   suite de `movel` littéraux sans `r` : le planificateur décélère à zéro à
   chaque point. Le « flux de mouvement continu » annoncé en mai est faux. Le
   mode paramétrique gagne en taille et en lisibilité, pas en fluidité.
2. **Le coût temporel est prohibitif.** 203 arrêts complets par cycle, à
   `URSCRIPT_ACCEL = 0.8` m/s² et `CIRC_SPEED = 36` mm/s, ajoutent de l'ordre de
   0.1 à 0.3 s par point, soit 20 à 60 s par cycle contre `CIRC_DURATION = 11` s
   visées par le protocole. Le protocole d'étalement n'est plus respecté.

Trois options réelles, à trancher expérimentalement :

| Option | Principe | Force | Fluidité | Validation `ur5_sim` |
| :--- | :--- | :--- | :--- | :--- |
| A - blend réduit | `r = 0.0005` m (0.5 mm), très inférieur au rayon des cercles et à l'espacement des waypoints | perturbation résiduelle bornée, à mesurer | conservée | inchangée |
| B - blend nul | `r` retiré | garantie maximale | perdue, protocole hors durée | inchangée |
| C - `speedl` en boucle | XY piloté en vitesse sous `force_mode`, Z compliant | garantie, pas de transition de blend | continue | perdue (aucune pose littérale) |

Recommandation : **A par défaut** (changement d'une constante, mesurable
immédiatement sur URSim puis sur robot avec `get_tcp_force()`), **C évaluée en
parallèle** comme cible à moyen terme puisque c'est la seule qui donne à la fois
la fluidité et la garantie de force sur un CB3. B est conservée comme référence
de mesure, pas comme mode de production.

Le critère de décision est une mesure, pas un raisonnement : enregistrer la force
Z pendant un cycle complet dans les trois configurations et comparer l'écart type
et les pics à la cible 6.0 +/- 0.5 N.

---

## 4. Architecture du mode paramétrique (cycles 1-3)

### 4.1 Contraintes CB3 à respecter (ARCHITECTURE.md section 4)

- Pas de `movel` ni de `stopl` dans un thread secondaire.
- Pas de tranche de liste (`[0:3]`).
- Pas d'indexation chaînée `a[i][j]` : passer par une variable intermédiaire, ou
  mieux, émettre des listes **plates** de scalaires (`XS_1`, `YS_1`, ...) plutôt
  que des listes de poses.
- Pas de `for` : uniquement `while`.
- Budget mémoire vérifié par `_validate_script_memory` (`design/export.py:81-96`).
- Vitesses plafonnées par `_clamp_tcp_speed` (`design/export.py:69-78`).

### 4.2 Fonctions embarquées proposées

Deux primitives porteuses, plus un enrobage par cycle. La phase épicycloïdale est
transportée par l'abscisse curviligne cumulée `s`, passée en entrée et retournée
en sortie.

1. `carrier_line(x0, y0, x1, y1, s_in, s_total, n_turns, radius, n_steps)`
   - porteur rectiligne ; retourne `s_in + longueur du segment`.
2. `carrier_arc(cx, cy, r_turn, a0, a1, s_in, s_total, n_turns, radius, n_steps)`
   - porteur en demi-tour ; retourne `s_in + r_turn*|a1-a0|`.
3. `run_circular_cycle_N()` : descente sécurisée (identique à l'actuelle),
   puis appels successifs des primitives dans l'ordre porteur, puis
   `end_force_mode()` et transit de sortie.

Python conserve la responsabilité du calcul géométrique du porteur (`xs`,
`r_turn`, `y_bot`, `y_top`, `s_total`) : ces valeurs sont émises comme constantes
littérales, ce qui garde `design/trajectory.py` comme source unique de vérité de
la géométrie et évite de dupliquer la logique en URScript.

### 4.3 Modifications Python

`design/params.py` - ajouter :

```python
# --- Mode d'exportation URScript ---
URSCRIPT_EXPORT_MODE: str = 'discret'   # 'discret' | 'parametrique'
URSCRIPT_PARAM_N_STEPS: int = 400       # pas par cycle circulaire en mode parametrique
URSCRIPT_BLEND_CONTACT: float = 0.0005  # m - blend sous force_mode (option A, section 3.4)
```

`URSCRIPT_BLEND` reste la valeur des mouvements hors contact ; introduire
`URSCRIPT_BLEND_CONTACT` évite de faire varier les deux ensemble.

`design/export.py` - découper `_build_urscript_lines` :

- `_emit_preamble(...)` : globals, sondage Z, `apply_correction` (lignes 154-381 actuelles) ;
- `_emit_cycle_discret(idx, cyc)` : corps actuel des lignes 383-425 ;
- `_emit_carrier_defs()` : définitions `carrier_line` / `carrier_arc`, émises
  seulement en mode paramétrique ;
- `_emit_cycle_parametrique(idx, cyc)` : descente sécurisée plus appels porteurs ;
- `_build_urscript_lines(cycles, mode=None)` : `mode` par défaut lu dans
  `params.URSCRIPT_EXPORT_MODE`, dispatch par cycle (`type == 'circular'` et mode
  paramétrique, sinon discret).

Le mode paramétrique a besoin des paramètres du porteur, pas d'un nuage de
points. Il faut donc que `circular_cycle` expose sa description géométrique.
Ajouter dans `design/trajectory.py` :

```python
def circular_carrier_segments(rotation_deg=0):
    """Retourne la liste des segments porteurs ('line'|'arc', paramètres, longueur)
    et s_total, à partir des mêmes xs / r_turn / y_bot / y_top que circular_cycle."""
```

et refactorer `circular_cycle` pour consommer cette description, afin que les
deux modes ne puissent pas diverger.

`design/app.py` - le bas de figure est occupé par trois boutons
(`design/app.py:287-300`, positions imposées dans `_apply_ui_layout`, lignes
379-391). Deux options :

- ajouter un quatrième bouton bascule « Export: Discret / Param » et repasser la
  rangée à quatre positions dans `_apply_ui_layout` ;
- ou un `RadioButtons` dans l'espace libre à gauche des sliders.

Le bouton bascule est cohérent avec `btn_shape` (« Tri/Rect ») déjà présent et
demande moins de retouche de layout. `export_current_urscript`
(`design/app.py:313-343`) passe alors le mode à `generate_urscript`.

`design/app.py:main()` - ajouter `--export-mode {discret,parametrique}` pour que
le chemin headless soit testable en CI locale.

---

## 5. Force Z paramétrable depuis le pendant PolyScope

### 5.1 Exigence

L'opérateur doit pouvoir changer la force de contact sur le pendant, sans
relancer l'export Python. Aujourd'hui c'est impossible : la valeur est un
littéral inliné.

### 5.2 État actuel : valeur codée en dur, six occurrences

`design/export.py:403-406` inline `FORCE_Z_TARGET` dans le vecteur de wrench :

```python
lines.append(f'  force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], '
             f'[0, 0, {-FORCE_Z_TARGET:.1f}, 0, 0, 0], 2, '
             f'[{FORCE_LIMIT_XY}, {FORCE_LIMIT_XY}, {FORCE_LIMIT_Z}, '
             f'{FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}])')
```

Résultat dans le fichier livré, une fois par cycle :

```urscript
force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], [0, 0, -6.0, 0, 0, 0], 2, [0.008, 0.008, 0.04, 0.35, 0.35, 0.35])
```

Emplacements à modifier tant que la valeur reste un littéral (repli documenté,
à ne pas privilégier) :

| Fichier | Lignes | Nature |
| :--- | :--- | :--- |
| `design/export.py` | 403-406 | source de l'émission, seul point à corriger durablement |
| `etalement.script` | 93, puis une ligne `force_mode` par `def cycle_N()` | fichier généré, écrasé au prochain export |
| `etalement.urp` | 170, 265, 360, et suivantes | fichier généré, écrasé au prochain export |

Éditer les fichiers générés fonctionne pour un essai ponctuel, mais la
modification est perdue au prochain export : ce n'est pas une solution.

### 5.3 Correctif : émettre un global URScript

Ajouter dans le préambule, à côté des vitesses (`design/export.py:177-189`) :

```python
f'global FORCE_Z_TARGET = {force_z:.1f}  #sym:FORCE_Z_TARGET',
f'global FORCE_LIMIT_XY = {FORCE_LIMIT_XY}  #sym:FORCE_LIMIT_XY',
f'global FORCE_LIMIT_Z = {FORCE_LIMIT_Z}  #sym:FORCE_LIMIT_Z',
f'global FORCE_LIMIT_ROT = {FORCE_LIMIT_ROT}  #sym:FORCE_LIMIT_ROT',
```

et remplacer l'appel par :

```python
lines.append('  force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], '
             '[0, 0, -FORCE_Z_TARGET, 0, 0, 0], 2, '
             '[FORCE_LIMIT_XY, FORCE_LIMIT_XY, FORCE_LIMIT_Z, '
             'FORCE_LIMIT_ROT, FORCE_LIMIT_ROT, FORCE_LIMIT_ROT])')
```

`force_mode` accepte des variables dans ses arguments ; la valeur est relue à
chaque appel, donc à chaque début de cycle. Une modification faite entre deux
cycles prend effet au cycle suivant. Les limites suivent le même traitement pour
qu'il n'y ait qu'un seul endroit à toucher.

Le suffixe `#sym:FORCE_Z_TARGET` reprend la convention déjà utilisée pour les
vitesses : il donne à l'opérateur et au simulateur un point d'ancrage textuel
unique.

### 5.4 Ce que l'opérateur peut faire sur le pendant, par niveau

Le `.urp` généré est un programme composé d'**un seul noeud Script**
(`design/export.py:476-505` : le texte du script est inséré tel quel dans
`<program><robot><script>`). Cela conditionne ce qui est éditable dans l'IHM.

| Niveau | Manipulation sur le pendant | Disponible avec le correctif 5.3 |
| :--- | :--- | :--- |
| 1 - lecture | Onglet Program, sous-onglet Variables : les globaux sont visibles pendant l'exécution | oui |
| 2 - édition de la ligne | Onglet Program, sélectionner le noeud Script, Edit, modifier la ligne `global FORCE_Z_TARGET = 6.0` | oui, une seule ligne à changer |
| 3 - champ de saisie dédié, sans toucher au code | nécessite soit une variable d'installation, soit un vrai arbre de noeuds PolyScope dans le `.urp` | non, voir 5.5 |

Le niveau 2 répond à la demande avec le correctif 5.3 : une ligne, en tête de
programme, identifiée par son commentaire `#sym:`. Documenter cette procédure
dans le bloc d'en-tête émis par `_build_urscript_lines`, à côté du bloc
« PINCE 2F-85 : AUCUN ACTIONNEMENT ».

### 5.5 Niveau 3 : les deux voies, et leur coût

- **Variable d'installation.** Déclarer `FORCE_Z_TARGET_INSTALL` dans
  l'installation du robot ; le script l'affecte à son global au démarrage. La
  valeur devient éditable dans Installation > Variables, sans ouvrir le
  programme. Coût faible, mais couplage fort : si l'installation chargée ne
  déclare pas la variable, le programme ne compile pas. URScript n'offre aucun
  test d'existence de variable, donc pas de repli conditionnel possible. À
  émettre uniquement derrière un drapeau explicite,
  `URSCRIPT_FORCE_FROM_INSTALLATION` dans `design/params.py`, désactivé par
  défaut.
- **Arbre de noeuds PolyScope.** Générer un vrai `.urp` structuré (noeud
  d'assignation `FORCE_Z_TARGET := 6.0` suivi du noeud Script) au lieu de
  l'unique noeud Script actuel. La variable apparaît alors dans l'IHM comme une
  variable de programme, avec champ de saisie. Coût réel : réécriture de
  `generate_urp`, qui ne produit aujourd'hui que trois éléments XML. À traiter
  comme un chantier séparé, pas dans ce plan.

### 5.6 Garde-fou sur la valeur saisie

Ajouter dans `design/export.py` un `_clamp_force_target()` sur le modèle de
`_clamp_tcp_speed` (`design/export.py:69-78`), et les bornes correspondantes dans
`design/params.py` :

```python
FORCE_Z_MIN_N: float = 2.0   # N - plancher utile du FT-300 monte sur robot
FORCE_Z_MAX_N: float = 20.0  # N - garde-fou protocole / echantillon
```

Justification du plancher : sur un FT 300 postérieur à octobre 2017, le bruit de
signal sur Fz est de 0.1 N (écart type sur 1 s) et le seuil minimal recommandé
une fois le capteur monté sur le robot est de 1 N. Une consigne inférieure à
2 N n'est donc pas régulable de façon stable, d'autant que le doigt silicone
ajoute sa propre compliance. La cible protocole reste 6.0 +/- 0.5 N.

Point à vérifier sur le robot avant de se fier à une consigne fine : sur CB3,
`force_mode` régule sur l'estimation interne du contrôleur (courants
articulaires) sauf si le URCap FT 300 est installé et l'option d'utilisation du
signal du capteur en force mode activée. Test : `zero_ftsensor()` puis
enregistrement de `get_tcp_force()` pendant 10 s sans contact ; un écart type
proche de 0.1 N indique la lecture du FT 300, un écart type de l'ordre du newton
indique l'estimation interne.

### 5.7 Effet sur le simulateur

`ur5_sim/parsing/urscript.py:60-62` (`SPEED_GLOBAL_RE`) sait déjà lire
`global <NOM> = <valeur>`. Ajouter `FORCE_Z_TARGET` à une table de globaux
contrôlés et comparer la valeur émise à `design.params.FORCE_Z_TARGET`, avec
report dans `ur5_sim/reporting`. Aujourd'hui `FORCE_Z_TARGET_N` côté
`ur5_sim/config.py` ne sert que d'étiquette pour le HUD.

---

## 6. Sondage de surface : 1 point actif, 3 points conservé pour la suite

### 6.1 Règle pour ce plan

Le sondage 3 points **n'est pas utilisable** en l'état et **ne doit pas être
supprimé**. Aucune des phases décrites en section 9 ne retire, ne renomme ni ne
réécrit le code correspondant. Le refactor de `_build_urscript_lines` (phase 4)
transporte le bloc inerte tel quel, placeholders compris.

### 6.2 Inventaire du code à préserver

| Emplacement | Contenu | État |
| :--- | :--- | :--- |
| `design/export.py:130-150` | calcul de `_nhat` et `probe_blocks` | commenté |
| `design/export.py:277-375` | `probe_one()` et `probe_surface_plane()` en chaîne littérale inerte, avec placeholders `<approach_P1>`, `<floor_P1>`, etc. | non émis |
| `design/params.py:115-125` | `PROBE_POINTS_PLATE_MM`, `PROBE_TILT_MAX_RAD`, `PROBE_RETRY_MAX`, `PROBE_FLOOR_PLATE_MM` | définis, inutilisés par l'émetteur |
| `ur5_sim/probe.py` | rejeu géométrique du sondage 3 points | inerte, `SIM_PROBE_ENABLE = False` |
| `ur5_sim/parsing/urscript.py:42-50` | `PROBE_DEF_RE`, `PROBE_ONE_RE` | prêts, sans entrée à parser |
| `tests/test_probe_sim.py` | tests du rejeu | parqués |

Actif à la place : `probe_surface_z()` (`design/export.py:244-274`), sondage Z
1 point par contact de force, qui écrit la composante Z de `MEAS_FRAME`.

### 6.3 Pourquoi il ne peut pas servir maintenant

Le sondage 3 points descend vers des poses « plancher » à Z codé en dur : il
suppose connue la hauteur de la plaque, ce qui est faux puisque cette hauteur
dépend du jog de l'opérateur et de la pose de la plaque. Il ne gère ni la
hauteur inconnue ni la rotation de la plaque. C'est aussi la raison pour
laquelle `NHAT` n'est plus disponible et pourquoi la force reste normale au plan
nominal (section 2.4).

### 6.4 Forme visée à la prochaine amélioration : une option d'export

Le sondage devient un choix de l'utilisateur, au même titre que le mode
d'exportation :

```python
# design/params.py
PROBE_MODE: str = 'z1'   # 'z1' | 'plane3'
```

- `design/app.py` : bouton bascule « Sonde: Z1 / Plan3 », à côté de
  « Tri/Rect » et du futur « Export: Discret / Param ».
- `design/app.py:main()` : `--probe-mode {z1,plane3}`.
- `design/export.py` : `_emit_probe_z1()` et `_emit_probe_plane3()`, un seul des
  deux émis. En `z1`, le script livré ne contient aucune ligne du sondage 3
  points, donc aucun coût mémoire.
- `plane3` reste refusé par l'émetteur (message explicite) tant que la condition
  de réactivation ci-dessous n'est pas remplie.

### 6.5 Conditions de réactivation de `plane3`

1. Chaque point est atteint par descente surveillée en force (`speedl` plus
   lecture de `get_tcp_force()` plus `stopl`, comme `probe_surface_z`), et non
   par une pose plancher à Z fixe. La hauteur de la plaque reste inconnue avant
   le sondage.
2. `MEAS_FRAME` reconstruit porte la hauteur **et** l'inclinaison ; le garde-fou
   `PROBE_TILT_MAX_RAD` reste actif.
3. `force_mode` utilise alors la normale mesurée, ce qui referme la lacune
   décrite en section 2.4.
4. Côté simulateur : `SIM_PROBE_ENABLE = True`, rejeu de `ur5_sim/probe.py`
   corrigé, `tests/test_probe_sim.py` réactivé et vert.
5. Contraintes CB3 inchangées : pas de `movel` ni de `stopl` dans un thread, pas
   de tranche `[0:3]`.

Tant que ces cinq points ne sont pas satisfaits, `plane3` reste du code mort
documenté, pas une option offerte à l'opérateur.

---

## 7. Impact sur `ur5_sim` - point bloquant à traiter avant l'implémentation

C'est la contrainte majeure absente de la version initiale.

`ur5_sim/parsing/urscript.py` extrait un littéral `p[x, y, z, rx, ry, rz]` par
ligne `movel` (`ANY_POSE_RE`, lignes 30-33 ; `parse_poses`, ligne 104). En mode
paramétrique, les poses sont calculées à l'exécution dans une boucle `while` :
**le parseur ne voit plus aucune pose**, donc plus d'IK, plus de contrôle des
limites articulaires, plus de contrainte de surface, plus de rejeu Swift. La
validation hors ligne tombe à zéro sur les cycles 1-3.

Trois réponses possibles :

| Réponse | Effort | Fidélité de validation |
| :--- | :--- | :--- |
| 1. Export jumeau : générer aussi un `etalement_check.script` discret à partir des mêmes segments porteurs, et valider celui-là | faible | élevée si et seulement si les deux sorties dérivent de `circular_carrier_segments` |
| 2. Bloc d'en-tête paramétrique lu par `ur5_sim` : émettre les segments en commentaires structurés (`#seg:line,x0,y0,x1,y1,...`) et régénérer le nuage de points en Python côté simulateur | moyen | élevée, et le fichier livré au robot reste unique |
| 3. Mini-interpréteur URScript dans `ur5_sim` (boucles, variables locales, `sin`/`cos`) | élevé | maximale mais hors proportion |

Recommandation : **réponse 2**. Le commentaire structuré voyage avec le script
livré, ne coûte rien au contrôleur, et se parse avec une regex du même style que
celles déjà présentes. Prévoir `SEG_RE` dans `ur5_sim/parsing/urscript.py` et une
extension de `parse_poses` qui, en présence de blocs `#seg:`, densifie les
segments avec la même formule que `circular_cycle`.

Sans l'un de ces trois mécanismes, le mode paramétrique ne doit pas être activé
par défaut : il retirerait le seul garde-fou hors ligne du projet.

---

## 8. Comparaison des modes, chiffres révisés

| Caractéristique | Discret actuel | Discret + option A | Paramétrique (cycles 1-3) |
| :--- | :--- | :--- | :--- |
| Lignes du `.script` | 818 | 818 | environ 250 |
| Taille | 113 kB (57 % du budget) | 113 kB | environ 35 kB |
| `movel` littéraux | 661 | 661 | 55 environ (cycles 4-6 et transits) |
| Force pendant le contact | continue sauf transitions de blend 2 mm | continue, blend 0.5 mm | identique au discret à densité égale |
| Fluidité | conservée | conservée | identique (voir 3.4) |
| Durée par cycle circulaire | environ 11 s | environ 11 s | dépend de `n_steps`, à mesurer |
| Perpendicularité de la force | plan nominal (limite du sondage 1 point) | idem | idem |
| Validation `ur5_sim` | complète | complète | nulle sans le mécanisme de la section 7 |
| Complexité Python | interpolation 2D dans `trajectory.py` | idem | idem plus émission de segments |

Conclusion honnête : le mode paramétrique est un gain de lisibilité et de taille,
pas un gain de qualité de force. Le vrai levier sur la force est le traitement du
blend (section 3.4) et, à terme, le rework du sondage de plan (section 2.4).

---

## 9. Étapes d'implémentation

**Phase 0 - Alignement des chemins d'export** (préalable, sans quoi rien n'est
mesurable) : faire converger l'UI et le headless sur la même densité de waypoints
(section 2.6). Vérifier avec `python -m ur5_sim --check` sur les deux sorties.

**Phase 1 - Force Z en global URScript** (indépendante des autres phases, à
livrer en premier) : émettre `global FORCE_Z_TARGET` et les trois globaux de
limites, remplacer les littéraux dans `force_mode` (`design/export.py:403-406`),
ajouter `_clamp_force_target()` et les bornes `FORCE_Z_MIN_N` / `FORCE_Z_MAX_N`,
documenter dans l'en-tête du script la procédure d'édition sur le pendant
(section 5.3 et 5.4). Rétro-compatible : le script reste identique dans son
comportement à consigne inchangée.

**Phase 2 - Blend sous force_mode** : introduire `URSCRIPT_BLEND_CONTACT`,
émettre `r=URSCRIPT_BLEND_CONTACT` sur les `movel` de contact
(`design/export.py:416-417`), garder `URSCRIPT_BLEND` pour les transits.
Mesurer la force Z sur URSim puis sur robot dans les configurations A, B et C de
la section 3.4. Retenir la configuration sur la mesure.

**Phase 3 - Description géométrique du porteur** : `circular_carrier_segments`
dans `design/trajectory.py`, `circular_cycle` refactorée pour la consommer.
Test de non-régression : le nuage de points produit doit être identique au
courant à 1e-9 près.

**Phase 4 - Émission paramétrique** : `_emit_carrier_defs` et
`_emit_cycle_parametrique` dans `design/export.py`, avec listes plates de
scalaires et `apply_correction(p_world)` à un argument.

**Phase 5 - Validation hors ligne** : blocs `#seg:` et extension du parseur
(section 7, réponse 2). Critère de réussite : `python -m ur5_sim --check` sur un
script paramétrique reproduit le même verdict IK que sur son équivalent discret.

**Phase 6 - UI et CLI** : bouton bascule dans `design/app.py`, drapeau
`--export-mode`, mode discret conservé par défaut.

**Phase 7 - URSim puis robot** : chargement du programme, surveillance de
`get_tcp_force()`, comparaison des durées de cycle avec `CIRC_DURATION = 11` s.
Vérifier au passage l'édition de `FORCE_Z_TARGET` depuis le pendant : modifier la
ligne, relancer, confirmer la nouvelle consigne au capteur.

**Phase 8 - Documentation** (obligatoire, pas optionnelle) : mettre `README.md`,
`ARCHITECTURE.md` et `CLAUDE.md` au niveau du code livré. Détail en 9.2. Une
phase n'est terminée que lorsque la documentation correspondante est à jour ; à
défaut, la carte du code diverge du code et la prochaine session à froid repart
sur une base fausse.

**Hors périmètre de ce plan, amélioration suivante** : option de sondage
`PROBE_MODE` et remise en service du sondage 3 points (section 6). Le code reste
en place et inerte d'ici là ; aucune phase ci-dessus ne le touche.

### 9.1 Exécution avec le plugin Superpowers

L'exécution de ce plan passe par les compétences du plugin Superpowers, invoquées
avec l'outil `Skill` et annoncées avant usage. Séquence attendue :

| Moment | Compétence | Usage ici |
| :--- | :--- | :--- |
| Avant de toucher au code | `superpowers:using-git-worktrees` | isoler la branche de travail de l'espace courant |
| Au démarrage de l'exécution | `superpowers:executing-plans` | dérouler les phases une par une avec points de contrôle |
| À chaque phase de code | `superpowers:test-driven-development` | écrire le test qui échoue avant l'implémentation (le test d'identité octet pour octet de la phase 3 est le cas d'école) |
| Sur tout écart de comportement | `superpowers:systematic-debugging` | avant de proposer un correctif, notamment sur les écarts URSim ou robot |
| Phases indépendantes | `superpowers:subagent-driven-development` ou `superpowers:dispatching-parallel-agents` | par exemple tests et documentation menés en parallèle des phases 4 et 5 |
| Avant fusion | `superpowers:requesting-code-review` puis `superpowers:receiving-code-review` | revue du travail, puis traitement rigoureux des retours |
| Avant toute annonce d'achèvement | `superpowers:verification-before-completion` | aucune affirmation de succès sans sortie de commande à l'appui |
| Fin de branche | `superpowers:finishing-a-development-branch` | décision d'intégration, la fusion vers `main` reste validée par l'humain |

La phase de conception est déjà faite : ce document est le produit de
`superpowers:brainstorming`. Ne relancer cette compétence que si le périmètre
change. Si l'exécutant veut une granularité tâche par tâche, passer par
`superpowers:writing-plans` pour dériver un plan d'exécution de ce document, sans
le réécrire.

### 9.2 Documentation à mettre à jour

| Fichier | Section | Mise à jour attendue |
| :--- | :--- | :--- |
| `README.md` | « Export options » | décrire les deux modes d'export (discret, paramétrique), leur portée (cycles 1-3), et la conséquence sur la validation `ur5_sim` |
| `README.md` | « Starting the software » | ajouter le drapeau `--export-mode {discret,parametrique}` et la procédure d'édition de `FORCE_Z_TARGET` sur le pendant |
| `README.md` | « What is in this repository » | mentionner les globaux de force éditables si la phase 1 est livrée |
| `ARCHITECTURE.md` | section 2, table `design/` | nouvelles fonctions d'émission (`_emit_preamble`, `_emit_cycle_discret`, `_emit_cycle_parametrique`, `_emit_carrier_defs`) et `circular_carrier_segments` dans `trajectory.py` |
| `ARCHITECTURE.md` | section 4, contraintes CB3 | force et limites émises en globaux, `URSCRIPT_BLEND_CONTACT`, interdiction d'indexation chaînée |
| `ARCHITECTURE.md` | section 5, surrogate de force | lecture du global `FORCE_Z_TARGET` par le simulateur |
| `ARCHITECTURE.md` | section 6, sondage | statut inchangé, option `PROBE_MODE` annoncée comme amélioration suivante |
| `ARCHITECTURE.md` | section 9, tests | nouveaux fichiers de tests de la table ci-dessous |
| `ARCHITECTURE.md` | section 10, règles pour la suite | remplacer le prochain chantier par ce qui reste réellement ouvert |
| `CLAUDE.md` | « Common commands », « High-level architecture » | mêmes ajouts, format condensé ; ne pas dupliquer le détail d'`ARCHITECTURE.md` |

Vérifier que les liens des documents modifiés résolvent, et que les numéros de
ligne cités dans ce plan sont corrigés s'ils ont bougé.

### Tests à ajouter ou à étendre

| Test | Objet |
| :--- | :--- |
| `tests/test_trajectory_segments.py` (nouveau) | `circular_carrier_segments` reproduit `circular_cycle` |
| `tests/test_export_parametric.py` (nouveau) | le script paramétrique contient les defs porteuses, aucune indexation chaînée, aucun `[0:3]`, budget mémoire respecté |
| `tests/test_urscript_parse.py` (étendre) | parsing des blocs `#seg:` et densification |
| `tests/test_limits.py` (étendre) | vitesse TCP en mode paramétrique |
| `tests/test_force_global.py` (nouveau) | `global FORCE_Z_TARGET` émis une fois, aucun littéral de force restant dans les `force_mode`, `_clamp_force_target` borne bien sous `FORCE_Z_MIN_N` et au-dessus de `FORCE_Z_MAX_N` |
| `tests/test_probe_sim.py` (rester parqué) | ne pas réactiver dans ce plan ; sert de garde-fou pour la remise en service future du sondage 3 points |

Rappel d'exécution (unittest, pas de pytest dans le `.venv`) :

```bash
python -m unittest discover -s tests -p "test_*.py"
```

---

## 10. Squelettes URScript (CB3, API courante)

Les exemples ci-dessous respectent la signature `apply_correction(p_world)`, les
limites de `design/params.py`, et l'interdiction d'indexation chaînée.

### 10.1 Primitives porteuses (cycles 1-3)

```urscript
# Porteur rectiligne. Retourne l'abscisse curviligne cumulee en sortie.
def carrier_line(x0, y0, x1, y1, s_in, s_total, n_turns, radius, n_steps, z_c, v_c):
  local dx = x1 - x0
  local dy = y1 - y0
  local seg_len = sqrt(dx*dx + dy*dy)
  local i = 1
  while i <= n_steps:
    local a = i / n_steps
    local cx = x0 + dx*a
    local cy = y0 + dy*a
    local theta = 2*3.14159265*n_turns*(s_in + seg_len*a)/s_total
    local px = cx + radius*cos(theta)
    local py = cy + radius*sin(theta)
    movel(apply_correction(p[px, py, z_c, 3.14159, 0.0, 0.0]),
          a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=v_c*SPEED_FACTOR, r=URSCRIPT_BLEND_CONTACT)
    i = i + 1
  end
  return s_in + seg_len
end

# Porteur en demi-tour (rayon r_turn, angles a0 -> a1 en radians).
def carrier_arc(cx0, cy0, r_turn, a0, a1, s_in, s_total, n_turns, radius, n_steps, z_c, v_c):
  local da = a1 - a0
  local seg_len = r_turn*norm(da)
  local i = 1
  while i <= n_steps:
    local a = i / n_steps
    local ang = a0 + da*a
    local cx = cx0 + r_turn*cos(ang)
    local cy = cy0 + r_turn*sin(ang)
    local theta = 2*3.14159265*n_turns*(s_in + seg_len*a)/s_total
    local px = cx + radius*cos(theta)
    local py = cy + radius*sin(theta)
    movel(apply_correction(p[px, py, z_c, 3.14159, 0.0, 0.0]),
          a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=v_c*SPEED_FACTOR, r=URSCRIPT_BLEND_CONTACT)
    i = i + 1
  end
  return s_in + seg_len
end
```

Points de vigilance :

- les poses sont construites littéralement en coordonnées robot, puis passées à
  `apply_correction` : c'est la même chaîne que les waypoints discrets, donc le
  même comportement vis-à-vis de `MEAS_FRAME` ;
- ne pas utiliser `pose_trans(p_carrier, offset)` pour ajouter le petit cercle :
  avec `RX = pi`, le repère outil est retourné et l'offset changerait de signe en
  Y ;
- `radius` et `n_turns` viennent de `CIRC_R_CIRCLE` et
  `CIRC_N_CIRCLES * CIRC_N_PASSES` ;
- `n_steps` par segment est dérivé de `URSCRIPT_PARAM_N_STEPS` au prorata de la
  longueur du segment, calculé côté Python.

### 10.2 Enrobage d'un cycle circulaire

```urscript
def cycle_1():
  # --- Cycle 1 - Circulaire (0deg), mode parametrique ---
  movel(apply_correction(p[<transit_in>]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=<v_transit>*SPEED_FACTOR)
  sleep(0.2)
  zero_ftsensor()
  sleep(0.2)
  force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], [0, 0, -FORCE_Z_TARGET, 0, 0, 0], 2,
             [FORCE_LIMIT_XY, FORCE_LIMIT_XY, FORCE_LIMIT_Z,
              FORCE_LIMIT_ROT, FORCE_LIMIT_ROT, FORCE_LIMIT_ROT])
  movel(apply_correction(p[<contact_deep>]), a=URSCRIPT_ACCEL*ACCEL_FACTOR,
        v=URSCRIPT_RECONTACT_V*SPEED_FACTOR)

  local s = 0.0
  s = carrier_line(<x0>, <y_start>, <x0>, <y_bot>, s, S_TOTAL_1, N_TURNS_1, R_CIRC, <n1>, Z_C, V_CIRC)
  s = carrier_line(<x0>, <y_bot>, <x0>, <y_top>, s, S_TOTAL_1, N_TURNS_1, R_CIRC, <n2>, Z_C, V_CIRC)
  s = carrier_arc(<cx1>, <y_top>, <r_turn>, 3.14159265, 0.0, s, S_TOTAL_1, N_TURNS_1, R_CIRC, <n3>, Z_C, V_CIRC)
  # ... passes et demi-tours suivants, emis par _emit_cycle_parametrique ...

  end_force_mode()
  movel(apply_correction(p[<transit_out>]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=<v_transit>*SPEED_FACTOR)
end
```

Les limites passent par les globaux émis en section 5.3, dont les valeurs sont
`[0.008, 0.008, 0.040, 0.35, 0.35, 0.35]` (section 2.5) : réutiliser 0.002
provoquerait l'arrêt de protection observé sur le robot. `FORCE_Z_TARGET` est le
global éditable sur le pendant, relu à chaque appel de `force_mode`.

### 10.3 Cycles 4-6

Inchangés, mode discret (section 2.7). Seule modification : `r=URSCRIPT_BLEND`
remplacé par `r=URSCRIPT_BLEND_CONTACT` sur les `movel` situés entre
`force_mode` et `end_force_mode`.

---

## 11. Vérification

1. Régénérer : `python ur5_etalementv6.py --export --export-urp --no-show`.
2. Contrôles statiques sur `etalement.script` :
   - `force_mode(MEAS_FRAME, ...)` précède chaque `movel(... contact_deep ...)` ;
   - une seule ligne `global FORCE_Z_TARGET = ...  #sym:FORCE_Z_TARGET`, et aucun
     littéral numérique de force restant dans les appels `force_mode` ;
   - aucun `movel` de contact ne porte `r=URSCRIPT_BLEND` (valeur transit) ;
   - aucune indexation chaînée `[i][j]`, aucune tranche `[0:3]` ;
   - aucune ligne du sondage 3 points émise (`probe_one`, `probe_surface_plane`,
     `NHAT_*`, `NOMINAL_P*`) tant que `PROBE_MODE` vaut `z1` ;
   - taille sous `URSCRIPT_MAX_BYTES` (message de `_validate_script_memory`).
3. Validation hors ligne : `python -m ur5_sim --check`, puis
   `python -m ur5_sim --visualize` pour l'inspection visuelle. En mode
   paramétrique, ces commandes n'ont de sens qu'après la phase 5.
4. Tests : `python -m unittest discover -s tests -p "test_*.py"`.
4b. Documentation : `README.md`, `ARCHITECTURE.md` et `CLAUDE.md` décrivent le
   code livré (table 9.2), et les liens résolvent.
5. URSim : surveiller la force Z sur un cycle complet, vérifier 6.0 +/- 0.5 N,
   aucun pic au-delà de 7 N, et relever la durée du cycle.
6. Pendant PolyScope : ouvrir le noeud Script, modifier la ligne
   `global FORCE_Z_TARGET`, relancer, confirmer que la force mesurée suit la
   nouvelle consigne (section 5.4, niveau 2).
7. Robot réel : même relevé via `get_tcp_force()`, plus contrôle visuel de
   l'uniformité du dépôt. Vérifier d'abord si `force_mode` régule sur le FT 300
   ou sur l'estimation interne (test d'écart type, section 5.6) : la tolérance
   +/- 0.5 N n'est atteignable que dans le premier cas.

---

## 12. Journal de révision

| Date | Modification |
| :--- | :--- |
| mai 2026 | Version initiale, cible `ur5_etalementv6.py` monolithique |
| 29 juillet 2026 | Réécriture sur le package `design/` ; lacune 1 marquée FAIT ; lacune 3 requalifiée (sondage 3 points désactivé, `NHAT` indisponible) ; ajout des métriques mesurées du script courant ; correction de la conclusion sur le blend et la fluidité (section 3.4) ; restriction du mode paramétrique aux cycles 1-3 ; ajout du point bloquant `ur5_sim` (section 5) ; squelettes URScript alignés sur `apply_correction(p_world)` et sur `FORCE_LIMIT_XY = 0.008` |
| 29 juillet 2026 (suite) | Ajout de la section 5 : force Z exposée en global URScript éditable sur le pendant, niveaux d'accès IHM, garde-fou `FORCE_Z_MIN_N` justifié par le plancher du FT 300, emplacements des littéraux à changer en repli. Ajout de la section 6 : statut du sondage 3 points, code conservé et intouchable dans ce plan, forme visée en option `PROBE_MODE` et conditions de réactivation. Renumérotation des sections suivantes ; phase 1 dédiée au global de force |
| 29 juillet 2026 (fin) | Section 0 rendue autonome pour une reprise à froid sans historique. Ajout de la phase 8 « Documentation » avec la table 9.2 (`README.md`, `ARCHITECTURE.md`, `CLAUDE.md`) et du critère de vérification associé. Ajout de la sous-section 9.1 : exécution pilotée par le plugin Superpowers, avec la compétence attendue à chaque moment |
