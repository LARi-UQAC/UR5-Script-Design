# Plan d'Optimisation : Génération URScript par Fonctions Embarquées

## 1. Objectif
Réduire drastiquement le nombre de lignes du fichier `.script` tout en garantissant un maintien strict et continu de la force de 6 N perpendiculaire au plan durant TOUTE la trajectoire (Cycles 1-3 circulaires et Cycles 4-6 linéaires), en utilisant la puissance de calcul du contrôleur UR5.
Avant les mouvements dans le plan X-Y, le robot cherche la surface en Z en s'assurant que la descente au contact est sécurisée (force active avant impact) et en supprimant les instabilités liées au `blend radius` en mode force.

---

## 2. Diagnostic : lacunes du mode discret actuel

> **Fichier cible** : `c:\Martin Otis\OutilsLogiciels\UR5Script\ur5_etalementv6.py`
> **Fonction principale** : `_build_urscript_lines()` lignes 1032-1242.

Ces lacunes motivent à la fois les correctifs immédiats (mode discret, Section 4) et l'architecture paramétrique (Section 3).

### 2.1 Ce qui fonctionne
- `force_mode(...)` est activé une seule fois avant la boucle des waypoints (ligne 1206).
- `end_force_mode()` est appelé après le dernier waypoint (ligne 1224).
- Tous les `movel()` des waypoints (lignes 1214-1219) s'exécutent avec la force active : Z est contrôlé en force, XY en position. Correct pour l'application.

### 2.2 Lacune 1 — Descente au contact sans force (CRITIQUE, lignes 1204-1205)

```urscript
movel(apply_correction(pose_contact, ...), a=..., v=URSCRIPT_CONTACT_V*...)
# PUIS seulement :
force_mode(get_actual_tcp_pose(), [0,0,1,0,0,0], [0,0,-6.0,0,0,0], 2, [...])
```

Le robot descend en contrôle de **position** jusqu'à `ROBOT_Z_SURFACE` corrigé par le plan sondé. Si la surface réelle est au-dessus de la position calculée (erreur de sondage, gonflement de la crème), le robot pousse avec le couple maximal des joints — la force peut largement dépasser 6 N sans aucune limitation.

**Séquence actuelle (incorrecte) :**
```
movel(transit_in)        # au-dessus surface, pas de force
movel(pose_contact)      # descente — PAS DE FORCE  <-- risque
force_mode(...)          # force activée trop tard
loop waypoints           # force OK
end_force_mode()
movel(transit_out)
```

**Séquence corrigée :**
```
movel(transit_in)        # au-dessus surface, pas de force
force_mode(...)          # force activée AVANT la descente
movel(pose_contact_deep) # descente contrôlée : stoppe à 6 N  <-- sûr
loop waypoints           # force OK
end_force_mode()
movel(transit_out)
```

### 2.3 Lacune 2 — Blend radius incompatible avec force_mode (CRITIQUE, ligne 1219)

```python
f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR, r=URSCRIPT_BLEND'
#                                                              ^^^^^^^^^^^^^^
#                                                              r = 0.002 m
```

Lorsque le blend radius est actif (`r > 0`) pendant `force_mode`, le planificateur de trajectoire interpole entre deux `movel` consécutifs dans une sphère de blend. Pendant cette transition, le contrôleur de force n'est **pas garanti** de maintenir la force cible — des pics ou des pertes de contact peuvent survenir à chaque waypoint.

Conséquence directe : pour un cycle circulaire de 80 waypoints, il y a 80 transitions de blend où la force n'est pas fiable.

### 2.4 Lacune 3 — Frame de référence force_mode capturée une seule fois (MINEUR)

```urscript
force_mode(get_actual_tcp_pose(), ...)
```

`get_actual_tcp_pose()` capture la pose TCP au moment de l'activation. Si le TCP dérive légèrement en Z (compliance) entre le premier et le dernier waypoint, l'axe Z du frame de référence s'éloigne de la normale à la surface. Avec orientation fixe (`ROBOT_RX=π, RY=RZ=0`), l'impact est négligeable — mais passer la normale sondée (`NHAT`) comme frame fixe élimine ce biais (voir Section 3).

---

## 2b. Garantie de Contrôle de Force — Cycles 1-3 (Circulaires)

Les cycles 1-3 génèrent une trajectoire épicycloïdale : un déplacement porteur linéaire (4 passes × 50 mm) combiné à des orbites circulaires (R = 5 mm, 20 cercles par passe). Cette géométrie crée des conditions aggravantes pour les deux lacunes identifiées.

### Analyse spécifique au mode discret

| Paramètre | Valeur | Impact |
| :--- | :--- | :--- |
| Rayon des cercles | 5 mm | Circonférence : ~31.4 mm par cercle |
| Nombre de waypoints exportés | 80 (`URSCRIPT_N_WAYPOINTS_CIRCULAR`) | Espacement moyen ~34 mm sur ~2 713 mm de trajet total |
| Blend radius actuel | 0.002 m (2 mm) | Présent sur CHAQUE waypoint en mode force |

Sur une trajectoire courbe (épicycloïde), chaque entrée/sortie de blend provoque une interpolation géométrique entre deux arcs de cercle consécutifs. Le contrôleur de force n'étant pas garanti pendant la transition, la force peut fluctuer 80 fois par cycle circulaire.

**Garantie force cycles 1-3 — Mode Discret :**
1. Appliquer **Correctif 1** (force avant descente) : identique aux cycles linéaires.
2. Appliquer **Correctif 2** (supprimer `r=URSCRIPT_BLEND`) : critique pour les trajectoires courbes.
3. Augmenter `URSCRIPT_N_WAYPOINTS_CIRCULAR` de **80 à 200** pour compenser la perte de fluidité due à la suppression du blend. Avec 200 waypoints, l'espacement moyen passe de ~34 mm à ~14 mm, suffisant pour approximer les cercles de R = 5 mm (7 waypoints par cercle minimum).

**Garantie force cycles 1-3 — Mode Paramétrique :**
1. La boucle `while` dans `run_epicycloide_with_force` tourne avec `n_steps = 500` (ou ajustable) sans jamais appeler `end_force_mode()` pendant la trajectoire.
2. Aucun `r=` n'est passé aux `movel` internes — force maintenue en continu.
3. `force_mode` reste actif de la descente au contact jusqu'à `end_force_mode()` en fin de cycle.
4. La normale du plan sondé (`NHAT`) sert de frame de référence fixe — force toujours perpendiculaire au plan réel, indépendamment de la courbure de la trajectoire.

**En résumé : dans les deux modes, force_mode est activé AVANT la descente et reste actif sans interruption ni blend pendant toute la durée des cycles 1-3.**

---

## 2c. Garantie de Contrôle de Force — Cycles 4-6 (Linéaires)

Les cycles 4-6 génèrent un boustrophédon rectiligne : 13 passes horizontales alternées, réparties uniformément sur la surface (Y de MARGIN à SURFACE_H-MARGIN).

### Géométrie exportée

`get_waypoint_indices()` en mode `'linear'` retourne uniquement les **coins** (début et fin de chaque passe). Pour 13 passes × 50 points = 650 points totaux, le résultat est 26 waypoints : index 0, 49, 50, 99, 100, 149...

| Tronçon | Longueur | Blend actuel | Part du tronçon en blend |
| :--- | :--- | :--- | :--- |
| Passe droite (X_min ↔ X_max) | 42 mm | 2 mm | ~5 % — faible impact |
| Transition inter-passes (ΔY) | 3.5 mm | 2 mm | **~57 %** — critique |

La transition inter-passes (`ΔY = (50 - 2×4) / 12 ≈ 3.5 mm`) est une translation purement latérale en Y entre la fin d'une passe et le début de la suivante. Avec un blend de 2 mm sur 3.5 mm, le robot est en zone de blend pendant 57 % de ce mouvement — le contrôleur de force n'est **pas garanti** sur 12 transitions par cycle, soit 36 transitions sur les cycles 4-6 combinés.

Pendant les passes droites (42 mm, aucun waypoint intermédiaire), la force est maintenue en continu — ce tronçon n'est pas problématique.

### Garantie force cycles 4-6 — Mode Discret

1. Appliquer **Correctif 1** (force avant descente) : identique aux cycles 1-3.
2. Appliquer **Correctif 2** (supprimer `r=URSCRIPT_BLEND`) : élimine les 12 transitions en blend par cycle.
   - Sans blend : le robot marque un arrêt complet à chaque coin (ΔY = 3.5 mm se fait en position contrôlée, force Z maintenue à 6 N pendant le bref arrêt et la translation).
   - Augmentation de `LIN_N_POINTS_PER_SEGMENT` **non nécessaire** : les passes sont des droites, les coins sont les seuls waypoints exportés, la trajectoire reste exacte sans densification.
   - Impact temporel : environ 12 micro-arrêts par cycle. Avec `URSCRIPT_CONTACT_V = 0.05 m/s` et ΔY = 3.5 mm, chaque transition dure ~0.1 s — négligeable.

### Garantie force cycles 4-6 — Mode Paramétrique

1. `run_linear_with_force` reçoit la liste des 26 coins.
2. `force_mode` activé **avant** la descente au contact — élimine la Lacune 1.
3. La boucle `while` itère sur les coins avec `movel` sans `r=` — aucun blend pendant les transitions.
4. `force_mode` reste actif de la descente jusqu'à `end_force_mode()` en fin de cycle — 13 passes + 12 transitions couvertes en continu.

**En résumé : dans les deux modes, force_mode couvre les 13 passes et les 12 transitions ΔY des cycles 4-6 sans interruption. La suppression du blend élimine le seul point de vulnérabilité (transitions courtes à 57 % en blend).**

---

## 3. Architecture de la Solution Optimale (Mode Paramétrique)

Passage d'une approche **"Point-par-Point"** (Python calcule tout) à une approche **"Paramétrique"** (Python envoie les consignes, le robot calcule).

### 3.1 Fonctions URScript Dédiées (Côté Robot)
Ajouter dans le préambule du script généré des fonctions de haut niveau :

1. **`execute_circular_pass(p_start, p_end, n_circles, radius, f_z, depth)`** :
   - Active `force_mode` avec la normale du plan sondé (`NHAT`) comme frame de référence — élimine la Lacune 3.
   - Descend avec `movel` vers `p_start_deep` (Z = surface − `depth`) **après** activation de `force_mode` — élimine la Lacune 1.
   - Calcule l'interpolation épicycloïdale en boucle `while` interne.
   - Applique `apply_correction` dynamiquement pour suivre le plan incliné.
   - Utilise `r=0` (ou `r=0.0005`) dans la boucle — élimine la Lacune 2.

2. **`execute_linear_trajectory(p_list, speed, f_z, depth)`** :
   - Même séquence de descente sécurisée avant `force_mode`.
   - Prend la liste des coins du boustrophédon.
   - Active `force_mode` une seule fois pour toute la séquence de droites.
   - Parcourt les coins avec `movel` sans blend radius.

### 3.2 Avantages pour le Contrôle de Force
- **Zéro Interruption** : L'activation/désactivation de la force ne se fait qu'aux transitions transit/contact. Durant tout l'étalement, le contrôleur 500 Hz du robot travaille sur un flux de mouvement continu.
- **Vecteur Perpendiculaire** : En mode paramétrique, la normale du plan (`NHAT`) est passée directement à `force_mode` — les 6 N sont toujours exactement perpendiculaires au plan calibré, et non simplement selon l'axe Z du robot.
- **Stabilité des droites** : Pour les cycles 4-6, l'appel groupé évite les micro-arrêts aux coins qui pourraient causer des pics de force.

---

## 4. Modifications de l'Interface Python (`ur5_etalementv6.py`)

### 4.1 Paramètres Globaux (lignes 98-118)

Ajouter `FORCE_CONTACT_DEPTH` dans la section de contrôle de force :

```python
# --- Contrôle de force en Z ---
FORCE_Z_TARGET       = 6.0    # N   — force cible (ISO : 6.0 +/- 0.5 N)
FORCE_LIMIT_XY       = 0.002  # m   — déviation max en XY
FORCE_LIMIT_Z        = 0.040  # m   — déviation max en Z (compliance verticale)
FORCE_LIMIT_ROT      = 0.35   # rad — déviation max en orientation
FORCE_CONTACT_DEPTH  = 0.005  # m   — profondeur sous nominal pour descente forcée
```

### 4.2 Nouvel Élément UI (Mode d'Exportation)

Ajouter un groupe `RadioButtons` sous les curseurs actuels :
- **Label** : `"Mode d'exportation"`
- **Option 1** : `"Points Discrétisés"` — comportement actuel, compatibilité maximale, avec correctifs force appliqués.
- **Option 2** : `"Fonctions Robot (Optimisé & Force Continue)"` — mode recommandé, script compact.

### 4.3 Correctifs du Mode Discret (`_build_urscript_lines`, lignes 1200-1219)

Ces correctifs s'appliquent au mode discret ET servent de base aux fonctions paramétriques.

**Correctif 1 — Descente contrôlée en force** (remplace lignes 1204-1209) :

```python
# Z profond pour que le contrôleur de force stoppe avant d'atteindre la cible
z_contact_deep_m = ROBOT_Z_SURFACE - FORCE_CONTACT_DEPTH
pose_contact_deep = _fmt_pose([px0, py0, z_contact_deep_m, ROBOT_RX, ROBOT_RY, ROBOT_RZ])

lines.append(f'  movel(apply_correction({pose_transit_in}, {px0_m:.6f}, {py0_m:.6f}), '
             f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=URSCRIPT_TRANSIT_V*SPEED_FACTOR)')
# force_mode activé AVANT la descente au contact
lines.append(f'  force_mode(get_actual_tcp_pose(), [0, 0, 1, 0, 0, 0], '
             f'[0, 0, {-FORCE_Z_TARGET:.1f}, 0, 0, 0], 2, '
             f'[{FORCE_LIMIT_XY}, {FORCE_LIMIT_XY}, {FORCE_LIMIT_Z}, '
             f'{FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}])')
lines.append(f'  movel(apply_correction({pose_contact_deep}, {px0_m:.6f}, {py0_m:.6f}), '
             f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=URSCRIPT_CONTACT_V*SPEED_FACTOR)')
# Retirer l'ancien movel(pose_contact) et l'ancien appel force_mode déplacé ici
```

**Correctif 2 — Supprimer le blend radius en mode force** (ligne 1218-1219) :

```python
# Avant :
lines.append(f'  movel(apply_correction({pose_wp}, {px_m:.6f}, {py_m:.6f}), '
             f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR, r=URSCRIPT_BLEND)')

# Après :
lines.append(f'  movel(apply_correction({pose_wp}, {px_m:.6f}, {py_m:.6f}), '
             f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR)')
```

> **Conséquence** : sans blend, le robot ralentit à chaque waypoint. Compenser en augmentant `URSCRIPT_N_WAYPOINTS_CIRCULAR` de 80 à ~200 points (cycles circulaires) et `LIN_N_POINTS_PER_SEGMENT` pour les cycles linéaires.

### 4.4 Logique de Génération (Mode Paramétrique)

Si le mode `Optimisé` est sélectionné dans l'UI :
- Cycles 1-3 : un seul appel `run_epicycloide_with_force(p_start, p_end, n_circles, radius, FORCE_Z_TARGET, FORCE_CONTACT_DEPTH)`.
- Cycles 4-6 : un seul appel `run_linear_with_force(LIST_OF_CORNERS, speed, FORCE_Z_TARGET, FORCE_CONTACT_DEPTH)`.

---

## 5. Comparaison des Modes

| Caractéristique | Mode Discrétisé + Correctifs | Mode Paramétrique |
| :--- | :--- | :--- |
| **Taille du script** | ~500-1000 lignes | ~100-200 lignes |
| **Complexité Python** | Élevée (interpolation 2D) | Faible (envoi de paramètres) |
| **Contrôle de Force** | Continu (après correctifs) | Continu (natif) |
| **Descente contact** | Force active avant impact | Force active avant impact |
| **Blend radius** | Supprimé | Non applicable (boucle while) |
| **Sécurité contact** | Haute | Haute |
| **Maintenance** | Lisible | Très lisible (structurel) |

---

## 6. Étapes d'Implémentation

1. **Phase 1 — Correctifs Mode Discret** : Appliquer correctifs 1 et 2 (Sections 4.1 et 4.3). Valider immédiatement sur URSim.
2. **Phase 2 — URScript** : Développer et tester `apply_epicycloid` et `run_linear_with_force` sur URSim — valider la syntaxe des boucles et des fonctions trigonométriques.
3. **Phase 3 — Python** : Ajouter la variable d'état `export_mode` dans la classe de visualisation.
4. **Phase 4 — UI** : Intégrer le sélecteur `matplotlib.widgets` pour basculer entre les modes.
5. **Phase 5 — Validation** : Comparer les sorties via `ur5_sim --check` pour s'assurer que l'enveloppe géométrique reste identique entre les deux modes.

---

## 7. Concepts URScript (Exemples)

### 7.1 Mode Épicycloïde Sécurisé (Cycles 1-3)

```urscript
def run_epicycloide_with_force(p_start, p_end, n_total_circles, radius, f_z, depth):
  # 1. Transit au-dessus de la surface
  movel(apply_correction(p_start_transit, p_start[0], p_start[1]), a=1.2, v=0.3)

  # 2. CORRECTIF 1 : force_mode AVANT la descente
  force_mode(get_actual_tcp_pose(), [0, 0, 1, 0, 0, 0], [0, 0, -f_z, 0, 0, 0], 2,
             [0.002, 0.002, 0.040, 0.35, 0.35, 0.35])

  # 3. CORRECTIF 1 : descente profonde — stoppe à f_z N avant d'atteindre p_start_deep
  movel(apply_correction(p_start_deep, p_start[0], p_start[1]), a=0.5, v=0.05)

  # 4. Boucle épicycloïdale — CORRECTIF 2 : r=0 (pas de blend en force_mode)
  local i = 0
  local n_steps = 500
  while i <= n_steps:
    local alpha = i / n_steps
    local p_carrier = interpolate_pose(p_start, p_end, alpha)
    local theta = alpha * 2 * 3.14159 * n_total_circles
    local offset = p[cos(theta)*radius, sin(theta)*radius, 0, 0, 0, 0]
    local p_target = pose_trans(p_carrier, offset)
    movel(apply_correction(p_target, p_target[0], p_target[1]), a=1.2, v=0.05)
    i = i + 1
  end

  end_force_mode()
end
```

### 7.2 Mode Linéaire Sécurisé (Cycles 4-6)

```urscript
def run_linear_with_force(p_corners, n_corners, speed, f_z, depth):
  # Transit vers le premier coin
  movel(apply_correction(p_transit_in, p_corners[0][0], p_corners[0][1]), a=1.2, v=0.3)

  # CORRECTIF 1 : force_mode AVANT la descente
  force_mode(get_actual_tcp_pose(), [0, 0, 1, 0, 0, 0], [0, 0, -f_z, 0, 0, 0], 2,
             [0.002, 0.002, 0.040, 0.35, 0.35, 0.35])

  # Descente contrôlée
  movel(apply_correction(p_contact_deep, p_corners[0][0], p_corners[0][1]), a=0.5, v=0.05)

  # CORRECTIF 2 : parcours linéaire sans blend radius
  local i = 0
  while i < n_corners:
    movel(apply_correction(p_corners[i], p_corners[i][0], p_corners[i][1]), a=1.2, v=speed)
    i = i + 1
  end

  end_force_mode()
end
```

---

## 8. Vérification

1. Régénérer le script : `python ur5_etalementv6.py --export`
2. Ouvrir `etalement.script` et vérifier que :
   - `force_mode(...)` apparaît **avant** chaque `movel(pose_contact_deep, ...)`
   - Aucun `movel(...)` dans les boucles waypoints ne contient `r=`
   - `end_force_mode()` apparaît **après** le dernier waypoint de chaque cycle
3. Simuler sur URSim et surveiller le canal de force Z :
   - Force atteint 6 N pendant la descente et reste stable (< 0.5 N d'écart)
   - Pas de pic > 7 N à aucun waypoint
4. Sur robot réel : surveiller `get_tcp_force()` dans un thread watcher pendant l'étalement pour valider la stabilité continue de la force.

---

*Note : Ce plan permet de diviser la taille du fichier exporté par 5 à 10 tout en corrigeant les trois lacunes de contrôle de force identifiées. Les correctifs du mode discret (Section 4.3) peuvent être appliqués immédiatement sans attendre le mode paramétrique.*
