"""
design — Package de conception de trajectoires UR5.

Modules :
  params      : source unique de vérité pour toutes les constantes protocole
  geometry    : primitives SE(3) et conversion coordonnées (plate_to_robot, _abs_pose)
  trajectory  : générateurs de cycles (circular, linear, triangular)
  export      : génération URScript (.script) et PolyScope (.urp)
  live_ipc    : réception UDP et overlay live TCP depuis le simulateur
  app         : interface graphique matplotlib + point d'entrée main()
"""
