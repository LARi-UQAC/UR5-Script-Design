"""
ur5_etalementv6.py — Point d entree compatible amont.
Toute la logique de conception de trajectoires reside maintenant dans design/.
"""
from design.app import main

if __name__ == "__main__":
    import sys
    sys.exit(main() or 0)
