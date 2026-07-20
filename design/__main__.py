"""Point d'entrée pour `python -m design`."""
from design.app import main
import sys
sys.exit(main() or 0)
