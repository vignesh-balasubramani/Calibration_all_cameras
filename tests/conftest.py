"""Make the flat `src/` and `utils/` script directories importable from the tests."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in ("src", "utils"):
    sys.path.insert(0, str(ROOT / sub))
