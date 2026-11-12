"""IE415 phone-laptop visual-inertial SLAM course project."""

import os
import tempfile
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "ie415-slam-matplotlib"),
)

__version__ = "1.0.0"
