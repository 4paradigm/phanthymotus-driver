"""G1 numerical process configuration; no SDK access."""
from pathlib import Path
from common.motion.worker import NumericalWorker as _Worker


class NumericalWorker(_Worker):
    def __init__(self, path, velocity=None):
        super().__init__(path, velocity, driver_dir=Path(__file__).resolve().parents[1],
            solver_module='motion.kinematics', solver_class='IK',
            visualization_module='motion.visualization')
