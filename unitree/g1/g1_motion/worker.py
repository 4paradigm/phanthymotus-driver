"""G1 numerical process configuration; no SDK access."""
from pathlib import Path
from common.motion.worker import NumericalWorker as _Worker


class NumericalWorker(_Worker):
    def __init__(self, path, velocity=None):
        python = Path('/opt/g1-motion/bin/python')
        super().__init__(path, velocity, python_executable=python if python.is_file() else None, driver_dir=Path(__file__).resolve().parents[1],
            solver_module='g1_motion.kinematics', solver_class='G1IK',
            visualization_module='g1_motion.visualization')
