# PiPER motion sources

- `models/piper.urdf`: unmodified `src/piper_description/urdf/piper_description.urdf`
  from <https://github.com/agilexrobotics/piper_ros/tree/017ffefa64511bc6325bd77ddc4e16065c152051>,
  MIT, Copyright (c) 2024 RosenYin. License retained in `models/LICENSE`.
  SHA256: `22c15f95e50003b01713bae158638f7d19cdb71b1bd1095289865b835067002b`.
- `common/teleop_contract.py` and its contract tests are reused from PhanthyMotus
  Driver PR #329, commit `77a9fa3aec4054eca6ba8a1984035b636ef88041`.
- `common/motion/worker.py` is adapted from Driver PR #330,
  commit `c5de68f8a81f6489c52165abe73e39356c3b9e9e`.
  PiPER removes unused render/return RPCs, the visualization module argument,
  and the unused `solve_frame` wrapper. The load/self-test/solve protocol,
  request deadlines and process lifecycle remain.

PiPER FK/IK uses the existing NumPy dependency and does not load visual meshes.
The URDF is a kinematic model, not evidence that a particular arm, TCP or paired
installation has been calibrated. No mesh collision checking is claimed.
