"""Black Panther X constants."""

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg
from src import SRC_PATH

##
# MJCF and assets.
##

BPX_XML: Path = (
  SRC_PATH / "assets" / "robots" / "black_panther_x" / "xmls" / "bpx.xml"
)
assert BPX_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, BPX_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(BPX_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

BPX_ACTUATOR_HIP_ROLL = BuiltinPositionActuatorCfg(
  target_names_expr=(".*hip_roll.*",),
  stiffness=20.0,
  damping=1.0,
  effort_limit=30,
  armature=0.01,
)
BPX_ACTUATOR_HIP_PITCH = BuiltinPositionActuatorCfg(
  target_names_expr=(".*hip_pitch.*",),
  stiffness=20.0,
  damping=1.0,
  effort_limit=30,
  armature=0.01,
)
BPX_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=(".*knee.*",),
  stiffness=40.0,
  damping=2.0,
  effort_limit=30,
  armature=0.02,
)

##
# Keyframes.
##

INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.42),
  joint_pos={
    ".*[fh]l_hip_roll_joint": -0.1,
    ".*[fh]r_hip_roll_joint": 0.1,
    ".*hip_pitch_joint": 0.9,
    ".*knee_joint": -1.8,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

_foot_regex = "^[fh][lr]_toe_link_collision_0$"

# This disables all collisions except the feet.
# Furthermore, feet self collisions are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(_foot_regex,),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  solimp=(0.9, 0.95, 0.023),
)

# This enables all collisions, excluding self collisions.
# Foot collisions are given custom condim, friction and solimp.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision.*",),
  condim={_foot_regex: 3, ".*_collision.*": 1},
  priority={_foot_regex: 1},
  friction={_foot_regex: (0.6,)},
  solimp={_foot_regex: (0.9, 0.95, 0.023)},
  contype=1,
  conaffinity=0,
)

##
# Final config.
##

BPX_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    BPX_ACTUATOR_HIP_ROLL,
    BPX_ACTUATOR_HIP_PITCH,
    BPX_ACTUATOR_KNEE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_bpx_robot_cfg() -> EntityCfg:
  """Get a fresh BPX robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=INIT_STATE,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=BPX_ARTICULATION,
  )


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_bpx_robot_cfg())

  viewer.launch(robot.spec.compile())
