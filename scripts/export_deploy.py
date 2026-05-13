import sys
import os

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", ".."))

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.entity.entity import Entity
from mjlab.utils.string import resolve_expr


def export_deploy_params(env_cfg: ManagerBasedRlEnvCfg | None):
    if env_cfg is None:
        raise RuntimeError(f"env_cfg is None")

    robot_cfg = env_cfg.scene.entities["robot"]
    entity = Entity(robot_cfg)
    joint_names = entity.joint_names

    # ------------------------------------------------------------------
    # 1. Joint names
    # ------------------------------------------------------------------
    print("=" * 70)
    print(f"Robot joint names ({entity.num_joints} total):")
    for i, name in enumerate(joint_names):
        print(f"  [{i:2d}] {name}")
    print()

    # ------------------------------------------------------------------
    # 2. PD parameters (stiffness, damping, effort limit)
    # ------------------------------------------------------------------
    joint_to_params: dict[str, tuple[float | None, float | None, float | None]] = {}
    for actuator in entity.actuators:
        cfg = actuator.cfg
        kp = getattr(cfg, "stiffness", None)
        kd = getattr(cfg, "damping", None)
        effort_limit = getattr(cfg, "effort_limit", None)
        for name in actuator.target_names:
            joint_to_params[name] = (kp, kd, effort_limit)

    print("=" * 70)
    print("Actuator PD parameters (in joint name order):")
    print(f"{'Joint Name':<30s} {'Stiffness (kp)':>15s} {'Damping (kd)':>15s} {'Effort Limit':>15s}")
    print("-" * 75)

    for name in joint_names:
        params = joint_to_params.get(name)
        if params is None:
            kp_str = kd_str = effort_str = "N/A"
        else:
            kp, kd, effort_limit = params
            kp_str = f"{kp:.4f}" if kp is not None else "N/A"
            kd_str = f"{kd:.4f}" if kd is not None else "N/A"
            effort_str = f"{effort_limit:.4f}" if effort_limit is not None else "N/A"
        print(f"  {name:<30s} {kp_str:>15s} {kd_str:>15s} {effort_str:>15s}")
    print()

    # ------------------------------------------------------------------
    # 3. Default joint positions
    # ------------------------------------------------------------------
    init_joint_pos = entity.cfg.init_state.joint_pos
    print("=" * 70)
    print("Default joint positions (in joint name order):")
    print(f"{'Joint Name':<30s} {'Default Pos':>15s}")
    print("-" * 45)

    if init_joint_pos is None:
        for name in joint_names:
            print(f"  {name:<30s} {'(from keyframe)':>15s}")
    else:
        default_pos_vals = resolve_expr(init_joint_pos, joint_names, 0.0)
        for name, val in zip(joint_names, default_pos_vals):
            print(f"  {name:<30s} {val:>15.6f}")
    print()

    # ------------------------------------------------------------------
    # 4. Action scale & offset
    # ------------------------------------------------------------------
    try:
        joint_pos_action = env_cfg.actions["joint_pos"]
        action_scale = joint_pos_action.scale
        action_offset = joint_pos_action.offset
        use_default_offset = getattr(joint_pos_action, "use_default_offset", False)
    except (KeyError, AttributeError):
        action_scale = None
        action_offset = None
        use_default_offset = False

    print("=" * 70)
    print("Action scale (in joint name order):")
    print(f"{'Joint Name':<30s} {'Scale':>15s}")
    print("-" * 45)

    if action_scale is None:
        for name in joint_names:
            print(f"  {name:<30s} {'N/A':>15s}")
    elif isinstance(action_scale, dict):
        scale_vals = resolve_expr(action_scale, joint_names, 0.25)
        for name, val in zip(joint_names, scale_vals):
            print(f"  {name:<30s} {val:>15.6f}")
    else:
        for name in joint_names:
            print(f"  {name:<30s} {action_scale:>15.6f}")
    print()

    print("=" * 70)
    print("Action offset (in joint name order):")
    if use_default_offset:
        print("  (use_default_offset=True, effective offset = default_joint_pos above)")
    print(f"{'Joint Name':<30s} {'Offset':>15s}")
    print("-" * 45)

    if action_offset is None:
        for name in joint_names:
            print(f"  {name:<30s} {'N/A':>15s}")
    elif isinstance(action_offset, dict):
        offset_vals = resolve_expr(action_offset, joint_names, 0.0)
        for name, val in zip(joint_names, offset_vals):
            print(f"  {name:<30s} {val:>15.6f}")
    else:
        for name in joint_names:
            print(f"  {name:<30s} {action_offset:>15.6f}")
    print()


    # ------------------------------------------------------------------
    # 5. Observations
    # ------------------------------------------------------------------
    for group_name, group_cfg in env_cfg.observations.items():
        group_history = group_cfg.history_length
        print("=" * 70)
        print(f"Observation group: {group_name}")
        if group_history is not None:
            print(f"  (group-level history_length={group_history} overrides term-level)")
        print(f"  concatenate_terms={group_cfg.concatenate_terms}")
        print(f"  enable_corruption={group_cfg.enable_corruption}")
        print()
        hdr = "  {:<30s} {:>12s} {:>18s} {:>12s}  {:>20s}".format(
            "Term Name", "Scale", "Clip", "History Len", "Noise")
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))

        for term_name, term_cfg in group_cfg.terms.items():
            # Scale
            s = term_cfg.scale
            if s is None:
                scale_str = "None"
            elif isinstance(s, (int, float)):
                scale_str = f"{s:.6g}"
            elif isinstance(s, (tuple, list)):
                scale_str = f"({', '.join(f'{v:.4g}' for v in s[:4])}{', ...' if len(s) > 4 else ''})"
            else:
                scale_str = str(type(s).__name__)

            # Clip
            c = term_cfg.clip
            if c is None:
                clip_str = "None"
            else:
                clip_str = f"({c[0]:.4g}, {c[1]:.4g})"

            # History length (term-level, overridden by group if set)
            hl = term_cfg.history_length
            if group_history is not None:
                hl_str = f"{hl} -> {group_history}"
            else:
                hl_str = str(1)

            # Noise
            noise = term_cfg.noise
            if noise is None:
                noise_str = "None"
            else:
                cls_name = type(noise).__name__
                if hasattr(noise, "n_min") and hasattr(noise, "n_max"):
                    noise_str = f"{cls_name}(n_min={noise.n_min}, n_max={noise.n_max})"
                elif hasattr(noise, "mean") and hasattr(noise, "std"):
                    noise_str = f"{cls_name}(mean={noise.mean}, std={noise.std})"
                elif hasattr(noise, "bias"):
                    noise_str = f"{cls_name}(bias={noise.bias})"
                else:
                    noise_str = cls_name

            print(f"  {term_name:<30s} {scale_str:>12s} {clip_str:>18s} {hl_str:>12s}  {noise_str:>20s}")
        print()


if __name__ == "__main__":
    import src.tasks
    from src.tasks.velocity.config.bpx.env_cfgs import bpx_rough_env_cfg
    env_cfg = bpx_rough_env_cfg(play=True)
    export_deploy_params(env_cfg)
