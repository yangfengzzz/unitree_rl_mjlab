# Repository Guidelines

## Environment
- Use the Conda environment `unitree_rl_mjlab` for Python commands.
- Prefer non-interactive commands in automation:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/list_envs.py
  conda run -n unitree_rl_mjlab python scripts/train.py Unitree-G1-Flat --env.scene.num-envs=4096
  ```
- For an interactive shell, run:
  ```bash
  conda activate unitree_rl_mjlab
  ```
- The documented setup uses Python 3.11 and installs the project with `pip install -e .`.
- System packages needed for C++ simulation/deployment include `libyaml-cpp-dev`, `libboost-all-dev`, `libeigen3-dev`, `libspdlog-dev`, and `libfmt-dev`.

## Project Layout
- `scripts/`: Python entry points for training, playback, task listing, terrain visualization, and motion conversion.
- `src/tasks/velocity/`: velocity-tracking tasks, configs, rewards, observations, terminations, and runner code.
- `src/tasks/tracking/`: motion-tracking tasks, configs, rewards, observations, commands, metrics, and runner code.
- `src/assets/`: robot assets, MuJoCo XMLs, meshes, constants, and sample motion files.
- `simulate/`: integrated Unitree MuJoCo simulator and C++ build files.
- `deploy/`: real/sim deployment code, robot-specific controllers, shared headers, and bundled third-party libraries.
- `doc/`: setup docs, licenses, and demonstration media.

## Common Commands
- List registered tasks:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/list_envs.py
  ```
- Train a velocity policy:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/train.py Unitree-G1-Flat --env.scene.num-envs=4096
  ```
- Train with multiple GPUs:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/train.py Unitree-G1-Flat --gpu-ids 0 1 --env.scene.num-envs=4096
  ```
- Convert a CSV motion file to NPZ:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/csv_to_npz.py --input-file src/assets/motions/g1/dance1_subject2.csv --output-name dance1_subject2.npz --input-fps 30 --output-fps 50 --robot g1
  ```
- Play a trained policy:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/play.py Unitree-G1-Flat --checkpoint_file=logs/rsl_rl/g1_velocity/<run>/model_<iteration>.pt
  ```
- Build simulator:
  ```bash
  cmake -S simulate -B simulate/build
  cmake --build simulate/build -j8
  ```
- Build a robot controller, for example G1:
  ```bash
  cmake -S deploy/robots/g1 -B deploy/robots/g1/build
  cmake --build deploy/robots/g1/build -j8
  ```

## Coding Style
- Follow the local Python style: two-space indentation, type hints where useful, dataclass-based configs, and concise docstrings for public helpers.
- Prefer existing mjlab registries, config classes, and task structure over introducing new framework patterns.
- Keep robot-specific constants and assets under the matching `src/assets/robots/unitree_*` directory.
- Keep task logic separated by domain: observations/rewards/terminations/commands belong in the relevant `mdp/` module, while RL runner customization belongs in `rl/`.
- Do not reformat large generated XML, mesh, CSV, ONNX, or third-party files unless the task explicitly requires it.

## Verification
- For Python changes, at minimum run the smallest relevant command through the Conda environment, such as:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/list_envs.py
  ```
- For task/config changes, run a low-cost smoke test with a small environment count before suggesting full training:
  ```bash
  conda run -n unitree_rl_mjlab python scripts/train.py <Task-ID> --env.scene.num-envs=1
  ```
- For C++ simulator or deployment changes, verify the affected CMake target builds.
- If GPU, MuJoCo display, joystick, robot network, or hardware access is unavailable, state exactly which verification was skipped and why.

## Training And Generated Artifacts
- Training outputs are written under `logs/rsl_rl/<experiment>/<date_time>/`.
- Checkpoints, exported `policy.onnx`, `policy.onnx.data`, videos, simulator builds, and deployment build directories are generated artifacts. Do not commit them unless explicitly requested.
- Large binary assets are already present in this repository; avoid adding new large binaries without confirming they are required.

## Deployment Safety
- Treat real-robot deployment as safety-critical.
- Never change real-robot network interfaces, controller startup behavior, robot mode assumptions, or policy paths casually.
- Use `--network=lo` for simulation deployment and the actual Ethernet interface, such as `enp5s0`, only for real-robot deployment.
- Before real-robot instructions, preserve the documented sequence: robot suspended, zero-torque mode, debug mode via `L2 + R2`, wired network configured to `192.168.123.222/24`, then controller launch.
- Prefer simulation deployment through `simulate/build/unitree_mujoco` before real hardware execution.
