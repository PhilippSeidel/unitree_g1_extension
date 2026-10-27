# Unitree G1 extension for INCAR

INCAR extension for the Unitree G1 humanoid. It provides:

- **Arm teleoperation**: a robot interface that turns the INCAR end-effector velocity commands into joint targets for one 7-DoF arm (right or left) of the G1, using closed-loop differential IK. It works with LeRobot's MuJoCo simulation and with the real robot.

- **Cameras**: two INCAR camera types, `g1_zmq` (real robot) and `g1_sim` (MuJoCo simulation), that receive JPEG frames over ZMQ.

## Install

Requires an INCAR venv made by the INCAR installer, Ubuntu 22.04 or Ubuntu 24.04. (Unofficially also Debian 13)

```bash
git clone https://github.com/PhilippSeidel/unitree_g1_extension.git
cd unitree_g1_extension
bash install.sh
```


It installs into the INCAR venv:

- [LeRobot](https://github.com/huggingface/lerobot) (a pinned commit of `main`) and this extension
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python), as an editable checkout in `<venv>/src/unitree-sdk2py`, with the CycloneDDS 0.10 library it needs, built into `<venv>/cyclonedds`
- Pinocchio built from source with CasADi support, needed by LeRobot's G1 arm IK. This takes a long time and changes numpy to 2.3.x.
- MuJoCo and the other packages LeRobot's G1 code and its simulation import

Keep `<venv>/cyclonedds` and `<venv>/src/unitree-sdk2py`: the installed packages load files from there.

LeRobot's dependency pins conflict with INCAR's: it pins an older torch and the PyPI `draccus`, which replaces INCAR's `draccus` fork. The script therefore installs LeRobot and this extension without their dependencies. When reinstalling either by hand, use `pip install --no-deps`. Otherwise pip downgrades torch and breaks INCAR. `pip check` keeps reporting LeRobot's unmet pins; that is expected.

## Cameras

Add a `g1_zmq` or `g1_sim` camera to the `cameras` in your `workspace_config.json`:

```json
"cameras": {
    "ego_view_real": {
        "type": "g1_zmq",
        "camera_name": "head_camera"
    },
    "ego_view_sim": {
        "type": "g1_sim",
        "camera_name": "head_camera"
    }
}
```

`camera_name` must match the name of a camera in the image stream. 
## Arm teleoperation

### Start

#### Robot side (real robot only)

Hang the robot on the gantry, feet off the ground, with the arms free to move. Then start the LeRobot G1 server on the robot:
```bash
python ~/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py           # arm control only
python ~/lerobot/src/lerobot/robots/unitree_g1/run_g1_server.py --camera  # arm control and head camera stream for g1_zmq
```
#### INCAR side

1. Source the venv.
2. Start the INCAR app as usual. Then start the G1 interface:
   ```bash
   python -m unitree_g1_extension.robot_interface.run --robot sim               # MuJoCo simulation, opens the viewer
   python -m unitree_g1_extension.robot_interface.run --robot sim --no-viewer   # MuJoCo simulation, streams the cameras over ZMQ
   python -m unitree_g1_extension.robot_interface.run --robot real              # real robot
   ```
   With `--no-viewer` on a machine without a display, set `MUJOCO_GL=egl`.

| Option | Values | Default | Description |
|---|---|---|---|
| `--robot` | `sim`, `real` | `sim` | Simulation or real robot |
| `--arm` | `right`, `left` | `right` | Arm to teleoperate |
| `--no-viewer` | flag | off | Simulation only: don't open the MuJoCo window, publish the sim cameras over ZMQ instead (needed for `g1_sim` cameras) |


### Behavior

- The interface subscribes to `right.commands.arm.ee.velocity`, a 6D twist `[vx vy vz wx wy wz]` in the torso frame. With `--arm left`, the left arm follows the right controller, mirrored about the robot's sagittal plane.
- Velocities are clamped to 0.25 m/s and 1.0 rad/s in simulation and to 0.1 m/s and 0.5 rad/s on the real robot.
- If no command arrives for 0.2 s, the arm stops and holds its position.
- If the arm lags its target by more than 0.2 rad (e.g. because it is blocked), the target is reset to the measured position.
- The arm state (`ee_pose`, `joint_pos`, `joint_vel`, `joint_effort`) is published to INCAR as `right_arm` or `left_arm` every 10 ms.
