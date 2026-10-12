import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

from .kinematics import RIGHT_ARM_JOINT_NAMES, RightArmIK  # noqa: F401  (re-exported)

log = logging.getLogger(__name__)

INCAR_TO_G1_AXES = np.eye(3)

RIGHT_ARM_KEYS = tuple(
    f"{name}.q"
    for name in (
        "kRightShoulderPitch",
        "kRightShoulderRoll",
        "kRightShoulderYaw",
        "kRightElbow",
        "kRightWristRoll",
        "kRightWristPitch",
        "kRightWristYaw",
    )
)

MAX_TRACKING_ERROR = 0.2  # rad

 
 
@dataclass
class ArmState: 

    ee_pose: np.ndarray  # [x, y, z, qx, qy, qz, qw], metres
    joint_pos: np.ndarray  # 7 values in rad, ordered as RIGHT_ARM_JOINT_NAMES
 
 
class G1Robot():

    def __init__(
        self,
        config: UnitreeG1Config,
        max_lin_vel: float,
        max_ang_vel: float,
        control_dt: float = 0.01,
        urdf_path: str | None = None,  # default: LeRobot's arm-IK URDF from the hub repo
    ):
        self.max_lin_vel = float(max_lin_vel)
        self.max_ang_vel = float(max_ang_vel)
        self.control_dt = control_dt
        self._config = config
        self._urdf_path = urdf_path
        self._g1 = None
        self._ik = None
        self._twist = np.zeros(6)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._t_warned = 0.0

    def connect(self) -> None:
        self._g1 = UnitreeG1(self._config)
        self._g1.connect()  # waits for the first state; sends nothing yet
        self._after_connect()

        self._ik = RightArmIK(self._urdf_path or _lerobot_urdf())
        self._ik.reset(self._measured_q())

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._g1 is not None:
            self._before_disconnect()
            self._g1.disconnect()

    def _after_connect(self) -> None:
        """Hook: runs once LeRobot is connected, before the arm is commanded."""

    def _before_disconnect(self) -> None:
        """Hook: runs after the last arm command, before LeRobot disconnects."""

 
    def command_right_ee_velocity(self, twist_incar: Sequence[float]) -> bool:
        """Apply one `right.commands.arm.ee.velocity` message.

        `twist_incar` is [vx, vy, vz, wx, wy, wz] in m/s and rad/s, INCAR frame.
        Returns False (and does nothing) if the message is not 6 finite numbers.
        """
        twist = np.asarray(twist_incar, dtype=float).reshape(-1)
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            return False
        twist_g1 = self._clamp(self.incar_to_g1(twist))

        with self._lock:
            self._twist = twist_g1
        return True

    def stop_right_arm(self) -> None:
        with self._lock:
            self._twist = np.zeros(6)
            self._ik.hold()

    def right_arm_state(self) -> ArmState:
        with self._lock:
            q = self._measured_q()
            return ArmState(ee_pose=self._ik.pose(q), joint_pos=q)

    def incar_to_g1(self, twist: np.ndarray) -> np.ndarray:
        M = INCAR_TO_G1_AXES
        return np.concatenate([M @ twist[:3], -M @ twist[3:]])

 
    def _clamp(self, twist: np.ndarray) -> np.ndarray:
        out = twist.copy()
        for sl, limit in ((slice(0, 3), self.max_lin_vel), (slice(3, 6), self.max_ang_vel)):
            norm = math.sqrt(float(out[sl] @ out[sl]))
            if norm > limit:
                out[sl] *= limit / norm
        return out

    def _measured_q(self) -> np.ndarray:
        obs = self._g1.get_observation()
        return np.array([obs[k] for k in RIGHT_ARM_KEYS], dtype=float)

    def _control_loop(self) -> None:
        while not self._stop.is_set():
            start = time.monotonic()
            q_measured = self._measured_q()
            with self._lock:
                lag = np.max(np.abs(self._ik.q - q_measured))
                if lag > MAX_TRACKING_ERROR:
                    self._ik.reset(q_measured)
                    if start - self._t_warned > 1.0:
                        self._t_warned = start
                        log.warning("right arm lags its target by %.2f rad (blocked?); holding where it is", lag)
                q = self._ik.step(self._twist, self.control_dt)
            self._g1.send_action(dict(zip(RIGHT_ARM_KEYS, q.tolist())))
            time.sleep(max(0.0, self.control_dt - (time.monotonic() - start)))


def _lerobot_urdf() -> str:
    from huggingface_hub import snapshot_download
    return os.path.join(snapshot_download("lerobot/unitree-g1-mujoco"), "assets", "g1_body29_hand14.urdf")
