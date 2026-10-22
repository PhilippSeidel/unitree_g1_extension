import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

from .kinematics import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, ArmIK  # noqa: F401  (re-exported)

log = logging.getLogger(__name__)

INCAR_TO_G1_AXES = np.eye(3)

_MOTORS = ("ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw")
ARM_MOTORS = {side: tuple(f"k{side.capitalize()}{m}" for m in _MOTORS) for side in ("right", "left")}

MIRROR_TWIST = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])

MAX_TRACKING_ERROR = 0.2  # rad


@dataclass
class ArmState:
    ee_pose: np.ndarray  # [x, y, z, qx, qy, qz, qw], metres, torso frame
    joint_pos: np.ndarray  # rad, 7 values, shoulder pitch -> wrist yaw
    joint_vel: np.ndarray  # rad/s, same order
    joint_effort: np.ndarray  # Nm, motor torque estimate, same order


class G1Robot:
    def __init__(
        self,
        config: UnitreeG1Config,
        max_lin_vel: float,
        max_ang_vel: float,
        control_dt: float = 0.01,
        urdf_path: str | None = None,  
        arm: str = "right",  
    ):
        if arm not in ARM_MOTORS:
            raise ValueError(f"arm must be 'right' or 'left', not {arm!r}")
        self.side = arm
        self.max_lin_vel = float(max_lin_vel)
        self.max_ang_vel = float(max_ang_vel)
        self.control_dt = control_dt
        self._motors = ARM_MOTORS[arm]
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
        self._g1.connect()  
        self._after_connect()

        self._ik = ArmIK(self._urdf_path or _lerobot_urdf(), self.side)
        self._ik.reset(self._read(self._g1.get_observation(), "q"))

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
        pass

    def _before_disconnect(self) -> None:
        pass

    def command_ee_velocity(self, twist_incar: Sequence[float]) -> bool:
        twist = np.asarray(twist_incar, dtype=float).reshape(-1)
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            return False
        twist_g1 = self._clamp(self.incar_to_g1(twist))
        if self.side == "left":
            twist_g1 = MIRROR_TWIST * twist_g1

        with self._lock:
            self._twist = twist_g1
        return True

    def stop_arm(self) -> None:
        with self._lock:
            self._twist = np.zeros(6)
            self._ik.hold()

    def arm_state(self) -> ArmState:
        obs = self._g1.get_observation()
        q = self._read(obs, "q")
        with self._lock:
            ee_pose = self._ik.pose(q)
        return ArmState(ee_pose, q, self._read(obs, "dq"), self._read(obs, "tau"))

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

    def _read(self, obs: dict, key: str) -> np.ndarray:
        return np.array([obs[f"{m}.{key}"] for m in self._motors], dtype=float)

    def _control_loop(self) -> None:
        while not self._stop.is_set():
            start = time.monotonic()
            q_measured = self._read(self._g1.get_observation(), "q")
            with self._lock:
                lag = np.max(np.abs(self._ik.q - q_measured))
                if lag > MAX_TRACKING_ERROR:
                    self._ik.reset(q_measured)
                    if start - self._t_warned > 1.0:
                        self._t_warned = start
                        log.warning("%s arm lags its target by %.2f rad (blocked?); holding where it is", self.side, lag)
                q = self._ik.step(self._twist, self.control_dt)
            self._g1.send_action({f"{m}.q": float(v) for m, v in zip(self._motors, q)})
            time.sleep(max(0.0, self.control_dt - (time.monotonic() - start)))


def _lerobot_urdf() -> str:
    from huggingface_hub import snapshot_download

    return os.path.join(snapshot_download("lerobot/unitree-g1-mujoco"), "assets", "g1_body29_hand14.urdf")