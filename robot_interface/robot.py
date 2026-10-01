import threading
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

import numpy as np

RIGHT_ARM_JOINT_NAMES = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
 
 
@dataclass
class ArmState: 

    ee_pose: np.ndarray  # [x, y, z, qx, qy, qz, qw], metres
    joint_pos: np.ndarray  # 7 values in rad, ordered as RIGHT_ARM_JOINT_NAMES
 
 
class G1Robot(ABC):

    def __init__(self, max_lin_vel: float = 0.25, max_ang_vel: float = 1.0):
        self.max_lin_vel = float(max_lin_vel) 
        self.max_ang_vel = float(max_ang_vel)
        self._lock = threading.Lock()
 
    def command_right_ee_velocity(self, twist_incar: Sequence[float]) -> None:
        """Apply one `right.commands.arm.ee.velocity` message.
 
        `twist_incar` is [vx, vy, vz, wx, wy, wz] in m/s and rad/s, INCAR frame.
        """
        twist = np.asarray(twist_incar, dtype=float).reshape(-1)
        twist_g1 = self._clamp(self.incar_to_g1(twist))

        with self._lock:
            self._apply_right_ee_velocity(twist_g1)
 
    def stop_right_arm(self) -> None:
        with self._lock:
            self._stop_right_arm()
 
    def right_arm_state(self) -> ArmState:
        with self._lock:
            return self._read_right_arm_state()

    def incar_to_g1(self, twist: np.ndarray) -> np.ndarray:
        return np.concatenate([np.eye(3) @ twist[:3], -np.eye(3) @ twist[3:]])
 
    @abstractmethod
    def connect(self) -> None:
        """Open the connection to the robot or simulator and start holding the arm."""
        pass
 
    @abstractmethod
    def close(self) -> None:
        """Release the robot or simulator. Called once on shutdown."""
        pass
 
    @abstractmethod
    def _apply_right_ee_velocity(self, twist_g1: np.ndarray) -> None:
        pass
 
    @abstractmethod
    def _stop_right_arm(self) -> None:
        pass
 
    @abstractmethod
    def _read_right_arm_state(self) -> ArmState:
        pass
 
 
    def _clamp(self, twist: np.ndarray) -> np.ndarray:
        out = twist.copy()
        for sl, limit in ((slice(0, 3), self.max_lin_vel), (slice(3, 6), self.max_ang_vel)):
            norm = math.sqrt(float(out[sl] @ out[sl]))
            if norm > limit:
                out[sl] *= limit / norm
        return out
 
