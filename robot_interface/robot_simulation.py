import os
import threading
import time
 
import numpy as np
from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
 
from .kinematics import RightArmIK
from .robot import ArmState, G1Robot

 
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
 
 
class G1RobotSim(G1Robot):
    def __init__(
        self,
        max_lin_vel: float = 0.25,
        max_ang_vel: float = 1.0,
        control_dt: float = 0.01,
        viewer: bool = True,  # MuJoCo window; False publishes the sim cameras over ZMQ instead
        urdf_path: str | None = None,
    ):
        super().__init__(max_lin_vel, max_ang_vel)
        self.control_dt = control_dt
        self._config = UnitreeG1Config(
            is_simulation=True,
            gravity_compensation=True,
            end_effector="dex3",  # fingers are not commanded
            sim_publish_images=not viewer,
        )
        self._urdf_path = urdf_path
        self._g1 = None
        self._ik = None
        self._twist = np.zeros(6)
        self._stop = threading.Event()
        self._thread = None
 
    def connect(self) -> None:
        self._g1 = UnitreeG1(self._config)
        self._g1.connect()  # starts the sim and waits for its first state
 
        self._ik = RightArmIK(self._urdf_path or _lerobot_urdf())
        self._ik.reset(self._measured_q())
 
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
 
    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._g1 is not None:
            self._g1.disconnect()
 
    def _apply_right_ee_velocity(self, twist_g1: np.ndarray) -> None:
        self._twist = twist_g1
 
    def _stop_right_arm(self) -> None:
        self._twist = np.zeros(6)
        self._ik.hold()
 
    def _read_right_arm_state(self) -> ArmState:
        q = self._measured_q()
        return ArmState(ee_pose=self._ik.pose(q), joint_pos=q)
 
    def _measured_q(self) -> np.ndarray:
        obs = self._g1.get_observation()
        return np.array([obs[k] for k in RIGHT_ARM_KEYS], dtype=float)
 
    def _control_loop(self) -> None:
        while not self._stop.is_set():
            start = time.monotonic()
            with self._lock:
                q = self._ik.step(self._twist, self.control_dt)
            self._g1.send_action(dict(zip(RIGHT_ARM_KEYS, q.tolist())))
            time.sleep(max(0.0, self.control_dt - (time.monotonic() - start)))
 
 
def _lerobot_urdf() -> str:
    """The URDF LeRobot's G1_29_ArmIK uses; already cached once the sim has been downloaded."""
    from huggingface_hub import snapshot_download
 
    return os.path.join(snapshot_download("lerobot/unitree-g1-mujoco"), "assets", "g1_body29_hand14.urdf")
