import logging
import time

import numpy as np
from lerobot.robots.unitree_g1 import UnitreeG1Config

from .robot import G1Robot

log = logging.getLogger(__name__)

RAMP_TIME = 2.0  # s, stiffness ramp at connect and close
SETTLE_TIME = 1.5  # s of damping only after the ramp-out, so the arms come to rest before going passive

CHECKLIST = """
Check:
  - the robot hangs on the gantry, feet off the ground, arms free to move
  - run_g1_server.py is running on the robot
Type 'yes' to stiffen all joints over {:.0f} s and start teleoperation: """


class G1RobotReal(G1Robot):
    def __init__(
        self,
        max_lin_vel: float = 0.1,
        max_ang_vel: float = 0.5,
        robot_ip: str = "192.168.123.164",
        **kwargs,  # control_dt, urdf_path
    ):
        config = UnitreeG1Config(
            is_simulation=False,
            robot_ip=robot_ip,
            # Feedforward from LeRobot's hand14 URDF, i.e. assuming Dex3 hands; see sim.py.
            gravity_compensation=True,
        )
        super().__init__(config, max_lin_vel, max_ang_vel, **kwargs)

    def connect(self) -> None:
        if input(CHECKLIST.format(RAMP_TIME)).strip().lower() != "yes":
            raise SystemExit("aborted, robot not touched")
        super().connect()

    def _after_connect(self) -> None:
        log.info("stiffening all joints over %.0f s", RAMP_TIME)
        self._ramp_stiffness(0.0, 1.0)

    def _before_disconnect(self) -> None:
        log.info("releasing all joints over %.0f s", RAMP_TIME)
        self._ramp_stiffness(1.0, 0.0, then_hold=SETTLE_TIME)

    def _ramp_stiffness(self, start: float, end: float, then_hold: float = 0.0) -> None:
        """Hold every joint where it is now while kp goes from start to end times LeRobot's
        gains, then keep sending the end value for `then_hold` seconds. kd stays at LeRobot's
        gains. The right arm's gravity feedforward is scaled along with kp: the limp arm may
        rest against the body, and switching the feedforward on at once would lift it off."""
        hold = {k: v for k, v in self._g1.get_observation().items() if k.endswith(".q")}
        q = np.array(list(hold.values()))  # G1_29_JointIndex order: arms are 15-28, right arm 22-28
        tau_gravity = np.zeros(len(q))
        tau_gravity[22:29] = self._g1.arm_ik.solve_tau(q[15:29])[7:]
        ramp_steps = round(RAMP_TIME / self.control_dt)
        for i in range(ramp_steps + round(then_hold / self.control_dt) + 1):
            scale = start + (end - start) * min(i / ramp_steps, 1.0)
            self._g1.publish_lowcmd(
                hold, kp=scale * np.asarray(self._g1.kp), kd=self._g1.kd, tau=scale * tau_gravity
            )
            time.sleep(self.control_dt)