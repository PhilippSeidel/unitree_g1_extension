import argparse
import logging
import threading
import time
from typing import Callable, Optional, Sequence
from incar_networking.robot_interface import IncarRobotInterface

from .robot import G1Robot
from .robot_real import G1RobotReal
from .robot_simulation import G1RobotSim
 
log = logging.getLogger(__name__)

 
 
class IncarG1Bridge:

    def __init__(
        self,
        robot: G1Robot,
        command_timeout: float = 0.2,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.robot = robot
        self.command_timeout = float(command_timeout)
        self._clock = clock
        self._lock = threading.Lock()
        self._t_last_cmd: Optional[float] = None
        self._streaming = False
 
    def on_right_ee_velocity(self, velocity: Sequence[float]) -> None:
        with self._lock:
            if not self.robot.command_ee_velocity(velocity):
                return
            self._t_last_cmd = self._clock()
            if not self._streaming:
                self._streaming = True
                log.info("command stream started")
 
    def on_loop(self, interface) -> None:
        """Loop callback, called by IncarRobotInterface every dt."""
        self._check_watchdog()
        state = self.robot.arm_state()
        interface.set_robot_state(  
            f"{self.robot.side}_arm",
            ee_pose=state.ee_pose.tolist(),
            joint_pos=state.joint_pos.tolist(),
            joint_vel=state.joint_vel.tolist(),
            joint_effort=state.joint_effort.tolist(),
        )
        interface.publish_state()
 
    def _check_watchdog(self) -> None:
        with self._lock:
            if not self._streaming:
                return
            if self._clock() - self._t_last_cmd <= self.command_timeout:
                return
            self._streaming = False
            self.robot.stop_arm()
        log.info("command stream stopped (> %.2f s without a message), arm stopped", self.command_timeout)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="INCAR teleoperation of one arm of the G1")
    parser.add_argument(
        "--robot",
        choices=["sim", "real"],
        default="sim",
        help="sim: LeRobot's MuJoCo G1 (default); real: the G1 on the gantry, via g1_server.py on the robot",
    )
    parser.add_argument(
        "--arm",
        choices=["right", "left"],
        default="right",
        help="arm to teleoperate (default right); left follows the right controller, mirrored",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="sim only: don't open the MuJoCo window, publish the sim cameras over ZMQ instead (for g1_sim cameras)",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)

    if args.robot == "real":
        robot = G1RobotReal(arm=args.arm)
    else:
        robot = G1RobotSim(arm=args.arm, viewer=not args.no_viewer)
    robot.connect()
    bridge = IncarG1Bridge(robot)

    interface = IncarRobotInterface(
        0.01,
        command_hooks = {
            "right.commands.arm.ee.velocity": bridge.on_right_ee_velocity
        },
        loop_callbacks = [
            bridge.on_loop
        ]
    )
    try:
        interface.start("127.0.0.1")
        threading.Event().wait()  
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()  