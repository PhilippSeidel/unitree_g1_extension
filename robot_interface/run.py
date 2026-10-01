import logging
import threading
import time
from typing import Callable, Optional, Sequence
from incar_networking.robot_interface import IncarRobotInterface
 
from .robot import G1Robot
from .robot_simulation import G1RobotSim
 
log = logging.getLogger(__name__)
 
STATE_MODULE = "right_arm"
 
 
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
            if not self.robot.command_right_ee_velocity(velocity):
                return
            self._t_last_cmd = self._clock()
            if not self._streaming:
                self._streaming = True
                log.info("command stream started")
 
    def on_loop(self, interface) -> None:
        self._check_watchdog()
        state = self.robot.right_arm_state()
        interface.set_robot_state(
            STATE_MODULE,
            ee_pose=state.ee_pose.tolist(),
            joint_pos=state.joint_pos.tolist(),
        )
        interface.publish_state()
 
    def _check_watchdog(self) -> None:
        with self._lock:
            if not self._streaming:
                return
            if self._clock() - self._t_last_cmd <= self.command_timeout:
                return
            self._streaming = False
            self.robot.stop_right_arm()
        log.info("command stream stopped (> %.2f s without a message), arm stopped", self.command_timeout)



if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
 
    robot = G1RobotSim()  # switch here between simulation and real robot
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
        threading.Event().wait()  # in case start() returns; keep running until Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()  # LeRobot's sim threads are not daemons; without this Ctrl-C hangs
