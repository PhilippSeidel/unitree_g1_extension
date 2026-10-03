from lerobot.robots.unitree_g1 import UnitreeG1Config

from .robot import G1Robot

class G1RobotSim(G1Robot):
    def __init__(
        self,
        max_lin_vel: float = 0.25,
        max_ang_vel: float = 1.0,
        viewer: bool = True,  # MuJoCo window; False publishes the sim cameras over ZMQ instead
        **kwargs,  # control_dt, urdf_path
    ):
        config = UnitreeG1Config(
            is_simulation=True,
            gravity_compensation=True,
            end_effector="dex3",  # fingers are not commanded
            sim_publish_images=not viewer,
        )
        super().__init__(config, max_lin_vel, max_ang_vel, **kwargs)