import numpy as np
import pinocchio as pin

# Arm joints of the 29-DoF G1, in the order joint_pos is reported (URDF joint names).
_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")


def arm_joint_names(side: str) -> tuple:
    return tuple(f"{side}_{j}_joint" for j in _JOINTS)


RIGHT_ARM_JOINT_NAMES = arm_joint_names("right")
LEFT_ARM_JOINT_NAMES = arm_joint_names("left")

EE_OFFSET = (0.05, 0.0, 0.0)  # from <side>_wrist_yaw_joint, as LeRobot's "R_ee" / "L_ee"
LIMIT_MARGIN = 0.05  # rad kept clear of each joint limit


class ArmIK:
    """Closed-loop differential IK for one arm (side "right" or "left").
    """

    def __init__(
        self,
        urdf_path: str,
        side: str = "right",
        gain: float = 10.0,  # 1/s, how fast the hand closes in on the target
        damping: float = 0.05,  # DLS damping, keeps it smooth near singularities
        posture_gain: float = 1.0,  # 1/s, null-space pull back to the start posture
        max_joint_vel: float = 2.0,  # rad/s
        max_lag: tuple = (0.02, 0.1),  # m, rad
    ):
        if side not in ("right", "left"):
            raise ValueError(f"side must be 'right' or 'left', not {side!r}")
        self.side = side
        self.model = pin.buildModelFromUrdf(urdf_path)
        wrist = self.model.getJointId(f"{side}_wrist_yaw_joint")
        self._ee = self.model.addFrame(
            pin.Frame(f"{side}_ee", wrist, pin.SE3(np.eye(3), np.array(EE_OFFSET)), pin.FrameType.OP_FRAME)
        )
        self._torso = self.model.getFrameId("torso_link")
        self.data = self.model.createData()

        joint_ids = [self.model.getJointId(n) for n in arm_joint_names(side)]
        self._iq = [self.model.idx_qs[j] for j in joint_ids]
        self._iv = [self.model.idx_vs[j] for j in joint_ids]
        self.lower = self.model.lowerPositionLimit[self._iq] + LIMIT_MARGIN
        self.upper = self.model.upperPositionLimit[self._iq] - LIMIT_MARGIN

        self.gain, self.damping, self.posture_gain = gain, damping, posture_gain
        self.max_joint_vel = max_joint_vel
        self.max_lag = max_lag

        self._q_full = pin.neutral(self.model)  # every other joint stays at zero
        self.q = np.zeros(7)  # commanded joint positions
        self._q_posture = np.zeros(7)
        self._target = pin.SE3.Identity()

    # ------------------------------------------------------------------ API
    def reset(self, q: np.ndarray) -> None:
        """Start from measured joint positions; the hand holds where it is."""
        self.q = np.clip(np.asarray(q, dtype=float), self.lower, self.upper)
        self._q_posture = self.q.copy()
        self.hold()

    def hold(self) -> None:
        """Drop any remaining target error: stop where the commanded arm is now."""
        self._target = self._fk(self.q)

    def step(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """Advance by dt with EE twist [vx vy vz wx wy wz] (torso frame); return joint targets."""
        v, w = twist[:3], twist[3:]
        self._target = pin.SE3(pin.exp3(w * dt) @ self._target.rotation, self._target.translation + v * dt)

        current = self._fk(self.q)
        err = self._error(current)
        lag_lin, lag_ang = np.linalg.norm(err[:3]), np.linalg.norm(err[3:])
        if lag_lin > self.max_lag[0] or lag_ang > self.max_lag[1]:  # anti-windup
            err[:3] *= min(1.0, self.max_lag[0] / max(lag_lin, 1e-12))
            err[3:] *= min(1.0, self.max_lag[1] / max(lag_ang, 1e-12))
            self._target = pin.SE3(pin.exp3(err[3:]) @ current.rotation, current.translation + err[:3])

        J = self._jacobian()
        J_dls = J.T @ np.linalg.inv(J @ J.T + self.damping**2 * np.eye(6))
        dq = J_dls @ (twist + self.gain * err)
        null = np.eye(7) - np.linalg.pinv(J, rcond=1e-2) @ J  # exact null space, so the hand isn't disturbed
        dq += null @ (self.posture_gain * (self._q_posture - self.q))

        peak = np.max(np.abs(dq))
        if peak > self.max_joint_vel:
            dq *= self.max_joint_vel / peak
        self.q = np.clip(self.q + dq * dt, self.lower, self.upper)
        return self.q.copy()

    def pose(self, q: np.ndarray) -> np.ndarray:
        """EE pose for joint positions q as [x y z qx qy qz qw], torso frame."""
        T = self._fk(q)
        return np.concatenate([T.translation, pin.Quaternion(T.rotation).coeffs()])

    # ------------------------------------------------------------ internals
    def _fk(self, q: np.ndarray) -> pin.SE3:
        self._q_full[self._iq] = q
        pin.framesForwardKinematics(self.model, self.data, self._q_full)
        return self.data.oMf[self._torso].actInv(self.data.oMf[self._ee])

    def _error(self, current: pin.SE3) -> np.ndarray:
        """[position error, rotation error as axis*angle], torso frame."""
        e_rot = pin.log3(self._target.rotation @ current.rotation.T)
        return np.concatenate([self._target.translation - current.translation, e_rot])

    def _jacobian(self) -> np.ndarray:
        """6x7 Jacobian of the EE (linear at the EE point, angular), torso axes.
        Uses the configuration of the last _fk call."""
        J = pin.computeFrameJacobian(self.model, self.data, self._q_full, self._ee, pin.LOCAL_WORLD_ALIGNED)
        R = self.data.oMf[self._torso].rotation.T
        return np.vstack([R @ J[:3, self._iv], R @ J[3:, self._iv]])