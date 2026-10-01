import numpy as np
import pinocchio as pin
 
from .robot import RIGHT_ARM_JOINT_NAMES
 
EE_OFFSET = (0.05, 0.0, 0.0)  # from right_wrist_yaw_joint, as LeRobot's "R_ee"
LIMIT_MARGIN = 0.05  # rad kept clear of each joint limit
 
 
class RightArmIK:
 
    def __init__(
        self,
        urdf_path: str,
        gain: float = 10.0, 
        damping: float = 0.05,  
        posture_gain: float = 1.0, 
        max_joint_vel: float = 2.0,  
        max_lag: tuple = (0.02, 0.1),
    ):
        self.model = pin.buildModelFromUrdf(urdf_path)
        wrist = self.model.getJointId("right_wrist_yaw_joint")
        self._ee = self.model.addFrame(
            pin.Frame("right_ee", wrist, pin.SE3(np.eye(3), np.array(EE_OFFSET)), pin.FrameType.OP_FRAME)
        )
        self._torso = self.model.getFrameId("torso_link")
        self.data = self.model.createData()
 
        joint_ids = [self.model.getJointId(n) for n in RIGHT_ARM_JOINT_NAMES]
        self._iq = [self.model.idx_qs[j] for j in joint_ids]
        self._iv = [self.model.idx_vs[j] for j in joint_ids]
        self.lower = self.model.lowerPositionLimit[self._iq] + LIMIT_MARGIN
        self.upper = self.model.upperPositionLimit[self._iq] - LIMIT_MARGIN
 
        self.gain, self.damping, self.posture_gain = gain, damping, posture_gain
        self.max_joint_vel = max_joint_vel
        self.max_lag = max_lag
 
        self._q_full = pin.neutral(self.model) 
        self.q = np.zeros(7) 
        self._q_posture = np.zeros(7)
        self._target = pin.SE3.Identity()
 
    def reset(self, q: np.ndarray) -> None:
        self.q = np.clip(np.asarray(q, dtype=float), self.lower, self.upper)
        self._q_posture = self.q.copy()
        self.hold()
 
    def hold(self) -> None:
        self._target = self._fk(self.q)
 
    def step(self, twist: np.ndarray, dt: float) -> np.ndarray:
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
        T = self._fk(q)
        return np.concatenate([T.translation, pin.Quaternion(T.rotation).coeffs()])
 
    def _fk(self, q: np.ndarray) -> pin.SE3:
        self._q_full[self._iq] = q
        pin.framesForwardKinematics(self.model, self.data, self._q_full)
        return self.data.oMf[self._torso].actInv(self.data.oMf[self._ee])
 
    def _error(self, current: pin.SE3) -> np.ndarray:
        e_rot = pin.log3(self._target.rotation @ current.rotation.T)
        return np.concatenate([self._target.translation - current.translation, e_rot])
 
    def _jacobian(self) -> np.ndarray:
        J = pin.computeFrameJacobian(self.model, self.data, self._q_full, self._ee, pin.LOCAL_WORLD_ALIGNED)
        R = self.data.oMf[self._torso].rotation.T
        return np.vstack([R @ J[:3, self._iv], R @ J[3:, self._iv]])
