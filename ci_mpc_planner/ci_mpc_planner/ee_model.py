"""FR3 end-effector kinematics and translational operational-space inertia from the URDF (Pinocchio).

Same reduction as the controller's PinocchioModel: fingers locked, arm joints fr3_joint1..7.
"""
import numpy as np
import pinocchio as pin

ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


class EeModel:
    def __init__(self, urdf_path, ee_frame="fr3_hand_tcp", armature=None):
        """armature: 7 values [kg m^2] (rotor inertia reflected through the gearbox), added to M by crba."""
        full = pin.buildModelFromUrdf(urdf_path)
        lock = [j for j in range(1, full.njoints) if full.names[j] not in ARM_JOINTS]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        if armature:
            self.model.armature[:] = np.asarray(armature, dtype=float)
        self.data = self.model.createData()
        self.ee = self.model.getFrameId(ee_frame)

    def update(self, q, dq):
        """Returns (position, rotation, linear velocity, translational inertia Lambda_t (3x3))."""
        m, d = self.model, self.data
        pin.computeJointJacobians(m, d, q)
        pin.framesForwardKinematics(m, d, q)
        J = pin.getFrameJacobian(m, d, self.ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        M = pin.crba(m, d, q)
        M = np.triu(M) + np.triu(M, 1).T
        Jt = J[:3]
        lam_t = np.linalg.inv(Jt @ np.linalg.solve(M, Jt.T))
        T = d.oMf[self.ee]
        return T.translation.copy(), T.rotation.copy(), Jt @ dq, lam_t
