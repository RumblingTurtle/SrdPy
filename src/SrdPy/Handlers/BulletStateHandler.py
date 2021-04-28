import numpy as np
import pybullet as p
class BulletStateHandler():
    def __init__(self,bodyId,jointIds):
        self.bodyId = bodyId
        self.jointIds = jointIds
        self.dofRobot = len(jointIds)
        q_state = []
        v_state = []
        for id in self.jointIds:
            state = p.getJointState(self.bodyId, id)
            q_state.append(float(state[0]))
            v_state.append(float(state[1]))
        self.dofRobot = len(q_state)
        self.q = q_state
        self.v = v_state
        self.a = np.full(len(self.q), np.nan)

    def getPositionVelocityAcceleration(self):
        q_state = []
        v_state = []
        for id in self.jointIds:
            state = p.getJointState(self.bodyId, id)
            q_state.append(float(state[0]))
            v_state.append(float(state[1]))
        self.dofRobot = len(q_state)
        self.q = q_state
        self.v = v_state
        return q_state, v_state, None