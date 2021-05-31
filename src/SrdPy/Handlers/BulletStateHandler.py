import numpy as np
import pybullet as p
class BulletStateHandler():

    def __init__(self,bodyId,jointIds,floatingFirst=False):
        self.bodyId = bodyId
        self.jointIds = jointIds
        self.dofRobot = len(jointIds)
        self.floatingFirst = floatingFirst
        self.setPosVelAcc()
        
    def setPosVelAcc(self):
        q_state = []
        v_state = []
        for id in self.jointIds:
            state = p.getJointState(self.bodyId, id)
            q_state.append(float(state[0]))
            v_state.append(float(state[1]))
        self.dofRobot = len(q_state)
        self.q = q_state
        self.v = v_state
        if self.floatingFirst:
            self.q = [0.0]*6+self.q
            self.v = [0.0]*6+self.v
        self.a = np.full(len(self.q), np.nan)

    def getPositionVelocityAcceleration(self):
        self.setPosVelAcc()
        return self.q, self.v, None
