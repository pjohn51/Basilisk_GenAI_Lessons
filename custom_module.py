
from Basilisk.architecture import sysModel, messaging

import numpy as np
from scipy.linalg import solve_discrete_are

class LQRwController(sysModel.SysModel):

    def __init__(self):
        super().__init__()
        self.modelTag = 'rwController'
        
        # Controller parameters
        self.dt = None
        self.initialized = False
        self.Q = np.diag([10.0,10.0])
        self.R = np.diag([10.0,10.0])

        # Input Message
        self.attMsg = messaging.AttGuidMsg_C()
        self.vehConfigInMsg  = messaging.VehicleConfigMsg_C()
        self.navInMsg = messaging.NavAttMsg_C()
        
        # Output Message
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
    
    def Reset(self, CurrentSimNanos):
        """Called once at simulation start — read static config messages."""
        vehConfig = self.vehConfigInMsg.read()
        I_flat = vehConfig.ISCPntB_B
        m = vehConfig.massSC
        self._inertia = np.array(I_flat).reshape(3, 3)
        self._m = m
        self.K = 2.0 * 0.7 * 0.15 * np.sum(I_flat)
        self.P = 0.15**2 * np.sum(I_flat)
    
    def UpdateState(self, CurrentSimNanos):
        """Called every FSW task time step."""
        # ── Read guidance error message ───────────────────────────────────────
        guidData    = self.navInMsg.read()
        # omega_BR_B  = np.array(guidData.omega_BR_B) # rate error in body frame wrt reference axis
        # sigma_BR = np.array(guidData.sigma_BR)
        omega_BN_B = np.array(guidData.omega_BN_B)
        
        # omega_BR_B = np.array(guidData.omega_RN_B)
        # omega_BN_B = omega_BR_B + omega_RN_B
        # print("Guidance message:",guidData, omega_BR_B)
        # print(omega_BN_B)

        # ── Simple Compensator────────────────────────────────────────────────────
        u = list(-10*omega_BN_B)
        # u = -self.K * sigma_BR - self.P * omega_BR_B
        # u=[0.0,0.0,1.0]
        # u = list(-1000*omega_BN_B)
        # print(f"Control output: {u}")
        # ── Write output torque message ───────────────────────────────────────
        torquePayload = messaging.CmdTorqueBodyMsgPayload()
        torquePayload.torqueRequestBody = u
        self.cmdTorqueOutMsg.write(torquePayload, CurrentSimNanos, self.moduleID)
