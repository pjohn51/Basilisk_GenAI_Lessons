
from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros

import numpy as np
from scipy.linalg import solve_discrete_are
import scipy.signal as signal

class LQRwController(sysModel.SysModel):

    def __init__(self, timestep):
        super().__init__()
        self.modelTag = 'rwController'
        
        # Controller parameters
        self.dt = timestep
        self.Q = np.eye(6,dtype=np.float64)
        self.Q[0:3,0:3]*=4 # more emphasis on velocity
        self.R = np.eye(3,dtype=np.float64)
        self.R*=10
        self.A = np.zeros_like(np.eye(6))
        self.A[0,3] = 1.0
        self.A[1,4] = 1.0
        self.A[2,5] = 1.0
        self.A*=1/2

        
        self.C = np.eye(6)
        self.D = np.zeros((6,3))

        # Input Message
        self.attMsg = messaging.AttGuidMsg_C()
        self.vehConfigInMsg  = messaging.VehicleConfigMsg_C()
        self.navInMsg = messaging.NavAttMsg_C()
        self.rwParams = messaging.RWArrayConfigMsg_C()
        
        # Output Message
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.commandRwMsg = messaging.ArrayMotorTorqueMsg()



    # def build_SS_model(self,Ix,Iy,Iz,dt):

    #     A_d, B_d, _, _, _ = signal.cont2discrete((self.A, self.B, self.C, self.D), dt, method='zoh')

    #     return A_d, B_d

    def Reset(self, CurrentSimNanos):
        """Called once at simulation start — read static config messages."""
        vehConfig = self.vehConfigInMsg.read()
        rwParams = self.rwParams.read()
        self.cmdTorqueOutMsg.write(
            messaging.CmdTorqueBodyMsgPayload()
        )
        self.inertia = np.array(vehConfig.ISCPntB_B).reshape((3,3))
        self.numRW = rwParams.numRW
        self.maxTorque = rwParams.uMax

        self.B = np.zeros((6,3))
        self.B[3,0] = 1/self.inertia[0,0]
        self.B[4,1] = 1/self.inertia[1,1]
        self.B[5,2] = 1/self.inertia[2,2]

        self.A, self.B, _, _, _ = signal.cont2discrete((self.A, self.B, self.C, self.D), self.dt, method='zoh')
        P = solve_discrete_are(self.A,self.B,self.Q,self.R)
        self.K =  np.linalg.inv(self.R + self.B.T@P@self.B) @ (self.B.T@P@self.A)

    
    def UpdateState(self, CurrentSimNanos):
        """Called every FSW task time step."""
        # ── Read guidance error message ───────────────────────────────────────
        guidData    = self.navInMsg.read()
        # omega_BR_B  = np.array(guidData.omega_BR_B) # rate error in body frame wrt reference axis
        sigma_BN = np.array(guidData.sigma_BN)
        omega_BN_B = np.array(guidData.omega_BN_B)

        #Note, this requires knowledge of the RW
        
        # omega_BR_B = np.array(guidData.omega_RN_B)
        # omega_BN_B = omega_BR_B + omega_RN_B
        # print("Guidance message:",guidData, omega_BR_B)
        # print(omega_BN_B)

        # LQR Controller
        curr_state = np.concatenate((sigma_BN,omega_BN_B),axis=-1)
        u = list(self.K@curr_state) # Results in a 3x1 array
            
        
        self.last_time = CurrentSimNanos*macros.NANO2SEC
        # ── Write output torque message ───────────────────────────────────────
        torquePayload = messaging.ArrayMotorTorqueMsgPayload()
        torquePayload.motorTorque = u
        self.commandRwMsg.write(torquePayload, CurrentSimNanos, self.moduleID)
