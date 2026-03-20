import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport, simIncludeGravBody, simIncludeRW
from Basilisk.simulation import spacecraft, reactionWheelStateEffector, simpleNav
from Basilisk.architecture import bskLogging
from Basilisk.utilities import vizSupport
from Basilisk.fswAlgorithms import inertial3D, attTrackingError, rwMotorTorque
from Basilisk.fswAlgorithms import vehicleConfigData

from custom_module import LQRwController

# bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)  # suppress info output
mu_earth = 3.986004418e14   # Earth gravitational parameter [m^3/s^2]
R_earth  = 6371e3           # Earth mean radius [m]

# ── Define classical orbital elements ─────────────────────────────────────────
oe = orbitalMotion.ClassicElements()
oe.a     = R_earth + 200e3       # semi-major axis: 200 km altitude circular orbit (LEO)
oe.e     = 0.001             # nearly circular
oe.i     = 53.0 * macros.D2R # 53 degree inclination (Starlink orbit)
oe.Omega = 137.8 * macros.D2R # RAAN
oe.omega = 347.8 * macros.D2R# argument of perigee
oe.f     = 85.3 * macros.D2R # true anomaly (starting position along orbit)

r0, v0 = orbitalMotion.elem2rv(mu_earth, oe) #r0, v0 for circular orbit
T_orbit = 2 * np.pi * np.sqrt(oe.a**3 / mu_earth) # complete orbit
#------------------------------------------------------------------

#------------Prepare simulation--------------------
scSim = SimulationBaseClass.SimBaseClass()
dynProcess = scSim.CreateNewProcess("DynamicsProcess", priority=10) #high priority dynamics to ensure this is run first at each timestep
fswProcess = scSim.CreateNewProcess("FswProcess",      priority=5)

dynTask = scSim.CreateNewTask("DynamicsTask", macros.sec2nano(0.5)) #500ms timestep
fswTask = scSim.CreateNewTask("FswTask",      macros.sec2nano(1.0))
dynProcess.addTask(dynTask)
fswProcess.addTask(fswTask)
#------------------------------------

#------------------Init spacecraft params--------------
scObject = spacecraft.Spacecraft()
scObject.ModelTag = "MySpacecraft"
scObject.hub.mHub = 100.0

I = np.diag([20.0, 30.0, 40.0])
scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I.flatten())
scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] #CoM offset
scObject.hub.omega_BN_BInit = list(np.random.rand(3).reshape((3,1))) #Initial Angular rotation, body frame WRT inertial
# scObject.hub.omega_BN_BInit = [0.0,0.0,0.0]

scObject.hub.r_CN_NInit = r0.tolist() #Initial altitude
scObject.hub.v_CN_NInit = v0.tolist() #Initial orbital velocity
scSim.AddModelToTask("DynamicsTask", scObject) #add scObject to task
#------------------------------------------------------------


print("Initialized spacecraft object")

#------Create central body------------------
gravFactory = simIncludeGravBody.gravBodyFactory()
earth = gravFactory.createEarth()
earth.isCentralBody = True
gravFactory.addBodiesTo(scObject)
#-------------------------------------------


# #-----Initialize Reaction Wheels--------------------------------------------
rwFactory = simIncludeRW.rwFactory()

# RW spin axes (unit vectors in body frame) — orthogonal configuration
varRWModel = reactionWheelStateEffector.BalancedWheels

# rwFactory.create('Honeywell_HR16',
#                  gsHat_B = [np.sqrt(2)/2, 0, np.sqrt(2)/2], # Spin axis
#                  maxMomentum = 50.0,   # N*m*s
#                  Omega      = 100.0,   # initial speed [RPM]
#                  RWModel    = varRWModel,
#                  useMaxTorque=False)


# rwFactory.create('Honeywell_HR16',
#                  gsHat_B = [-np.sqrt(2)/2, 0, np.sqrt(2)/2],
#                  maxMomentum = 50.0,
#                  Omega      = 200.0,   # [RPM]
#                  RWModel    = varRWModel,
#                  useMaxTorque=False)


# rwFactory.create('Honeywell_HR16',
#                  gsHat_B = [0, np.sqrt(2)/2, np.sqrt(2)/2],
#                  maxMomentum = 50.0,
#                  Omega      = 150.0,   # [RPM]
#                  RWModel    = varRWModel,
#                  useMaxTorque=False)

# rwFactory.create('Honeywell_HR16',
#                  gsHat_B = [0, -np.sqrt(2)/2, np.sqrt(2)/2],
#                  maxMomentum = 50.0,
#                  Omega      = 150.0,   # [RPM]
#                  RWModel    = varRWModel,
#                  useMaxTorque=False)
rwFactory.create('Honeywell_HR16',
                 gsHat_B = [1,0,0], # Spin axis
                 maxMomentum = 50.0,   # N*m*s
                 Omega      = 100.0,   # initial speed [RPM]
                 RWModel    = varRWModel)


rwFactory.create('Honeywell_HR16',
                 gsHat_B = [0,1,0],
                 maxMomentum = 50.0,
                 Omega      = 200.0,   # [RPM]
                 RWModel    = varRWModel)


rwFactory.create('Honeywell_HR16',
                 gsHat_B = [0,0,1],
                 maxMomentum = 50.0,
                 Omega      = 150.0,   # [RPM]
                 RWModel    = varRWModel)

rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
rwStateEffector.ModelTag = "ReactionWheels"

rwFactory.addToSpacecraft(scObject.ModelTag,
                          rwStateEffector,
                          scObject)

rwConfigMsg = rwFactory.getConfigMessage()
# #-----------------------------------------------------------------------------------


# ── vehicleConfigData: broadcasts mass properties to custom controller ────────────────
vehConfigObj = vehicleConfigData.vehicleConfigData()
vehConfigObj.ModelTag = "vehicleConfigData"
vehConfigObj.ISCPntB_B = I.flatten().tolist()
scSim.AddModelToTask("FswTask", vehConfigObj)

# Add tracking to the FSW modules
inertial3DObj = inertial3D.inertial3D()
inertial3DObj.ModelTag = "inertial3D"
inertial3DObj.sigma_R0N = [0.0, 0.0, 0.0] # Reference attitude
scSim.AddModelToTask("FswTask", inertial3DObj)


# ── SimpleNav (scStateOutMsg → attOutMsg / transOutMsg) ───────────────────────
sNavObj = simpleNav.SimpleNav()
sNavObj.ModelTag = "SimpleNav"
sNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg) # Subscribes to the spacecrafts state
scSim.AddModelToTask("DynamicsTask", sNavObj)

attErrObj = attTrackingError.attTrackingError()
attErrObj.ModelTag = "attTrackingError"
attErrObj.attNavInMsg.subscribeTo(sNavObj.attOutMsg)
attErrObj.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
scSim.AddModelToTask("FswTask", attErrObj)
#



# Initialize custom controller module
controller = LQRwController()
controller.attMsg.subscribeTo(attErrObj.attGuidOutMsg)
controller.vehConfigInMsg.subscribeTo(vehConfigObj.vecConfigOutMsg)
controller.navInMsg.subscribeTo(sNavObj.attOutMsg)
scSim.AddModelToTask("FswTask", controller)

# Connect controller to RWs
rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
rwMotorTorqueObj.ModelTag = "rwMotorTorque"
rwMotorTorqueObj.controlAxes_B = [1,0,0, 0,1,0, 0,0,1]
rwMotorTorqueObj.vehControlInMsg.subscribeTo(controller.cmdTorqueOutMsg)  # custom output!
rwMotorTorqueObj.rwParamsInMsg.subscribeTo(rwConfigMsg)
rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
scSim.AddModelToTask("FswTask", rwMotorTorqueObj)


# Add recorder to rwMotorCmd
rwMotorTorqueLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder()
scSim.AddModelToTask("FswTask", rwMotorTorqueLog)
ctrlTorqueLog = controller.cmdTorqueOutMsg.recorder()
scSim.AddModelToTask("FswTask", ctrlTorqueLog)

#Set up sim visualization
viz = vizSupport.enableUnityVisualization(scSim, 
                                          "DynamicsTask", 
                                          scObject, 
                                          saveFile="sim_output.bin")


#

scSim.InitializeSimulation()
scSim.ConfigureStopTime(macros.sec2nano(T_orbit))   # 200 seconds
print("Executing. . . ")
scSim.ExecuteSimulation()


# print(np.array(rwMotorTorqueLog.motorTorque))
print(ctrlTorqueLog.motorTorque)