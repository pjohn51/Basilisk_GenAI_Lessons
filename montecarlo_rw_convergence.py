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
from Basilisk.utilities import RigidBodyKinematics as rbk

from custom_module import LQRwController

import matplotlib.pyplot as plt

from BSK_masters import BSKSim, BSKScenario

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

fsw_dt = 1.0
#------------------------------------------------------------------


# #-----Initialize Reaction Wheels--------------------------------------------
def ScenarioFunction():


    #------------Prepare simulation--------------------
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("DynamicsProcess", priority=10) #high priority dynamics to ensure this is run first at each timestep
    fswProcess = scSim.CreateNewProcess("FswProcess",      priority=5)

    dynTask = scSim.CreateNewTask("DynamicsTask", macros.sec2nano(0.5)) #500ms timestep
    fswTask = scSim.CreateNewTask("FswTask",      macros.sec2nano(fsw_dt))
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
    scObject.hub.omega_BN_BInit = list(1/10*np.random.rand(3))
    scObject.hub.sigma_BNInit = list(1/10*np.random.rand(3))

    scObject.hub.r_CN_NInit = r0.tolist() #Initial altitude
    scObject.hub.v_CN_NInit = v0.tolist() #Initial orbital velocity
    # scSim.AddModelToTask("DynamicsTask", scObject) #add scObject to task

    scSim.hubref = scObject.hub
    scSim.AddModelToTask("DynamicsTask", scObject, None, 1)
    #------------------------------------------------------------


    #------Create central body------------------
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)
    #-------------------------------------------

    rwFactory = simIncludeRW.rwFactory()

    # RW spin axes (unit vectors in body frame) — orthogonal configuration
    varRWModel = reactionWheelStateEffector.BalancedWheels

    rw1 = rwFactory.create('Honeywell_HR16',
                    gsHat_B = [1,0,0], # Spin axis
                    maxMomentum = 50.0,   # N*m*s
                    Omega      = 100.0,   # initial speed [RPM]
                    RWModel    = varRWModel)


    rw2 = rwFactory.create('Honeywell_HR16',
                    gsHat_B = [0,1,0],
                    maxMomentum = 50.0,
                    Omega      = 200.0,   # [RPM]
                    RWModel    = varRWModel)


    rw3 = rwFactory.create('Honeywell_HR16',
                    gsHat_B = [0,0,1],
                    maxMomentum = 50.0,
                    Omega      = 150.0,   # [RPM]
                    RWModel    = varRWModel)

    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "ReactionWheels"

    scSim.RW1 = rw1
    scSim.RW2 = rw2
    scSim.RW3 = rw3

    rwFactory.addToSpacecraft(scObject.ModelTag,
                            rwStateEffector,
                            scObject)

    rwConfigMsg = rwFactory.getConfigMessage()
    scSim.AddModelToTask("FswTask",rwStateEffector)

    # #-----------------------------------------------------------------------------------

    # Write reaction wheel states


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

    # Initialize reaction wheel messaging
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    rwMotorTorqueObj.controlAxes_B = list(np.eye(3).flatten())
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(rwConfigMsg) # Parameters
    # rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg) #Connects subscribed message to the actual RW effector
    scSim.AddModelToTask("FswTask", rwMotorTorqueObj)


    # Initialize custom controller module
    controller = LQRwController(fsw_dt)
    controller.attMsg.subscribeTo(attErrObj.attGuidOutMsg)
    controller.vehConfigInMsg.subscribeTo(vehConfigObj.vecConfigOutMsg)
    controller.navInMsg.subscribeTo(sNavObj.attOutMsg)
    controller.rwParams.subscribeTo(rwConfigMsg)
    scSim.AddModelToTask("FswTask", controller)

    # Connect controller output to motor torque input
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(controller.cmdTorqueOutMsg)  #Actual motor torque
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(controller.commandRwMsg)
    # Add recorder to rwMotorCmd
    # rwMotorTorqueLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder()

    # Setup recorders list for tracking
    scSim.msgRecList = {}
    samplingTime = macros.sec2nano(1.0)
    scSim.msgRecList["poseMsg"] = sNavObj.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask("FswTask", scSim.msgRecList["poseMsg"])


    viz = vizSupport.enableUnityVisualization(scSim, 
                                          "DynamicsTask", 
                                          scObject,
                                          rwEffectorList=rwStateEffector,
                                          saveFile="sim_output.bin")
    #

    return scSim

def executeSim(scSim):
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(T_orbit))   # 200 seconds
    scSim.ExecuteSimulation()


def data_analysis(data, rp):
    '''
    data["messages"] contains the message data for each run, is length num runs * num messages kept track of
    this function is run once per run after all runs execute
    '''
    msgs = data["messages"]
    angular = np.array(msgs["poseMsg.omega_BN_B"])
    euler = np.array(msgs["poseMsg.sigma_BN"])

    initial_speed.append(np.linalg.norm(angular[0][1:]))
    initial_error.append(np.linalg.norm(euler[0][1:]))

    final_speed.append(np.linalg.norm(angular[-1][1:]))
    final_angle_error.append(np.linalg.norm(euler[-1][1:]))
    
    return True

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import InertiaTensorDispersion, UniformEulerAngleMRPDispersion, NormalVectorCartDispersion


num_sims = 32
num_threads = 8

monteCarlo = Controller()
monteCarlo.setArchiveDir("mc_output_dir")
monteCarlo.setSimulationFunction(ScenarioFunction)
monteCarlo.setExecutionFunction(executeSim)
# monteCarlo.setConfigurationFunction(configureSimulation)
monteCarlo.setExecutionCount(num_sims)        # number of MC runs
monteCarlo.setThreadCount(num_threads)             # parallel threads
monteCarlo.setShouldDisperseSeeds(True)
monteCarlo.setShowProgressBar(True)

# Add dispersions on start position
monteCarlo.addDispersion(UniformEulerAngleMRPDispersion("scObject.hub.sigma_BNInit"))
monteCarlo.addDispersion(NormalVectorCartDispersion("scObject.hub.omega_BN_BInit", 0, np.pi))

# Add retention policy (what data to save)
retainPolicy = RetentionPolicy()
retainPolicy.addMessageLog("poseMsg", ["sigma_BN","omega_BN_B"])
monteCarlo.addRetentionPolicy(retainPolicy)

retainPolicy.setDataCallback(data_analysis)


# Run (parallel by default)
monteCarlo.executeSimulations()

numSuccess = 0
numFail = 0
total = 0

initial_speed = []
initial_error = []

final_speed = []
final_angle_error = []

monteCarlo.executeCallbacks()

t = np.arange(num_sims)


plt.scatter(t,final_speed)
plt.scatter(t,initial_speed)
plt.legend(["Final Speed", "Initial Speed"])
plt.xlabel("Run")
plt.ylabel("Angular Speed")
plt.title("Angular Speed Across Runs")
plt.grid()
plt.show()

plt.scatter(t,final_angle_error)
plt.scatter(t,initial_error)
plt.legend(["Final Angle Error", "Initial Angle Error"])
plt.xlabel("Run")
plt.ylabel("Angle Error")
plt.title("Angle Error Across Runs")
plt.grid()
plt.show()