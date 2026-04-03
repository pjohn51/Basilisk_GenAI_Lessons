import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport, simIncludeGravBody, simIncludeRW
from Basilisk.utilities import vizSupport

from Basilisk.simulation import spacecraft, reactionWheelStateEffector, simpleNav

from Basilisk.architecture import bskLogging

from Basilisk.fswAlgorithms import inertial3D, attTrackingError, rwMotorTorque
from Basilisk.fswAlgorithms import vehicleConfigData

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import InertiaTensorDispersion, UniformEulerAngleMRPDispersion, NormalVectorCartDispersion

from custom_module import LQRwController
### BEGIN CONSTANTS ##########
# bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)  # suppress info output
MU_E = 3.986004418e14   # Earth gravitational parameter [m^3/s^2]
############################

class SimBase(object):
    def __init__(self, dynamics_dt: int = 2, fsw_dt: int = 1):
        '''
        REQUIRED
        Initializes the simulation base and creates task processes

        Parameters:
            dynamics_dt - [Hz] Update rate for the dynamics tasks
            fsw_dt - [Hz] Update rate for the Fsw tasks
        '''

        self.scSim = SimulationBaseClass.SimBaseClass()
        self.dynProcess = self.scSim.CreateNewProcess("DynamicsProcess", priority=10)
        self.fswProcess = self.scSim.CreateNewProcess("FswProcess",priority=5)
        self.dynTask = self.scSim.CreateNewTask("DynamicsTask",macros.sec2nano(1/dynamics_dt))
        self.fswTask = self.scSim.CreateNewTask("FswTask",macros.sec2nano(1/fsw_dt))

        self.dynProcess.addTask(self.dynTask)
        self.fswProcess.addTask(self.fswTask)

        self.sNavObj = simpleNav.SimpleNav()
        self.controller = LQRwController(1/fsw_dt)
        self.vehConfigObj = vehicleConfigData.vehicleConfigData()

        self.rwStateEffector = None

        self.recording = []
    
    def define_orbit(self, a: float = 7371e3, ecc: float = 0.001, inclination: float = None, raan: float = None, arg_perigree: float = None, true_anomaly: float = None):
        '''
        REQUIRED
        Creates initial orbital radius and velocity based on orbit parameters, assumes Earth as central body

        Parameters:
            a - [m] Semi-major axis
            ecc - Eccentricity
            inclination - [rad] Inclination (defaults to 53deg)
            raan - [rad] RAAN (defaults to 137.8 deg)
            arg_perigree - [rad] Argument of perigree (defaults to 347.8 deg)
            true_anomaly - [rad] True anomaly (defaults to 85.3 deg)
        '''

        oe = orbitalMotion.ClassicElements()
        oe.a = a
        oe.e = ecc
        oe.i = inclination if inclination else 53.0 * macros.D2R
        oe.Omega = raan if raan else 137.8 * macros.D2R
        oe.omega = arg_perigree if arg_perigree else 347.8 * macros.D2R
        oe.f = true_anomaly if true_anomaly else 85.3 * macros.D2R

        r0, v0 = orbitalMotion.elem2rv(MU_E, oe)
        self.r0 = r0.tolist()
        self.v0 = v0.tolist()
        self.T = 2 * np.pi * np.sqrt(oe.a**3/MU_E)

    
    def build_spacecraft(self, name: str = "MySpacecraft", sc_mass: float = 100.0, sc_inertia: list = None, initial_angular_pose: list = None\
        , CoM_offset: list = None):
        '''
        REQUIRED
        Initializes the spacecraft object and creates the central body

            name - Spacecraft name
            sc_inertia - [kgm^2] (flattened list) - 3x3 inertia tensor of the spacecraft flattened to a 1x9 list
            sc_mass - [kg] Spacecraft mass
            initial_angular_pose [rad;rad/s] (6x1 list) - 6DOF Euler Angle pose [angle;angular velocity]
            CoM_offset [m] - Center of mass offset (3x1 list i.e. [ [],[],[] ])
        '''

        # Initialize spacecraft object
        self.scObject = spacecraft.Spacecraft()
        self.scObject.ModelTag = name
        self.scObject.hub.mHub = sc_mass
        self.scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(sc_inertia) if sc_inertia else unitTestSupport.np2EigenMatrix3d(list(np.eye(3).flatten()))
        self.scObject.hub.r_BcB_B = CoM_offset if CoM_offset else [[0.0],[0.0],[0.0]]
        self.scObject.hub.sigma_BNInit = initial_angular_pose[:3] if initial_angular_pose else [0.0,0.0,0.0]
        self.scObject.hub.omega_BN_BInit = initial_angular_pose[3:] if initial_angular_pose else [0.0,0.0,0.0]
        self.scObject.hub.r_CN_NInit = self.r0
        self.scObject.hub.v_CN_NInit = self.v0
        self.scSim.AddModelToTask("DynamicsTask", self.scObject)
        # Expose scObject on scSim so MC dispersion path "scObject.hub.*" resolves
        self.scSim.scObject = self.scObject

        sc_inertia = np.eye(3) if sc_inertia is None else sc_inertia

        self.vehConfigObj.ModelTag = "vehicleConfigData"
        self.vehConfigObj.ISCPntB_B = sc_inertia.flatten().tolist()
        self.scSim.AddModelToTask("FswTask", self.vehConfigObj)

        # Initialize Earth as the central body
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        earth.isCentralBody = True
        gravFactory.addBodiesTo(self.scObject)
    
    def add_rw_wheels(self, spin_axes: list[list], mode: int = 0, spin_axes_inertias: list[float] = None, max_speed: float = None, max_momentum: float = None, **kwargs):
        '''
        OPTIONAL
        Adds Reaction Wheels to spacecraft object
        NOTE: kwargs will be applied to ALL RWs
        Parameters (per RW):
            spin_axes - [unit vector], list[list[int]]  Axes which the RW spins around (list should be size nx3 where n is # of RW)
            mode, int - Reaction wheel mode (see docstring in create_sim.py for more info)
                        0 (Balanced) - The wheels' principal axis are aligned with the spin axis, CM on spin axis, no imbalance effects (SUPPORTED)
                        1 (JitterSimple) - RW have mass imbalances that are approximated as an external force on the SC (NOT SUPPORTED YET)
                        2 (JitterFullyCoupled) - Models mass imbalance as internal forces & torques, RW mass decoupled from SC mass (NOT SUPPORTED YET)

            NEED ONE OF:

            spin_axes_inertias - [kgm^2]. list[float] Inertia around the spin axis (list should be size nx1 where n is # of rw)

            OR

            max_speed - [rad/s] Maximum RW speed
            max_momentum - [Nms] Maximum RW Momentum
        
        Optional kwargs (see docstring in create_sim.py for more info)
            Omega - [RPM] Initial RW Speed
            Omega_max - [RPM] Maximum RW Speed
            rWB_B, 3x1 list [m] - Position vector of RW relative to spacecraft body frame origin
            U_max - [Nm] Max motor torque
            P_max - [W] Max allowed power for changing wheel speed
            useMaxTorque, bool - Clips to U_max
            useMinTorque, bool - Clips below a minimum torque value
            useRWfriaction, bool - Enables RW friction
            fCoulomb - [Nm] Kinetic friction torque coefficient
            fStatic - [Nm] Static friction torque magnitude
            betaStatic - Stribeck friction coefficient
            cViscous - [Nms/rad] Viscous friction coefficient
        '''
        assert spin_axes_inertias is not None or (max_speed is not None and max_momentum is not None), "Either RW Inertia or Speed+Momentum must be supplied"
        assert all([abs(sum(i)-1)<1e-6 for i in spin_axes]), "All spin axes must be unit vectors"

        rwFactory = simIncludeRW.rwFactory()

        n = len(spin_axes)
        if "U_max" not in kwargs:
            kwargs["useMaxTorque"] = False #if max torque is 0 and useMaxTorque is true, this will throw error

        for rw in range(n):
            if spin_axes_inertias is not None:
                rwFactory.create('custom',spin_axes[rw],Js=float(spin_axes_inertias[rw]),**kwargs)
            else:
                rwFactory.create('custom',spin_axes[rw],Omega_max=float(max_speed),maxMomentum=float(max_momentum),**kwargs)
        
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.rwStateEffector.ModelTag = "ReactionWheels"
        rwFactory.addToSpacecraft(self.scObject.ModelTag, self.rwStateEffector, self.scObject)
        self.scSim.AddModelToTask("FswTask",self.rwStateEffector)

        self.rwConfigMsg = rwFactory.getConfigMessage()


    def run_sim(self,enable_viz: bool = False, sim_out: str = "sim_out.bin", stop_time:float=None):
        '''
        OPTIONAL
        Executes the simulation

        Parameters:
            enable_viz = Saves the sim run as visualization
            stop_time - [sec] Simulation stop time, defaults to a complete orbit
        '''
        if enable_viz:
            viz = vizSupport.enableUnityVisualization(self.scSim,
            "DynamicsTask", self.scObject, rwEffectorList = self.rwStateEffector, saveFile=sim_out)

        self.scSim.SetProgressBar(True)
        self.scSim.InitializeSimulation()
        stop_time = self.T if stop_time is None else stop_time
        self.scSim.ConfigureStopTime(macros.sec2nano(stop_time))
        self.scSim.ExecuteSimulation()
    
    def mc_callback(self, data, rp):
        '''
        IGNORE
        '''
        return


# Module-level references used by the MC Controller.  They must live at module
# scope so that copy.deepcopy() can serialise them by name rather than trying
# to pickle the SWIG objects they point to.
_mc_scSim = None
_mc_stop  = 0.0

def _mc_create():
    """Return the shared scSim instance for each MC run."""
    return _mc_scSim

def _mc_execute(scSim):
    """Execute one MC run on the given scSim."""
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(_mc_stop))
    scSim.ExecuteSimulation()


class MonteCarlo(object):

    def __init__(self, scSim, num_sims, num_threads, stop_time, output_dir="mc_output"):
        self.scSim = scSim
        self.num_sims = num_sims
        self.num_threads = num_threads
        self.stop_time = stop_time

        self.mc = Controller()
        self.mc.setArchiveDir(output_dir)

    def add_dispersions(self, dispersions, dispersion_type, bounds):
        self.mc.setShouldDisperseSeeds(True)
        for disp, disp_type, bound in zip(dispersions, dispersion_type, bounds):
            self.mc.addDispersion(disp_type(disp, *bound))

    def build_and_execute(self):
        global _mc_scSim, _mc_stop
        _mc_scSim = self.scSim
        _mc_stop  = self.stop_time

        self.mc.setSimulationFunction(_mc_create)   # module-level → deepcopy-safe
        self.mc.setExecutionFunction(_mc_execute)   # module-level → deepcopy-safe
        self.mc.setExecutionCount(self.num_sims)
        self.mc.setThreadCount(self.num_threads)
        self.mc.setShowProgressBar(True)

        self.mc.executeSimulations()



