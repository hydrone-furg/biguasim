import numpy as np

from biguasim.dynamics import uav, uuv, usv, hauv
from biguasim.dynamics.marine import MarinePlant, ROVControl, SurfaceVesselControl

# Marine vehicles' controller ('controller' param): 'original' is the vehicle family's own cascade (uuv/usv), adapted
# to the marine plant; 'marine' is ROVControl / SurfaceVesselControl, written with the plant.
CONTROLLERS = ('original', 'marine')


def _select_controller(model, original, marine):
    choice = model._marine.get('controller', 'original')
    if choice not in CONTROLLERS:
        raise ValueError(f"{type(model).__name__}: controller must be one of {CONTROLLERS}, got {choice!r}")
    return marine if choice == 'marine' else original

    
class BlueBoat(MarinePlant, SurfaceVesselControl, usv.Catamaran):                                                                   
    _params = {
        # 'rho' : 1026,      # Water density
        'rho' : 1026,

        'mass' : 17.0,         # kg: 14.5 kg spec without batteries + 2 batteries (estimate)
        #Params to each vessel
        'length' : 1.195,      # m
        'width' : 0.17,      # m
        'height' : 0.376,      # m

        'I' : np.diag([0.5, 10, 11]),

         'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([-0.5475, 0.285, -0.276]),        # Rotor 1 position
            'r2': np.array([-0.5475, -0.285, -0.276]),         # Rotor 2 position            
        },

        'rotor_directions': np.array([[1, 0, 0],
                                      [1, 0, 0]]),


        'k_eta' : 3.55e-4,  # Rotor torque constant
        'k_m' : 1.12e-5,    # Rotor momentum constant

        'rotor_speed_min': -295.75,   # minimum rotor speed, rad/s
        'rotor_speed_max': 288.08,    # maximum rotor speed, rad/s


        # Lower level controller properties (for higher level control abstractions)
        'k_vx': 1,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        # Yaw: M = kd (kp e - r), stiffness kd*kp, damping kd. Designed on the marine plant (2026-10-14) for wn 1 rad/s,
        # zeta 1 with Iz + I_A = 11 + 1 (the two thrusters give at most ~17 N*m): K = 12 N*m/rad, c = 24 N*m*s.
        # (Was 1.2 / 1.2: zeta 0.05, the heading rang back and forth ~45 deg after a turn.)
        'kp_yaw': 0.5,
        'kd_yaw': 24.0,

        'kp_pos': .1,            # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': .1,            # The attitude D gain (for cmd_pos_yaw)

        # ---- Marine plant (dynamics.marine), approved 2026-10-03 ----
        # Blue Robotics spec: 14.5 kg without batteries, 15 kg max payload, 160 mm freeboard
        # (2 batteries), 320 mm max draft, 3 m/s max speed with 2 x 8.2 kgf bollard thrust.
        # 'mass' above: 14.5 kg + 2 batteries (~1.25 kg each, estimate).
        'draft_reference': (32.0, 0.32),     # (kg, m): max-payload displacement and draft (spec)
        'cg_above_keel': 0.10,               # m, placeholder (no spec): CG low in the hulls
        'added_mass': [0.7, 15.0, 15.0, 0.2, 1.0, 1.0],    # estimates (slender twin hulls)
        'linear_damping': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'drag_coefficients': [0.285, 1.0, 1.0],  # C_x: full thrust (2 x 29.4 N here) gives the 3 m/s spec speed; bluff sides
        'controller': 'original',            # 'original' (usv.Catamaran) or 'marine' (SurfaceVesselControl)
    }            

    _scheme = 2
                                                                                               
                                                                                               
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueBoat._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
        self._init_marine(params or BlueBoat._params)
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params     

    def get_cmd_motor_speeds(self, state: dict, control: dict):
        controller = _select_controller(self, usv.Catamaran, SurfaceVesselControl)
        return controller.get_cmd_motor_speeds(self, state, control)
    
class BlueROV2(MarinePlant, ROVControl, uuv.HexaCopterFiveDoF):                                                                   
    _params = {
        'rho' : 997,      # Water density

        #spheroid
        'mass' : 10.5,
        'length' : 0.4571,      # m
        'width' : 0.3381,      # m
        'height' : 0.2539,      # m

        'I' : np.diag([0.26, 0.23, 0.37]),

        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.0, 0.22, 0.08]),        # Rotor 1 position
            'r2': np.array([0.0, -0.22, 0.08]),         # Rotor 2 position
            'r3': np.array([0.156, 0.111, 0.045]),    # Rotor 3 position
            'r4': np.array([0.156, -0.111, 0.045]),     # Rotor 4 position
            'r5': np.array([-0.156, 0.111, 0.045]),      # Rotor 5 position
            'r6': np.array([-0.156, -0.111, 0.045]),     # Rotor 6 position
            
        },

        'rotor_directions': np.array([[0, 0, 1],
                            [0, 0, 1],
                            [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
                            [np.cos(np.pi/4), np.sin(np.pi/4), 0],
                            [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
                            [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0]]),
        
        

        'k_eta' : 3.8e-4,  # Rotor torque constant
        'k_m' : 5.3e-6,    # Rotor momentum constant

        'rotor_speed_min': -278.9,   # minimum rotor speed, rad/s
        'rotor_speed_max': 278.9, # maximum rotor speed, rad/s

        # Lower level controller properties (for higher level control abstractions)
        'k_vxy': 1,           # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'k_vz': 60.0,           # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        # Roll 'rate PID' (setpoint 10 w): acts as rate damping c = 9 kp and extra stiffness k = 9 ki on top of the
        # righting moment W*BG. Designed on the marine plant (2026-10-14) for wn 4 rad/s, zeta 1 (holds < ~4 deg
        # against cruise moments; wn 2 left ~14 deg): I + I_A = 0.26 + 0.151, W*BG = 1.03 N*m/rad
        # -> c = 3.29 N*m*s, k = 5.55 N*m/rad; c*dt/I = 0.27 at 30 Hz.
        # (Was 10 / .1 / .01, tuned on the former plant whose thrust moments were ~14x weak and of opposite sign:
        # c*dt/I = 7.3 on the real plant, saturating the vertical thrusters.)
        'kp_roll': 0.3653,
        'ki_roll': 0.6163,
        'kd_roll': 0.0,

        # Yaw: M = kd (kp e - r), stiffness kd*kp, damping kd. Designed on the marine plant (2026-10-14) for wn 4 rad/s,
        # zeta 1 with Iz + I_A = 0.37 + 0.178: K = 8.77 N*m/rad, c = 4.38 N*m*s; c*dt/I = 0.27 at 30 Hz.
        # (Was 1 / 1, tuned on the former plant: stiffness 1 N*m/rad let the heading drift ~40 deg while moving.)
        'kp_yaw': 2.0,
        'kd_yaw': 4.384,

        'kp_pos': 1,            # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': 1,          # The attitude D gain (for cmd_pos_yaw)

        # ---- Marine plant (dynamics.marine), approved 2026-10-03 ----
        # Blue Robotics spec: 10-11 kg in air with ballast, +0.2 kg net buoyancy.
        'volume': (10.5 + 0.2) / 997,        # m^3, displaced: net +0.2 kg at rho 997
        'cg_below_cb': 0.01,                 # m (von Benzon et al. 2022, CAD)
        # von Benzon et al. 2022 (Heavy configuration) scaled by displaced volume (0.0107 / 0.0134).
        'added_mass': [5.09, 5.70, 14.94, 0.151, 0.108, 0.178],
        'linear_damping': [13.7, 0.0, 33.0, 0.0, 0.8, 0.0],
        # Panel drag coefficients calibrated on the BlueROV2Heavy hulls to von Benzon's measured quadratic
        # damping (X_u|u| 141, Y_v|v| 217, Z_w|w| 190 N s^2/m^2); same shape coefficients on this box.
        'drag_coefficients': [2.239, 3.885, 1.532],
        # ROVControl: quadratic damping (N s^2/m^2, body x/y/z) fed forward at the velocity setpoint.
        'ctrl_drag_ff': [96.7, 215.2, 118.8],
        'controller': 'original',            # 'original' (uuv.HexaCopterFiveDoF) or 'marine' (ROVControl)
    }            

    _scheme = 2                                                                               
                                                                                               
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueROV2._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
        self._init_marine(params or BlueROV2._params)
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params                 

    def get_cmd_motor_speeds(self, state: dict, control: dict):
        controller = _select_controller(self, uuv.HexaCopterFiveDoF, ROVControl)
        return controller.get_cmd_motor_speeds(self, state, control)
                             

class BlueROV2Heavy(MarinePlant, ROVControl, uuv.OctaCopterSixDoF):                                                                   
    _params = {
        'rho' : 997,      # Water density

        #spheroid
        'mass' : 13.5,         # kg, measured (von Benzon et al. 2022)
        'length' : 0.4571,      # m
        'width' : 0.3381,      # m
        'height' : 0.5539,      # m

        'I' : np.diag([0.26, 0.23, 0.37]),

        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.22, 0.22, 0.08]),          # Rotor 1 position
            'r2': np.array([0.22, -0.22, 0.08]),         # Rotor 2 position
            'r3': np.array([-0.22, 0.22, 0.08]),         # Rotor 3 position
            'r4': np.array([-0.22, -0.22, 0.08]),        # Rotor 4 position
            'r5': np.array([0.156, 0.111, 0.045]),       # Rotor 5 position
            'r6': np.array([0.156, -0.111, 0.045]),      # Rotor 6 position
            'r7': np.array([-0.156, 0.111, 0.045]),      # Rotor 7 position
            'r8': np.array([-0.156, -0.111, 0.045]),     # Rotor 8 position
            
        },

        'rotor_directions': np.array([[0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
                                    [np.cos(np.pi/4), np.sin(np.pi/4), 0],
                                    [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
                                    [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0]     ]),

        'k_eta' : 3.8e-4,  # Rotor torque constant
        'k_m' : 5.3e-6,    # Rotor momentum constant

        'rotor_speed_min': -278.9,   # minimum rotor speed, rad/s
        'rotor_speed_max': 278.9, # maximum rotor speed, rad/s

        # Lower level controller properties (for higher level control abstractions)
        'k_vxy': 1,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw)
        'k_vz': 30,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw)

        # Roll/pitch 'rate PIDs' (setpoint 10 w): rate damping c = 9 kp, extra stiffness k = 9 ki on top of W*BG
        # (1.32 N*m/rad). Designed on the marine plant (2026-10-14) for wn 4 rad/s, zeta 1 (holds < ~4 deg against
        # cruise moments; wn 2 left ~14 deg); c*dt/I = 0.27 at 30 Hz.
        # roll: I + I_A = 0.449 -> c = 3.59, k = 5.86;  pitch: I + I_A = 0.365 -> c = 2.92, k = 4.52.
        # (Were 10 / .1 / .01, tuned on the former plant whose thrust moments were ~14x weak and of opposite sign;
        # the roll ones were also hard-coded in OctaCopterSixDoF, which now reads these.)
        'kp_roll': 0.3991,
        'ki_roll': 0.6511,
        'kd_roll': 0.0,

        'kp_pitch': 0.3244,
        'ki_pitch': 0.5018,
        'kd_pitch': 0.0,
        
        # Yaw: M = kd (kp e - r), stiffness kd*kp, damping kd. Designed on the marine plant (2026-10-14) for wn 4 rad/s,
        # zeta 1 with Iz + I_A = 0.37 + 0.222: K = 9.47 N*m/rad, c = 4.74 N*m*s; c*dt/I = 0.27 at 30 Hz.
        # (Was 1 / 1, tuned on the former plant: stiffness 1 N*m/rad let the heading drift ~45 deg while moving.)
        'kp_yaw': 2.0,
        'kd_yaw': 4.736,

        'kp_pos': 1,           # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': 1,           # The attitude D gain (for cmd_pos_yaw)
                                                                                               
        # ---- Marine plant (dynamics.marine), approved 2026-10-03 ----
        # von Benzon et al. 2022, "An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater
        # Robot" (J. Mar. Sci. Eng.), Table A1, Heavy configuration (measured / CAD).
        'volume': 0.0134,                    # m^3, displaced (measured)
        'cg_below_cb': 0.01,                 # m (CAD)
        'added_mass': [6.36, 7.12, 18.68, 0.189, 0.135, 0.222],
        'linear_damping': [13.7, 0.0, 33.0, 0.0, 0.8, 0.0],
        # Panel drag coefficients reproducing the measured quadratic damping on these hulls.
        'drag_coefficients': [2.239, 3.885, 1.532],
        # ROVControl: quadratic damping (N s^2/m^2, body x/y/z) fed forward at the velocity setpoint.
        'ctrl_drag_ff': [141.0, 217.0, 190.0],
        'controller': 'original',            # 'original' (uuv.OctaCopterSixDoF) or 'marine' (ROVControl)
    }

    _scheme = 2
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueROV2Heavy._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
        self._init_marine(params or BlueROV2Heavy._params)
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return BlueROV2Heavy._params      

    def get_cmd_motor_speeds(self, state: dict, control: dict):
        controller = _select_controller(self, uuv.OctaCopterSixDoF, ROVControl)
        return controller.get_cmd_motor_speeds(self, state, control)

class DjiMatrice(uav.QuadCopterX):
    # DJI Matrice 300 RTK. Spec values (DJI datasheet): 6.3 kg with two TB60 batteries (3.6 kg without),
    # 9 kg max takeoff, 895 mm diagonal wheelbase, 23 m/s max speed (S-mode) at a 30° max pitch,
    # 15 m/s max wind resistance, 6 / 7 m/s max ascent / descent.
    _params = {
        # Inertial properties
        'mass': 6.3,            # kg, M300 RTK with two TB60 batteries (spec)

        'rho' : 1.225,      # Air density, kg/m^3 (used by the mesh panel drag)

        # Not measured for the M300; the attitude controllers scale by I, so closed-loop flight
        # does not depend on it, only the response to disturbance moments does.
        'I' : np.diag([0.07, 0.07,  0.13]),

        # Geometric properties, all vectors relative to center of mass
        'd' : 0.4475,           # Arm length: half the 895 mm diagonal wheelbase

        # Motor hubs measured on the M300 collision mesh (DynamicsSensor ExportGeometry), relative to
        # the engine's center of mass: 0.46 m (front) / 0.44 m (rear) from the center. The front arms
        # sit wider than the rear ones, as on the real airframe.
        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.283, 0.361, 0.109]),       # Rotor 1 position, front-left
            'r2': np.array([0.283, -0.361, 0.109]),      # Rotor 2 position, front-right
            'r3': np.array([-0.308, -0.321, 0.095]),     # Rotor 3 position, rear-right
            'r4': np.array([-0.308, 0.321, 0.095]),      # Rotor 4 position, rear-left
        },
        
        'k_eta' : 3.55e-4, 
        'k_m' : 1.12e-5,   

        'rotor_directions': np.array([1, -1, 1, -1]),  # Rotor spin directions (+1 for CW, -1 for CCW)
        'rotor_speed_min': 0,   # minimum rotor speed, rad/s
        # Thrust-to-weight 2 at the 9 kg max takeoff weight (4 k_eta w^2 = 176.5 N); hover at
        # 6.3 kg is 209 rad/s.
        'rotor_speed_max': 353.0, # maximum rotor speed, rad/s

        # Frame aerodynamic properties, used without exported geometry. Scaled so the vehicle
        # reaches its 23 m/s spec speed at its 30° max pitch (drag 35.7 N there).
        'c_Dx': 0.052,          # parasitic drag coefficient in body x-axis, N/(m/s)^2
        'c_Dy': 0.052,          # parasitic drag coefficient in body y-axis, N/(m/s)^2
        'c_Dz': 0.078,          # parasitic drag coefficient in body z-axis, N/(m/s)^2

        # Dimensionless drag coefficients of the collision-mesh drag panels (used once the
        # DynamicsSensor exports the geometry): axial drag = ½ ρ C_D A u², with the Matrice's
        # convex hulls. Chosen so drag along the body axes reproduces c_Dx, c_Dy, c_Dz above
        # (with the exposure blended over nearby directions, see QuadCopterX._panel_drag).
        'C_Dx': 0.992,
        'C_Dy': 0.773,
        'C_Dz': 1.096,

        # Apply the moments of the panel drag about the CoM. Off by default (untested with the
        # current gains).
        'drag_moments': False,

        # Lower level controller properties (for higher level control abstractions).
        # Sized for the engine's 20 Hz loop with one tick of delay (attitude bandwidth ~3 rad/s).
        'k_v': 1,              # The *world* velocity P gain (for cmd_vel)
        'kp_att': 9.0,            # The attitude P gain (1/s^2): ωn = 3 rad/s
        'kd_att': 4.8,            # The attitude D gain (1/s): ζ = 0.8

        'kp_yaw': 1,            # The attitude P gain (for cmd_vel_yaw)
        'kd_yaw': 0.1,          # The attitude D gain (for cmd_vel_yaw)

        # cmd_pos_yaw: PID on position, acceleration limited to the vehicle's envelope.
        'kp_pos': 0.8,            # 1/s^2
        'kd_pos': 2.5,            # 1/s
        'ki_pos': 0.2,            # 1/s^3; cancels steady wind drag
        'pos_integral_accel_limit': 3.0,   # m/s^2: cap on ki_pos * integral (15 m/s wind needs ~2.1)
        'pos_integral_radius': 5.0,        # m: integrate only this close to the target (anti-windup)
        'max_tilt_deg': 30.0,              # spec max pitch
        'max_vertical_accel': 4.0,         # m/s^2

    }

    _scheme = 1
    
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):
        super().__init__(
                        batch_size, 
                        params= params or DjiMatrice._params, 
                        device=device, 
                        control_abstraction=control_abstraction)
        
        self._params = params

    @property
    def params(self) -> dict: 
        return self._params     
                                                                                                    
class Hydrone(hauv.OctaCopterFourAirFourWaterCross):                                                                   
    _params = {
        'rho_air' : 1.225,      # Air density, kg/m^3 (was 1225: 1000x, gave ~1.7 kg of added mass and ~21 N of drag at 1 m/s in air)
        'rho_water' : 997,      # Water density

        #spheroid
        'mass' : 2.7,
        'l' : 0.33,      
        'd' : 0.27,     

        # 'rc': 0.0675,
        'rc': 0.0875,
        'dd': 0.02,

        'I' : np.diag([0.014464898, 0.028817961,  0.015410484]),
        

        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.25, 0, 0.03]),       # Rotor 1 position
            'r2': np.array([0, -0.25, 0.03]),       # Rotor 2 position
            'r3': np.array([-0.25, 0, 0.03]),      # Rotor 3 position
            'r4': np.array([0, 0.25, 0.03]),      # Rotor 4 position
            'r5': np.array([0.25, 0,  -0.03]),       # Rotor 1 position
            'r6': np.array([0, -0.25, -0.03]),       # Rotor 2 position
            'r7': np.array([-0.25, 0, -0.03]),      # Rotor 3 position
            'r8': np.array([0, 0.25,  -0.03]),      # Rotor 4 position
        },

        'rotor_directions': np.array([[0,0,1], # rotor axis orientation
                                    [0,0,1],
                                    [0,0,1],
                                    [0,0,1],
                                    [0,0,-1],
                                    [1,0,0],
                                    [0,0,-1],
                                    [1,0,0]]), 

        #air - water
        'k_eta' : np.array([3.55e-4,  3.8e-4]), 
        'k_m' : np.array([1.12e-5,  5.3e-6]),   

        'rotor_speed_min': np.array([0,  -278.9]),   # minimum rotor speed, rad/s
        'rotor_speed_max': np.array([592.4,  278.9]), # maximum rotor speed,

        # ---- Controller gains, retuned on the corrected plant (2026-10-14) ----
        # The former gains were tuned on a plant with the opposite roll sign and underwater moments ~70x too weak
        # (roll 0.00012 vs pitch 0.02, kp_air_x 100 vs kp_air_y 1, ...): on the r x F plant they diverged in ticks.
        # Attitude PIDs act on the angle error (P: I*wn^2, N*m/rad; D on its derivative, -rate: 2*zeta*wn*I).
        # Air: I = diag(0.0145, 0.0288, 0.0154) (added inertia in air negligible).
        #   roll/pitch wn 6 rad/s, zeta 0.8 (wn*dt = 0.2 at 30 Hz); yaw wn 3, zeta 1 (reaction-torque authority).
        'kp_air_roll' : 0.5207,
        'ki_air_roll' : 0,
        'kd_air_roll' : 0.1389,

        'kp_air_pitch' : 1.0374,
        'ki_air_pitch' : 0,
        'kd_air_pitch' : 0.2767,

        'kp_air_yaw' : 0.1387,
        'ki_air_yaw' : 0,
        'kd_air_yaw' : 0.0925,

        # Velocity -> acceleration (1/s), ~4x slower than the attitude. No horizontal integral: there is no steady
        # horizontal force to cancel in still air, and 0.3 made the position loop swing +-0.8 m with a ~20 s period.
        'kp_air_vx' : 1.5,
        'ki_air_vx' : 0,
        'kd_air_vx' : 0,

        'kp_air_vy' : 1.5,
        'ki_air_vy' : 0,
        'kd_air_vy' : 0,

        'kp_air_vz' : 2.0,
        'ki_air_vz' : 0.5,
        'kd_air_vz' : 0,

        # Position -> velocity (1/s), 3x slower than the velocity loop.
        'kp_air_x' : 0.5,
        'ki_air_x' : 0,
        'kd_air_x' : 0,

        'kp_air_y' : 0.5,
        'ki_air_y' : 0,
        'kd_air_y' : 0,

        'kp_air_z' : 0.5,
        'ki_air_z' : 0,
        'kd_air_z' : 0,

        # Water: I + added inertia (2/5 * 1.40 kg * rc^2 = 0.0043): pitch 0.0331, yaw 0.0197; wn 3 rad/s, zeta 1.
        'kp_water_vx' : 1.0,
        'kp_water_vz' : 1.5,

        'kp_water_pitch' : 0.2979,
        'ki_water_pitch' : 0,
        'kd_water_pitch' : 0.1986,

        'kp_water_yaw' : 0.1773,
        'ki_water_yaw' : 0,
        'kd_water_yaw' : 0.1182,

        'kp_water_pos' : 0.5,
    }            
                                                                                               
    _scheme = 1

    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params=None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or Hydrone._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params      
    
class TorpedoAUV(uuv.TorpedoAUV):
    _params = {
        #Dynamics
        "mass": 37,             # kg
        "length": 2.1,          # m
        "volume": 0.03636,      # m^3 displaced: +0.3 kg net buoyancy at 1026 kg/m^3
        "rho": 1026,
        "diam": 0.19,
        "r_bg": [0, 0, 0.02],   # CG from the CO (hull centroid = CB), NED: 2 cm below the CB
        "r_bb": [0, 0, 0],
        "r44": 0.3,
        "Cd": 0.42,
        "T_surge": 20,
        "T_sway": 20,
        "zeta_roll": 0.3,
        "zeta_pitch": 0.8,
        "T_yaw": 1,
        "K_nomoto": 5.0 / 20.0,

        #Actuador
        "fin_area": 0.00697,
        "fin_center": 0.07,
        "deltaMax_fin_deg": 20,
        "nMax": 2000,
        "T_delta": 0.1,
        "T_n": 0.1,
        "CL_delta_r": 0.5,
        "CL_delta_s": 0.7,
        
        #Control
        #depth
        'wn_d_z': 0.2,
        'Kp_z': 0.1,
        'T_z': 100,
        'Kp_theta': 5.0,
        'Kd_theta': 2.0,
        'Ki_theta': 0.3,
        'K_w':  5.0,
        'theta_max_deg': 30,

        #heading
        'wn_d': 1.2,
        'zeta_d': 0.8,
        'r_max': 0.9,
        'lam': 0.1,
        'phi_b': 0.1,
        'K_d': 0.5,
        'K_sigma': 0.05,

        #surge
        'kp_surge': 400.0,
        'ki_surge': 50.0,
        'kd_surge': 30.0,

        "rotor_speed_min": -1525, #rpm
        "rotor_speed_max": 1525, #rpm
    }

    _scheme = 1

    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_rudders_sterns_motor_speed', params = None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or TorpedoAUV._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params

class ModelsFactory:
    _types = {
        'BlueBoat' : BlueBoat,
        'BlueROV2' : BlueROV2,
        'BlueROV2Heavy' : BlueROV2Heavy,
        'DjiMatrice' : DjiMatrice,
        'Hydrone' : Hydrone,
        'TorpedoAUV' : TorpedoAUV
    }

    @classmethod
    def build_model(cls, agent_type : str):
        return cls._types[agent_type]


#começar por aqui
        



























