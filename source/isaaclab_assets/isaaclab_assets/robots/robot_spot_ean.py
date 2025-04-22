import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetLSTMCfg, DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.actuators import DelayedPDActuatorCfg, RemotizedPDActuatorCfg
from isaaclab.actuators import ImplicitActuatorCfg

ANYDRIVE_3_SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=[".*"],
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
    stiffness={".*": 40.0},
    damping={".*": 5.0},
)

joints = [
    'joint_front_left_shoulder',
    'joint_front_right_shoulder',
    'joint_rear_left_shoulder',
    'joint_rear_right_shoulder',
    'joint_front_left_leg',
    'joint_front_left_foot',
    'joint_front_right_leg',
    'joint_front_right_foot',
    'joint_rear_left_leg',
    'joint_rear_left_foot',
    'joint_rear_right_leg',
    'joint_rear_right_foot'
]

QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        #usd_path="/home/manh/Projects/IsaacLab/source/isaaclab_assets/data/spotmicroaiean_inercia.usd",
        usd_path="source/isaaclab_assets/data/spot_micro.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            rigid_body_enabled=True,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        scale=(2.5, 2.5, 2.5),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,  
            solver_position_iteration_count=32,  
            solver_velocity_iteration_count=16,   
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, .7),  
        #joint_pos={
            #".*_left_shoulder": 0.0,    
            #".*_right_shoulder": 0.0,       
            #".*_leg": -0.5,             
            #".*_foot": 1.0,             
        #},
        joint_vel={".*": 0.0}, 
    ),
    #soft_joint_pos_limit_factor=0.95,  
    actuators={
        "shoulder": DCMotorCfg(
            joint_names_expr=[".*"],
            saturation_effort=120.0,
            effort_limit=1000.0,
            velocity_limit=100.0,
            stiffness=25.,  # Tăng độ cứng để giữ ổn định thân robot
            damping=1.5,    # Damping vừa phải để linh hoạt
        ),
        #"other": DCMotorCfg(
            #joint_names_expr=[
                #'joint_front_left_leg',
                #'joint_front_left_foot',
                #'joint_front_right_leg',
                #'joint_front_right_foot',
                #'joint_rear_left_leg',
                #'joint_rear_left_foot',
                #'joint_rear_right_leg',
                #'joint_rear_right_foot'
            #],
            #saturation_effort=120.0,
            #effort_limit=1000.0,
            #velocity_limit=100.0,
            #stiffness=5.0,  # Tăng độ cứng để giữ ổn định thân robot
            #damping=0.1,    # Damping vừa phải để linh hoạt
        #),
    }
)