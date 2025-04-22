import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetLSTMCfg, DCMotorCfg, IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.actuators import DelayedPDActuatorCfg, RemotizedPDActuatorCfg



QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path="/localhome/local-hnto/IsaacLab/source/isaaclab_assets/data/spotmicroaiean_inercia.usd",
        # usd_path="/localhome/local-hnto/IsaacLab/spot_micro_rviz/urdf/spot_micro.usd",
        usd_path="/localhome/local-hnto/SpotM2-Jetson/spotMicro-ROS-Melodic-Jetson-Nano/spot_micro_rviz/urdf/spot_micro/spot_micro.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,  # Giữ nguyên để tối ưu
            solver_position_iteration_count=8,  # Tăng để cải thiện độ chính xác va chạm
            solver_velocity_iteration_count=2,  # Thêm lặp vận tốc để mô phỏng chuyển động mượt hơn
        ),
        # scale=(2., 2., 2.)
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),  # Giảm z xuống để chân dễ chạm đất hơn
        joint_pos={
            ".*_shoulder": 0.0,
            ".*_leg": 0.5,
            ".*_foot": -1.0,
        },
        joint_vel={".*": 0.0},  # Vận tốc ban đầu vẫn là 0
    ),
    soft_joint_pos_limit_factor=0.95,  # 95% phạm vi khớp thực tế
    actuators={
        # "shoulder_actuators": DelayedPDActuatorCfg(
        #     joint_names_expr=shoulder_joints,
        #     effort_limit=45.0,
        #     stiffness=60.0,
        #     damping=1.5,
        #     min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
        #     max_delay=2,  # physics time steps (max: 2.0*2=4.0ms)
        # ),
        # "leg_actuators": DelayedPDActuatorCfg(
        #     joint_names_expr=leg_joints,
        #     effort_limit=45.0,
        #     stiffness=60.0,
        #     damping=1.5,
        #     min_delay=0,
        #     max_delay=2,
        # ),
        # "foot_actuators": DelayedPDActuatorCfg(
        #     joint_names_expr=foot_joints,
        #     effort_limit=45.0,
        #     stiffness=60.0,
        #     damping=1.5,
        #     min_delay=0,
        #     max_delay=2,
        # ),
        # "base_legs": IdealPDActuatorCfg(
        #     joint_names_expr=[".*_leg", ".*_foot", ".*_shoulder"],
        #     stiffness    = 3.4 / 0.35,        # ≈ 10 N·m/rad  → có thể nâng tới ~40 khi robot nặng
        #     damping      = 0.05 * 40.0,       # ≈ 2 N·m·s/rad  (5 % k‑p) – bắt đầu ít, tăng dần
        #     effort_limit = 3.5,               # N·m  (ngang stall‑torque 6 V)
        #     velocity_limit = 8.0              # rad/s (> 7.5 để không kìm tốc độ tối đa)
        # ),
        "servos": ImplicitActuatorCfg(
            joint_names_expr=[".*_leg", ".*_foot", ".*_shoulder"],   # regex khớp tên hinge
            stiffness=20.0,                    # k_p  (N·m/rad)
            damping=0.5,                       # k_d  (N·m·s/rad)
            effort_limit=3.4,                  # τ_max
            velocity_limit=8.0,                # |ω|
            armature=0.0,                      # giữ mặc định
        )
    }
)