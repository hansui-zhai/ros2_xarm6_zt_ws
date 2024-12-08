from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import sys
import os.path as osp   
from ament_index_python.packages import get_package_share_directory
sys.path.append(osp.join(get_package_share_directory('nerf_data_recorder'), 'launch'))
import rs_launch
from moveit_configs_utils import MoveItConfigs
import yaml

D405_launch_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                          {'name': 'camera_namespace',             'default': 'camera', 'description': 'camera namespace'},
                          {'name': 'device_type',                  'default': "d405", 'description': 'choose device by type'},
                          {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                          {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                          {'name': 'align_depth.enable',           'default': 'true', 'description': 'enable align depth filter'},
                          {'name': 'enable_sync',                  'default': 'true', 'description': 'enable sync mode'},
                         ]


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def get_urdf():
    # 指定装置 URDF 的路径
    xarm6_with_D405_urdf_path=osp.join(get_package_share_directory("nerf_data_recorder"),"urdf","xarm6_with_D405.urdf")
    with open(xarm6_with_D405_urdf_path,'r') as urdf_file:
        robot_description=urdf_file.read()
    return robot_description

def get_srdf():
    # 指定装置 SRDF 的路径
    xarm6_with_D405_srdf_path=osp.join(get_package_share_directory("nerf_data_recorder"),"config","xarm6_with_D405.srdf")
    with open(xarm6_with_D405_srdf_path,'r') as urdf_file:
        robot_description_semantic=urdf_file.read()
    return robot_description_semantic


def get_rviz_config():
    xarm6_with_D405_rviz_config=osp.join(get_package_share_directory("nerf_data_recorder"),"rviz","xarm6_with_D405.rviz")
    return xarm6_with_D405_rviz_config

def get_ros2_control_params_path():
    ros2_control_params_path=osp.join(get_package_share_directory("nerf_data_recorder"),"config","xarm6_controllers.yaml")
    return ros2_control_params_path

def get_xarm_params_path():
    xarm_params_path=osp.join(get_package_share_directory("nerf_data_recorder"),"config","xarm_params.yaml")
    return xarm_params_path

def generate_launch_description():    
    # 创建 robot_state_publisher_node 用于广播机器人的 urdf 信息
    robot_state_publisher_node = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[{"robot_description":get_urdf()}],
        )
    
    # 创建一个 joint_state_publisher 虚拟的关节状态广播器 （如果没有连接机械臂的话）
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[{"robot_description":get_urdf(), "rate":50},]
    #     )
    
    # 创建一个 D405 相机节点，进行数据广播
    # 本地配置
    D405_declare_params=rs_launch.declare_configurable_parameters(D405_launch_parameters)
    # 全部配置
    params=rs_launch.configurable_parameters
    config_declare_params=rs_launch.declare_configurable_parameters(params)
    params_dict=rs_launch.set_configurable_parameters(params)    

    d405_camera_node= Node(
        package='realsense2_camera',
        namespace=LaunchConfiguration('camera_namespace'),
        name=LaunchConfiguration('camera_name'),
        executable='realsense2_camera_node',
        parameters=[params_dict],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
        )
    
    
    # 创建 xarm6 的 control 节点，广播实际机械臂目前的状态数据
    ros2_control_params = get_ros2_control_params_path()
    robot_params = get_xarm_params_path()
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {"robot_description":get_urdf()},
            ros2_control_params,
            robot_params,
        ],
        output='screen',
    )
    
    joint_state_publish_node=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager", "/controller_manager"],
    )
    
    xarm6_traj_controller_node=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm6_traj_controller","--controller-manager", "/controller_manager"],
    )
    
    
    
    # 创建一个 moveit_group 节点
    package_share_path = get_package_share_directory("nerf_data_recorder")
    moveit_config_dict={
        'robot_description':get_urdf(),
        'robot_description_semantic':get_srdf(),
        'robot_description_kinematics':load_yaml(osp.join(package_share_path,'config','kinematics.yaml')),
    }

    moveit_config_dict.update(load_yaml(osp.join(package_share_path,'config','planning_pipelines.yaml')))   #  planning_pipelines
    moveit_config_dict.update(load_yaml(osp.join(package_share_path,'config','trajectory_execution.yaml'))) #  trajectory_execution
    moveit_config_dict.update(load_yaml(osp.join(package_share_path,'config','planning_scene_monitor.yaml'))) #  planning_scene_monitor
    moveit_config_dict.update({}) #  sensors_3d
    moveit_config_dict.update({}) # move_group_capabilities
    moveit_config_dict['robot_description_planning']= load_yaml(osp.join(package_share_path,'config','joint_limits.yaml')) # joint_limits
    moveit_config_dict.update({}) # moveit_cpp
    moveit_config_dict.update({}) # pilz_cartesian_limits
    moveit_group_node=Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'use_sim_time': False},
        ],
    )
    
    
    
    # 创建一个 moveit_planner 节点
    move_group_interface_params = {
        'robot_description': moveit_config_dict['robot_description'],
        'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
        'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
    }
    xarm_planner_node = Node(
        name='xarm_planner_node',
        package='xarm_planner',
        executable='xarm_planner_node',
        output='screen',
        parameters=[
            move_group_interface_params,
            {
                'robot_type': 'xarm',
                'dof': 6,
                'prefix': ''
            },
        ],
    )
    
    
    # 创建一个 rviz 节点进行可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', get_rviz_config()],
        parameters=[
            {
                'robot_description': moveit_config_dict['robot_description'],
                'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
                'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
                'robot_description_planning': moveit_config_dict['robot_description_planning'],
                'planning_pipelines': moveit_config_dict['planning_pipelines'],
            },
            {
                'use_sim_time': False}
            ]
        )
    
    
    
    # 创建一个 
    return LaunchDescription(
        D405_declare_params+
        config_declare_params+
        [robot_state_publisher_node,
         ros2_control_node,
         joint_state_publish_node,
         xarm6_traj_controller_node,
         moveit_group_node,
         xarm_planner_node,
         rviz_node, 
         d405_camera_node])
