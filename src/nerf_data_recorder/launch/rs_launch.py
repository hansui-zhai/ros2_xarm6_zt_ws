# 取自 realsence ros 库 ，用于启动官方的 ros 节点
"""Launch realsense2_camera node."""


import os
import yaml
from launch import LaunchDescription,LaunchContext
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': 'camera', 'description': 'namespace for camera'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '0,0,0', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.depth_profile',   'default': '0,0,0', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.color_profile',   'default': '0,0,0', 'description': 'depth_module.exposure'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
                           {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable',            'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'spatial_filter.enable',        'default': 'false', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def get_D405_params():
    return {
        # 基本配置: 
        "camera_name":'camera',                               # 相机设置唯一的名称，用于标识多个相机实例
        "camera_namespace":'camera',                          # 定义相机的命名空间，组织 ROS 话题的层次结构。
        "serial_no":'',                                       # 通过序列号选择特定的设备。如果留空，则默认选择第一个可用设备。
        "usb_port_id" : '',                                   # 通过 USB 端口 ID 选择特定设备。
        "device_type":"d405",                                 # 设备类型（例如 D435、T265）选择设备。
        "config_file":"",                                     # 提供 YAML 配置文件，用于初始化设备的参数。
        "json_file_path":"",                                  # 允许使用 JSON 文件进行高级配置。
        "initial_reset":False,                                # 启动时是否对设备执行复位操作。
        "accelerate_gpu_with_glsl":True,                      # 是否使用 GLSL 加速 GPU。
        "rosbag_filename":"",                                 # Realsense 的 rosbag 文件运行设备，代替实际硬件。
        
        # 日志与输出:
        "log_level":"info",                                   # 设置调试日志级别，可以选择 [DEBUG|INFO|WARN|ERROR|FATAL]。
        "output":"screen",                                    # 节点输出方式，选择为屏幕输出（screen）或日志文件（log）。
        "diagnostics_period":0.0,                             # 发布诊断信息的频率（单位: 秒）。0 表示禁用诊断信息发布。
        
        # 流配置
        "enable_color": True,                                 # 是否启用彩色流（RGB）。
        "rgb_camera.color_profile":"1280x720x30",             # 设置 RGB 相机的分辨率和帧率。
        "rgb_camera.color_format":"RGB8",                     # 设置 RGB 流的格式。
        "rgb_camera.enable_auto_exposure":True,               # 启用/禁用自动曝光。
        
        "enable_depth": True,                                 # 是否启用深度流。
        "depth_module.depth_profile":"1280x720x30",           # 设置深度流的分辨率和帧率。
        "depth_module.depth_format":"Z16",                    # 深度流的格式（通常为 16 位深度值）。
        
        "depth_module.exposure":8500,                         # 手动设置深度模块的曝光值。
        "depth_module.gain":16,                               # 手动设置深度模块的增益值。
        "depth_module.hdr_enabled":False,                     # 是否启用深度模块的 HDR 模式。
        "depth_module.enable_auto_exposure":True,             # 启用/禁用深度模块的自动曝光。
        "depth_module.exposure.1":7500,                       # 深度模块的第一曝光值，用于 HDR 合并滤波器（hdr_merge filter）。
        "depth_module.gain.1":16,                             # 深度模块的第一增益值，用于 HDR 合并滤波器。
        "depth_module.exposure.2":1,                          # 深度模块的第二曝光值，用于 HDR 合并滤波器。
        "depth_module.gain.2":16,                             # 深度模块的第二增益值，用于 HDR 合并滤波器。
        
        "enable_rgbd":False,                                  # 是否开启 RGBD 话题
        "clip_distance":-2.,                                   # 
        "angular_velocity_cov":0.01,
        "linear_accel_cov":0.01,

        
        "depth_module.infra_profile":"1280x720x30",           # 设置红外流的分辨率和帧率。
        "enable_infra":False,                                 # 是否启用红外流（infra0）。
        "depth_module.infra_format":"RGB8",                   # infra0 流的格式。
        "enable_infra1":False,                                # 是否启用红外流（infra1）。
        "depth_module.infra1_format":"Y8",                    # infra1 流的格式。
        "enable_infra2":False,                                # 是否启用红外流（infra2）。
        "depth_module.infra2_format":"Y8",                    # infra2 流的格式。
        
        # IMU 与同步设置
        "enable_sync":False,                                   # 是否启用同步模式（如时间同步）。
        "enable_gyro":False,                                  # 是否启用陀螺仪数据流。
        "enable_accel":False,                                 # 是否启用加速度计数据流。
        "gyro_fps":0,                                         # 设置陀螺仪数据的帧率。
        "accel_fps":0,                                        # 设置加速度计数据的帧率。
        "unite_imu_method":0,                                 # IMU 数据整合方式。0 表示不整合，1 表示复制，2 表示线性插

        # 点云与后处理
        "pointcloud.enable":False,                            # 是否启用点云功能。
        "pointcloud.ordered_pc":False,                        # 是否生成有序点云。
        "pointcloud.stream_filter":2,                         # 点云的纹理流选择
        "pointcloud.stream_index_filter":0,                   # 点云的纹理流索引选择。
        "pointcloud.allow_no_texture_points": False,          # 
        "align_depth.enable":False,                           # 是否启用深度对齐到彩色图像。
        
        # 其他配置
        "publish_tf":True,                                    # 是否发布静态和动态 TF。
        "tf_publish_rate":0.0,                                # 动态 TF 的发布频率（Hz）。
        "wait_for_device_timeout":-1.0,                       # 等待设备连接的超时时间（秒）。
        "reconnect_timeout":6.0,                              # 重连设备的尝试间隔时间（秒）。
        "colorizer.enable":False,                             # 是否启用颜色化滤波器（colorizer filter），通常用于将深度数据可视化为彩色图像。
        "decimation_filter.enable":False,                     # 是否启用降采样滤波器（decimation filter），用于减少深度图的分辨率以优化性能。
        "spatial_filter.enable":False,                        # 是否启用空间滤波器（spatial filter），用于平滑深度图以去除噪声。
        "temporal_filter.enable":False,                       # 是否启用时间滤波器（temporal filter），通过时间上的平滑来减少深度图中的噪声。
        "disparity_filter.enable":False,                      # 是否启用视差滤波器（disparity filter），用于处理视差数据。
        "hole_filling_filter.enable":False,                   # 是否启用孔洞填充滤波器（hole filling filter），用于填充深度图中无效值的区域。
        "hdr_merge.enable":False,                             # 是否启用 HDR 合并滤波器（hdr_merge filter），用于将多曝光数据合并为单一深度图。

    }

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration('output' + param_name_suffix)
    if(os.getenv('ROS_DISTRO') == 'foxy'):
        # Foxy doesn't support output as substitution object (LaunchConfiguration object)
        # but supports it as string, so we fetch the string from this substitution object
        # see related PR that was merged for humble, iron, rolling: https://github.com/ros2/launch/pull/577
        _output = context.perform_substitution(_output)

    return [
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
    ])


