from launch_param_builder import ParameterBuilder,load_yaml
from moveit_configs_utils import MoveItConfigs

class MoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = None
    __urdf_package = ''
    __urdf_file_path = ''
    __srdf_file_path = ''
    __robot_description = ''

    def __init__(self,
                 context=None,
                 controllers_name='fake_controllers',
                 **kwargs):
        super().__init__('xarm_moveit_config')
        self.__moveit_configs = MoveItConfigs()
        self.__context = context

        def get_param_str(name, default_val):
            val = kwargs.get(name, default_val)
            return val if isinstance(val, str) else 'false' if val == False else 'true' if val == True else (val.perform(context) if context is not None else val) if isinstance(val, LaunchConfiguration) else str(val)

        def get_list_param_str(name, default_val):
            val = get_param_str(name, default_val)
            return val[1:-1] if context is not None and isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

        robot_ip = get_param_str('robot_ip', '')
        report_type = get_param_str('report_type', 'normal')
        baud_checkset = get_param_str('baud_checkset', True)
        default_gripper_baud = get_param_str('default_gripper_baud', 2000000)
        dof = get_param_str('dof', 7)
        robot_type = get_param_str('robot_type', 'xarm')
        prefix = get_param_str('prefix', '')
        hw_ns = get_param_str('hw_ns', 'xarm')
        limited = get_param_str('limited', False)
        effort_control = get_param_str('effort_control', False)
        velocity_control = get_param_str('velocity_control', False)
        model1300 = get_param_str('model1300', False)
        robot_sn = get_param_str('robot_sn', '')
        attach_to = get_param_str('attach_to', 'world')
        attach_xyz = get_list_param_str('attach_xyz', '0 0 0')
        attach_rpy = get_list_param_str('attach_rpy', '0 0 0')
        mesh_suffix = get_param_str('mesh_suffix', 'stl')
        kinematics_suffix = get_param_str('kinematics_suffix', '')
        ros2_control_plugin = get_param_str('ros2_control_plugin', 'uf_robot_hardware/UFRobotSystemHardware')
        ros2_control_params = get_param_str('ros2_control_params', '')
        add_gripper = get_param_str('add_gripper', False)
        add_vacuum_gripper = get_param_str('add_vacuum_gripper', False)
        add_bio_gripper = get_param_str('add_bio_gripper', False)
        add_realsense_d435i = get_param_str('add_realsense_d435i', False)
        add_d435i_links = get_param_str('add_d435i_links', True)
        use_gazebo_camera = get_param_str('use_gazebo_camera', False)
        add_other_geometry = get_param_str('add_other_geometry', False)
        geometry_type = get_param_str('geometry_type', 'box')
        geometry_mass = get_param_str('geometry_mass', 0.1)
        geometry_height = get_param_str('geometry_height', 0.1)
        geometry_radius = get_param_str('geometry_radius', 0.1)
        geometry_length = get_param_str('geometry_length', 0.1)
        geometry_width = get_param_str('geometry_width', 0.1)
        geometry_mesh_filename = get_param_str('geometry_mesh_filename', '')
        geometry_mesh_origin_xyz = get_list_param_str('geometry_mesh_origin_xyz', '0 0 0')
        geometry_mesh_origin_rpy = get_list_param_str('geometry_mesh_origin_rpy', '0 0 0')
        geometry_mesh_tcp_xyz = get_list_param_str('geometry_mesh_tcp_xyz', '0 0 0')
        geometry_mesh_tcp_rpy = get_list_param_str('geometry_mesh_tcp_rpy', '0 0 0')

        self.__prefix = prefix
        self.__robot_dof = dof
        self.__robot_type = robot_type
        self.__add_gripper = add_gripper
        self.__add_bio_gripper = add_bio_gripper
        self.__controllers_name = (controllers_name.perform(context) if context is not None else controllers_name) if isinstance(controllers_name, LaunchConfiguration) else controllers_name

        self.__urdf_xacro_args = {
            'robot_ip': robot_ip,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'model1300': model1300,
            'robot_sn': robot_sn,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'mesh_suffix': mesh_suffix,
            'kinematics_suffix': kinematics_suffix,
            'ros2_control_plugin': ros2_control_plugin,
            'ros2_control_params': ros2_control_params,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_d435i_links,
            'use_gazebo_camera': use_gazebo_camera,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }
        self.__srdf_xacro_args = {
            'prefix': prefix,
            'dof': dof,
            'robot_type': robot_type,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'add_other_geometry': add_other_geometry,
        }

        self.__urdf_package = Path(get_package_share_directory('xarm_description'))
        self.__urdf_file_path = Path('urdf/xarm_device.urdf.xacro')
        self.__srdf_file_path = Path('srdf/xarm.srdf.xacro')

        self.__robot_description = 'robot_description'

    def robot_description(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load robot description.

        :param file_path: Absolute or relative path to the URDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description loaded.
        """
        if file_path is None:
            robot_description_file_path = self.__urdf_package / self.__urdf_file_path
        else:
            robot_description_file_path = self._package_path / file_path
        mappings = mappings or self.__urdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            try:
                self.__moveit_configs.robot_description = {
                    self.__robot_description: load_xacro(
                        robot_description_file_path,
                        mappings=mappings,
                    )
                }
            except ParameterBuilderFileNotFoundError as e:
                logging.warning('\x1b[33;21m{}\x1b[0m'.format(e))
                logging.warning('\x1b[33;21mThe robot description will be loaded from /robot_description topic \x1b[0m')
        else:
            self.__moveit_configs.robot_description = {
                self.__robot_description: get_xacro_command(
                    str(robot_description_file_path), mappings=mappings
                )
            }

        return self

    def robot_description_semantic(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load semantic robot description.

        :param file_path: Absolute or relative path to the SRDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description_semantic loaded.
        """
        key = self.__robot_description + '_semantic'
        file_path = self._package_path / (file_path or self.__srdf_file_path)
        mappings = mappings or self.__srdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            self.__moveit_configs.robot_description_semantic = {
                key: load_xacro(file_path, mappings=mappings)
            }
        else:
            self.__moveit_configs.robot_description_semantic = {
                key: get_xacro_command(str(file_path), mappings=mappings)
            }
        
        return self

    def robot_description_kinematics(self, file_path = None):
        """Load IK solver parameters.

        :param file_path: Absolute or relative path to the kinematics yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_kinematics loaded.
        """
        key = self.__robot_description + '_kinematics'

        params = [self.__prefix, self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'kinematics.yaml'
                kinematics_yaml = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                kinematics_yaml = load_yaml(file_path) if file_path else {}
            if kinematics_yaml and self.__prefix:
                for name in list(kinematics_yaml.keys()):
                    kinematics_yaml['{}{}'.format(self.__prefix, name)] = kinematics_yaml.pop(name)
            self.__moveit_configs.robot_description_kinematics = {
                key: kinematics_yaml
            }
        else:
            self.__moveit_configs.robot_description_kinematics = {
                key: YamlParameterValue(
                    KinematicsYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof
                    ), value_type=str)
            }

        return self

    def joint_limits(self, file_path = None):
        """Load joint limits overrides.

        :param file_path: Absolute or relative path to the joint limits yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        key = self.__robot_description + '_planning'

        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'joint_limits.yaml'
                joint_limits = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                joint_limits = load_yaml(file_path) if file_path else {}
            joint_limits = joint_limits if joint_limits else {}
            if self.__robot_type != 'lite' and self.__add_gripper in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            elif self.__robot_type != 'lite' and self.__add_bio_gripper in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            if joint_limits and self.__prefix:
                for name in list(joint_limits['joint_limits']):
                    joint_limits['joint_limits']['{}{}'.format(self.__prefix, name)] = joint_limits['joint_limits'].pop(name)
            self.__moveit_configs.joint_limits = {
                key: joint_limits
            }
        else:
            self.__moveit_configs.joint_limits = {
                key: YamlParameterValue(
                    JointLimitsYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper
                ), value_type=str)
            }

        return self

    def moveit_cpp(self, file_path = None):
        """Load MoveItCpp parameters.

        :param file_path: Absolute or relative path to the MoveItCpp yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with moveit_cpp loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'moveit_cpp.yaml'
                moveit_cpp = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                moveit_cpp = load_yaml(file_path) if file_path else {}
            self.__moveit_configs.moveit_cpp = moveit_cpp

        return self

    def trajectory_execution(
        self,
        file_path = None,
        controllers_name = None,
        moveit_manage_controllers = False,
    ):
        """Load trajectory execution and moveit controller managers' parameters

        :param file_path: Absolute or relative path to the controllers yaml file (w.r.t. xarm_moveit_config).
        :param moveit_manage_controllers: Whether trajectory execution manager is allowed to switch controllers' states.
        :return: Instance of MoveItConfigsBuilder with trajectory_execution loaded.
        """
        controllers_name = controllers_name if controllers_name else self.__controllers_name

        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper, controllers_name]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            controllers_name = controllers_name if controllers_name.endswith('.yaml') else '{}.yaml'.format(controllers_name)
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / controllers_name
                controllers_yaml = load_yaml(file_path)
                controllers_yaml = controllers_yaml if controllers_yaml else {}
            else:
                file_path = self._package_path / file_path
                controllers_yaml = load_yaml(file_path) if file_path else {}
            if self.__robot_type != 'lite' and self.__add_gripper in ('True', 'true'):
                gripper_controllers_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / controllers_name)
                if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                    for name in gripper_controllers_yaml['controller_names']:
                        if name in gripper_controllers_yaml:
                            if name not in controllers_yaml['controller_names']:
                                controllers_yaml['controller_names'].append(name)
                            controllers_yaml[name] = gripper_controllers_yaml[name]
            elif self.__robot_type != 'lite' and self.__add_bio_gripper in ('True', 'true'):
                gripper_controllers_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / controllers_name)
                if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                    for name in gripper_controllers_yaml['controller_names']:
                        if name in gripper_controllers_yaml:
                            if name not in controllers_yaml['controller_names']:
                                controllers_yaml['controller_names'].append(name)
                            controllers_yaml[name] = gripper_controllers_yaml[name]

            if controllers_yaml and self.__prefix:
                for i, name in enumerate(controllers_yaml['controller_names']):
                    joints = controllers_yaml.get(name, {}).get('joints', [])
                    for j, joint in enumerate(joints):
                        joints[j] = '{}{}'.format(self.__prefix, joint)
                    controllers_yaml['controller_names'][i] = '{}{}'.format(self.__prefix, name)
                    if name in controllers_yaml:
                        controllers_yaml['{}{}'.format(self.__prefix, name)] = controllers_yaml.pop(name)

            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': controllers_yaml,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False,
                'plan_execution.record_trajectory_state_frequency': 10.0
            }
        else:
            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': YamlParameterValue(
                    ControllersYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper, controllers_name=controllers_name
                    ), value_type=str),
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False,
                'plan_execution.record_trajectory_state_frequency': 10.0
            }
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene = True,
        publish_geometry_updates = True,
        publish_state_updates = True,
        publish_transforms_updates = True,
        publish_robot_description = False,
        publish_robot_description_semantic = False,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # 'planning_scene_monitor': {
            'publish_planning_scene': publish_planning_scene,
            'publish_geometry_updates': publish_geometry_updates,
            'publish_state_updates': publish_state_updates,
            'publish_transforms_updates': publish_transforms_updates,
            'publish_robot_description': publish_robot_description,
            'publish_robot_description_semantic': publish_robot_description_semantic,
            # }
        }
        return self

    def sensors_3d(self, file_path = None):
        """Load sensors_3d parameters.

        :param file_path: Absolute or relative path to the sensors_3d yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'sensors_3d.yaml'
            else:
                file_path = self._package_path / file_path
            if file_path and file_path.exists():
                sensors_data = load_yaml(file_path)
                # TODO(mikeferguson): remove the second part of this check once
                # https://github.com/ros-planning/moveit_resources/pull/141 has made through buildfarm
                if sensors_data and len(sensors_data['sensors']) > 0 and sensors_data['sensors'][0]:
                    self.__moveit_configs.sensors_3d = sensors_data
        
        return self

    def planning_pipelines(
        self,
        default_planning_pipeline = None,
        pipelines = None,
        load_all = True,
        config_folder = None
    ):
        """Load planning pipelines parameters.

        :param default_planning_pipeline: Name of the default planning pipeline.
        :param pipelines: List of the planning pipelines to be loaded.
        :param load_all: Only used if pipelines is None.
                         If true, loads all pipelines defined in config package AND this package.
                         If false, only loads the pipelines defined in config package.
        :return: Instance of MoveItConfigsBuilder with planning_pipelines loaded.
        """
        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            if config_folder is None:
                config_folder = self._package_path / 'config' / robot_name
            else:
                config_folder = self._package_path / config_folder
            if pipelines is None:
                planning_pattern = re.compile('^(.*)_planning.yaml$')
                pipelines = get_pattern_matches(config_folder, planning_pattern)
            pipelines = list(set(pipelines))
             # Define default pipeline as needed
            if not default_planning_pipeline:
                if not pipelines or 'ompl' in pipelines:
                    default_planning_pipeline = 'ompl'
                else:
                    default_planning_pipeline = pipelines[0]

            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )

            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            default_config_folder = self._package_path / 'config' / 'moveit_configs'
            
            for pipeline in pipelines:
                filename = pipeline + '_planning.yaml'
                parameter_file = default_config_folder / filename
                if parameter_file.exists():
                    planning_yaml = load_yaml(parameter_file)
                    planning_yaml = planning_yaml if planning_yaml else {}
                else:
                    planning_yaml = {}

                parameter_file = config_folder / filename
                if parameter_file.exists():
                    pipeline_planning_yaml = load_yaml(parameter_file)
                    pipeline_planning_yaml = pipeline_planning_yaml if pipeline_planning_yaml else {}
                else:
                    pipeline_planning_yaml = {}
                
                if self.__add_gripper in ('True', 'true'):
                    parameter_file = self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / filename
                    if parameter_file.exists():
                        gripper_planning_yaml = load_yaml(parameter_file)
                        if gripper_planning_yaml:
                            pipeline_planning_yaml.update(gripper_planning_yaml)
                elif self.__add_bio_gripper in ('True', 'true'):
                    parameter_file = self._package_path / 'config' / 'bio_gripper' / filename
                    if parameter_file.exists():
                        gripper_planning_yaml = load_yaml(parameter_file)
                        if gripper_planning_yaml:
                            pipeline_planning_yaml.update(gripper_planning_yaml)
                if pipeline_planning_yaml and self.__prefix:
                    for name in list(pipeline_planning_yaml.keys()):
                        if pipeline == 'ompl' and name != 'planner_configs' and name not in planning_yaml:
                            pipeline_planning_yaml['{}{}'.format(self.__prefix, name)] = pipeline_planning_yaml.pop(name)
                
                planning_yaml.update(pipeline_planning_yaml)
                if pipeline == 'ompl' and 'planner_configs' not in planning_yaml:
                    parameter_file = default_config_folder / 'ompl_defaults.yaml'
                    planning_yaml.update(load_yaml(parameter_file))
                self.__moveit_configs.planning_pipelines[pipeline] = planning_yaml
            # # Special rule to add ompl planner_configs
            # if 'ompl' in self.__moveit_configs.planning_pipelines:
            #     ompl_config = self.__moveit_configs.planning_pipelines['ompl']
            #     if os.environ.get('ROS_DISTRO', '') > 'iron':
            #         ompl_config.update({
            #             'planning_plugins': ['ompl_interface/OMPLPlanner'],
            #             'request_adapters': [
            #                 'default_planning_request_adapters/ResolveConstraintFrames',
            #                 'default_planning_request_adapters/ValidateWorkspaceBounds',
            #                 'default_planning_request_adapters/CheckStartStateBounds',
            #                 'default_planning_request_adapters/CheckStartStateCollision',
            #             ],
            #             'response_adapters': [
            #                 'default_planning_response_adapters/AddTimeOptimalParameterization',
            #                 'default_planning_response_adapters/ValidateSolution',
            #                 'default_planning_response_adapters/DisplayMotionPath',
            #             ],
            #         })
            #     else:
            #         ompl_config.update({
            #             'planning_plugin': 'ompl_interface/OMPLPlanner',
            #             'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            #             'start_state_max_bounds_error': 0.1,
            #         })
        else:
            pipelines = list(set(pipelines)) if pipelines else ['ompl']
            default_planning_pipeline = default_planning_pipeline if default_planning_pipeline else 'ompl'
            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )
            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            for pipeline in pipelines:
                self.__moveit_configs.planning_pipelines[pipeline] = YamlParameterValue(
                    PlanningPipelinesYAML(pipeline, package_path=self._package_path, config_folder=config_folder,
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper
                    ), value_type=str)
        
        return self

    def pilz_cartesian_limits(self, file_path = None):
        """Load cartesian limits.

        :param file_path: Absolute or relative path to the cartesian limits file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with pilz_cartesian_limits loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            deprecated_path = self._package_path / 'config' / robot_name / 'cartesian_limits.yaml'
            if deprecated_path.exists():
                logging.warning('\x1b[33;21mcartesian_limits.yaml is deprecated, please rename to pilz_cartesian_limits.yaml\x1b[0m')
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'pilz_cartesian_limits.yaml'
                if not file_path.exists():
                    file_path = self._package_path / 'config' / 'moveit_configs' / 'pilz_cartesian_limits.yaml'
            else:
                file_path = self._package_path / file_path
            key = self.__robot_description + '_planning'
            if file_path.exists():
                self.__moveit_configs.pilz_cartesian_limits = {
                    key: load_yaml(file_path)
                }
        else:
            key = self.__robot_description + '_planning'
            self.__moveit_configs.pilz_cartesian_limits = {
                key: YamlParameterValue(
                    CommonYAML('pilz_cartesian_limits.yaml', package_path=self._package_path, 
                        robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                ), value_type=str)
            }

        return self

    def to_moveit_configs(self):
        """Get MoveIt configs from xarm_moveit_config.

        :return: An MoveItConfigs instance with all parameters loaded.
        """
        if not self.__moveit_configs.robot_description:
            self.robot_description()
        if not self.__moveit_configs.robot_description_semantic:
            self.robot_description_semantic()
        if not self.__moveit_configs.robot_description_kinematics:
            self.robot_description_kinematics()
        if not self.__moveit_configs.planning_pipelines:
            self.planning_pipelines()
        if not self.__moveit_configs.trajectory_execution:
            self.trajectory_execution()
        if not self.__moveit_configs.planning_scene_monitor:
            self.planning_scene_monitor()
        if not self.__moveit_configs.sensors_3d:
            self.sensors_3d()
        if not self.__moveit_configs.joint_limits:
            self.joint_limits()
        # TODO(JafarAbdi): We should have a default moveit_cpp.yaml as port of a moveit config package
        # if not self.__moveit_configs.moveit_cpp:
        #     self.moveit_cpp()
        if 'pilz_industrial_motion_planner' in self.__moveit_configs.planning_pipelines:
            if not self.__moveit_configs.pilz_cartesian_limits:
                self.pilz_cartesian_limits()
        return self.__moveit_configs

    def to_dict(self, include_moveit_configs = True):
        """Get loaded parameters from xarm_moveit_config as a dictionary.

        :param include_moveit_configs: Whether to include the MoveIt config parameters or
                                       only the ones from ParameterBuilder
        :return: Dictionary with all parameters loaded.
        """
        parameters = self._parameters
        if include_moveit_configs:
            parameters.update(self.to_moveit_configs().to_dict())
        return parameters