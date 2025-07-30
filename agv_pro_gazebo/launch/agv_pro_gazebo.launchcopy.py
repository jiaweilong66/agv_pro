import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name    = 'agv_pro_gazebo'
    pkg_dir     = get_package_share_directory(pkg_name)

    # 文件路径
    xacro_file  = os.path.join(pkg_dir, 'urdf', 'agv_pro.xacro')
    world_file  = os.path.join(pkg_dir, 'worlds','empty.world')
    rviz_config = os.path.join(pkg_dir, 'rviz',  'agvpro_display.rviz')
    nav2_params = os.path.join(pkg_dir, 'config','nav2_params.yaml')
    ctrl_yaml   = os.path.join(pkg_dir, 'config','agv_controller.yaml')  # 你的 controller yaml

    # 1) 声明 Launch 参数
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'map', 'run_map.yaml'),
        description='Full path to map YAML file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to Nav2 params file'
    )

    # 2) 生成 robot_description 参数
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # 3) Include Gazebo 启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 4) Spawn 机器人模型
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'agv_pro'],
        output='screen'
    )

    # 5) 发布 Joint/TF
    state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    joint_pub = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 6) RViz 可视化
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    # 7) 静态 transform map -> odom
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom'],
        output='screen'
    )

    # 8) 延迟启动 Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map':           LaunchConfiguration('map'),
            'params_file':   LaunchConfiguration('params_file'),
            'autostart':     'true',
            'rviz':          'false'
        }.items()
    )
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2_launch])

    return LaunchDescription([
        # —— 参数声明 —— 
        declare_use_sim,
        declare_map,
        declare_params,

        # —— Gazebo + 机器人 —— 
        gazebo,
        spawn_robot,

        # —— TF & State Publisher —— 
        state_pub,
        joint_pub,
        # —— RViz 可视化 —— 
        rviz,

        # —— 延迟启动 Nav2 —— 
        delayed_nav2,
    ])

