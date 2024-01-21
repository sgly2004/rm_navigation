import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    
    #=============================1.定位到包的地址=============================================================
    
    # 获取navigation包的目录
    navigation_dir = get_package_share_directory('navigation')
    # 获取nav2_bringup包的目录
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    #=============================2.声明参数，获取配置文件路径===================================================
    
    # 设置使用仿真时间，默认为true，因为在gazebo仿真环境中，时间是通过/clock话题获取的，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    # 设置地图文件路径，默认为map.yaml
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(navigation_dir,'maps','map.yaml'))
    # 设置nav2参数文件路径，默认为nav2.yaml
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(navigation_dir,'param','nav2_omni.yaml'))
    # 设置RViz配置文件路径，默认为nav2_default_view.rviz
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    # 设置urdf
    # default_model_path = os.path.join(navigation_dir, 'src/description/description.urdf')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='navigation').find('navigation')
    default_model_path = os.path.join(pkg_share, 'src/description/description.urdf')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    
    # 使用IncludeLaunchDescription启动nav2_bringup包中的bringup_launch.py文件，并传入参数
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    
    # 启动RViz节点，加载RViz配置文件，并传入参数
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],  # 使用指定的RViz配置文件
            parameters=[{'use_sim_time': use_sim_time}],  # 传递参数，指定是否使用仿真时间
            output='screen')  # 将输出显示在终端屏幕上
    
    #=============================4. 发布机器人的urdf==============
    
    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    # )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )

    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(navigation_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    # gazebo_launch_path = os.path.join(navigation_dir,'launch','gazebo_launch.py')

    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gazebo_launch_path)
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        # joint_state_publisher_node,
        # robot_state_publisher_node,  # 发布机器人的状态
        robot_localization_node,  # 定位
        nav2_bringup_launch,  # 启动导航
        rviz_node,  # 启动rviz
        # gazebo_launch
    ])
