import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import xacro

pkg_folder = 'ms2r1p'
robot_file = 'ms2r1p_amg.urdf.xacro'
rviz_file = 'rviz_config.rviz'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path, 'model', robot_file)
    default_rviz_config_path = os.path.join(pkg_path, 'model', rviz_file)

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

   
    robot_description_config = xacro.process_file(default_model_path)
    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    dxf_parser_node = Node(
        package='ms2r1p',
        executable='dxf_parser_node',
        name='dxf_parser'
    )

    trajectory_planner_node = Node(
        package='ms2r1p',
        executable='trajectory_planner_node',
        name='trajectory_planner'
    )


    trajectory_follower_node = Node( 
        package='ms2r1p', 
        executable='trajectory_follower',  
        name='trajectory_follower' )
    

    inverse_kinematics_node = Node(
        package='ms2r1p',
        executable='inverse_kinematics',
        name='inverse_kinematics'
    )

    direct_kinematics_node = Node(
        package='ms2r1p',
        executable='direct_kinematics',
        name='direct_kinematics'
    )

    state_publisher_node = Node(
        package='ms2r1p',
        executable='ms2r1p_state_publisher',
        name='state_publisher',
        parameters=[{
            "arm_1": "base_to_arm1_cx",
            "arm_2": "arm_1_arm_2_joint_cx",
            "prismatic": "arm_2_prismatic_joint_cx",
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        dxf_parser_node,
        direct_kinematics_node,
        #trajectory_planner_node,
        trajectory_follower_node,
        inverse_kinematics_node,
        state_publisher_node,
        #goal_pose_traducer_node,
        rviz_node,  

    ])