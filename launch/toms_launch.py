from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

import os
from glob import glob

def generate_launch_description():
    ld = LaunchDescription()
    
    # 状態遷移ノード
    behavior_planner = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('behavior_planner'),
                        'behavior_planner_launch.py'
                    ])
                ]),
    )

    # 画像処理ノード
    vision_package = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('vision_node'),
                        'vision_node_launch.py'
                    ])
                ]),
    )

    # マニピュレータ制御ノード
    arm_controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('arm_controller'),
                        'arm_launch.py'
                    ])
                ]),
    )
    
    #　カート制御ノード
    cart_controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('cart_controller'),
                        'cart_launch.py'
                    ])
                ]),
    )


    ld.add_action(behavior_planner)
    ld.add_action(vision_package)
    ld.add_action(arm_controller)
    ld.add_action(cart_controller)

    return ld