from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

pkg_name = 'behavior_planner'

def generate_launch_description():
    ld = LaunchDescription(
    )
    smach_node = Node(
        package             = pkg_name,
        executable          = 'smach_node',
        name                = 'smach_node',
        #output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    ld.add_action(smach_node)

    return ld

  