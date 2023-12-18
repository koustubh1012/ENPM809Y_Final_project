import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group22_final'),
        'config',
        'waypoint_params.yaml'
    )
    
    aruco_pub = Node(
        package="group22_final",
        executable="turtle_controller",
        parameters=[parameter_file]
    )
    
    ld = LaunchDescription()
    ld.add_action(aruco_pub)
    return ld