import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    micro_ros = launch.actions.ExecuteProcess(cmd = ['ros2','run','micro_ros_agent','micro_ros_agent','serial','--dev','/dev/ttyUSB0'], output = 'screen')

    odometry_node = Node(package = 'odometry', executable = 'Odometry_Node', output = 'screen')

    pura_persecucion_node = Node(package = 'pura_persecucion', executable = 'pura_persecucion_node', output = 'screen')

    ros_bag = launch.actions.ExecuteProcess(cmd = ['ros2','bag','record','/pos_global'],output='screen')

    l_d = LaunchDescription([micro_ros, odometry_node, pura_persecucion_node, ros_bag])

    return l_d
