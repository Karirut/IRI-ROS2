import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    camara = launch.actions.ExecuteProcess(cmd = ['ros2', 'launch', 'ros_deep_learning', 'video_viewer.ros2.launch'], output = 'screen')

    micro_ros = launch.actions.ExecuteProcess(cmd = ['ros2','run','micro_ros_agent','micro_ros_agent','serial','--dev', '/dev/ttyUSB0'], output = 'screen')  

    odometry_node = Node(package='odometry',executable = 'Odometry_Node', output = 'screen')

    color_node = Node(package='color_identification',executable = 'color_identification_node', output = 'screen')

    #line_node = Node(package='line_detection',executable = 'line_detection_node', output = 'screen')
    line_node = launch.actions.TimerAction(actions = [Node(package='line_detection', executable='line_detection_node', output='screen')], period = 5.0)

    l_d = LaunchDescription([camara, micro_ros, odometry_node, color_node, line_node])

    return l_d
