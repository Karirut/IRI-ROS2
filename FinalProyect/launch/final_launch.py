import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    video_camara = launch.actions.ExecuteProcess(cmd = ['ros2','launch','ros_deep_learning','video_viewer.ros2.launch'], output = 'screen')

    micro_ros = launch.actions.ExecuteProcess(cmd = ['ros2','run','micro_ros_agent','micro_ros_agent','serial','--dev','/dev/ttyUSB0'], output = 'screen')

    color_ident = Node(package='final_project',executable = 'ident_red_green_yellow_node', output = 'screen')

    line_node = Node(package='final_project',executable = 'line_detection_node', output = 'screen')

    #master_node = Node(package='final_project',executable = 'master_node', output = 'screen')

    #transito_node = Node(package='final_project', executable = 'transito_node', output = 'screen')

    #view_image = launch.actions.ExecuteProcess(cmd = ['ros2','run','rqt_image_view','rqt_image_view'], output = 'screen')

    l_d = LaunchDescription([video_camara, micro_ros, color_ident, line_node]) #master_node, transito_node])

    return l_d
