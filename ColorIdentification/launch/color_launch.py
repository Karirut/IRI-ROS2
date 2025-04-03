import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    video_camara = launch.actions.ExecuteProcess(cmd = ['ros2','launch','ros_deep_learning','video_viewer.ros2.launch'], output = 'screen')

    color_node = Node(package ='color_identification', executable = 'color_indentification_node', output = 'screen')

    view_image = launch.actions.ExecuteProcess(cmd = ['ros2','run','rqt_image_view','rqt_image_view'], output = 'screen')

    l_d = LaunchDescription([video_camara,color_node,view_image])

    return l_d
