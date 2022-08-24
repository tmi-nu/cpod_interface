import os

from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import AnonName
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            data =  yaml.safe_load(f)
            print(data)
            return data['/**']['ros__parameters']    
        
    ns = ""
    pkg = "coms_converter"
    
    
    plc_converter_component = Node(
        package=pkg, 
        namespace = '',
        executable = 'plc_converter',
        name='plc_converter',
        output = "screen",
        # remappings=,
        parameters=[load_composable_node_param("coms_converter_path")]
    )
    
    return [plc_converter_component]



def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    
    return launch.LaunchDescription(
        [
            # add_launch_arg(
            #     "coms_converter_path", 
            #     [os.path.join(get_package_share_directory('coms_converter'), "config", "plc_converter.param.yaml")],   
            # ),
            add_launch_arg(
                "coms_converter_path", 
                [os.path.join(get_package_share_directory('coms_converter'), "config", "plc_converter.param.yaml")],   
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )