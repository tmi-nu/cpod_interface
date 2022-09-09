import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    coms_connector_share_dir = get_package_share_directory("coms_connector")
    coms_converter_share_dir = get_package_share_directory("coms_converter")
    read_launch = [
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([coms_connector_share_dir+"/launch/plc_connector.launch.py"]),
        ), 
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([coms_converter_share_dir+"/launch/plc_converter.launch.py"]),
        )
    ]
    
    
    return launch.LaunchDescription(
        read_launch,
    )