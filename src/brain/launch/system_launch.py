import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/brain_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("perception"), '/launch/perception_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("inventory"), '/launch/inventory_launch.py']
            )
        )
    ])