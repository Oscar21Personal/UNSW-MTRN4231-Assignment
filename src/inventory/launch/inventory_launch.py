import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="inventory",
            executable="inventory_transformations",
            name="inventory_transformations",
            output='screen'
        ),
        Node(
            package="inventory",
            executable="inventory_manager",
            name="inventory_manager",
            output='screen'
        )
    ])