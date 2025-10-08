import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="brain",
            executable="brain_transformations",
            name="brain_transformations",
            output='screen'
        ),
        Node(
            package="brain",
            executable="brain",
            name="brain",
            output='screen'
        )
    ])