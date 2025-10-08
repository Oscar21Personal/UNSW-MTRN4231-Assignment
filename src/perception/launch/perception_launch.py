import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="perception",
            executable="perception",
            name="perception",
            output='screen'
        )
    ])