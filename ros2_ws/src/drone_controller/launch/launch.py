from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Declare the number of drones argument
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to launch'
    )

    # Function to create nodes for each drone
    def create_drone_nodes(context):
        num_drones = int(LaunchConfiguration('num_drones').perform(context))
        nodes = []
        
        for i in range(1, num_drones + 1):
            node = Node(
                package='drone_controller',
                executable='controller_node',
                name=f'drone_controller_{i}',
                parameters=[{'drone_number': i}],
                output='screen'
            )
            nodes.append(node)
        
        return nodes

    # Create the launch description
    return LaunchDescription([
        num_drones_arg,
        OpaqueFunction(function=create_drone_nodes)
    ])
