from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Get the launch configurations
    num_drones = LaunchConfiguration('num_drones').perform(context)
    priorities_str = LaunchConfiguration('priorities').perform(context)
    
    # Convert to Python objects
    num_drones = int(num_drones)
    priorities = eval(priorities_str)
    
    # Validate priorities
    if not isinstance(priorities, list) or len(priorities) != num_drones:
        priorities = [i for i in range(num_drones)]
    
    # List to hold the drone nodes
    drone_nodes = []
    
    for i in range(num_drones):
        # Define parameters for each drone
        drone_id = i + 1
        priority = priorities[i]
        
        # Create a node for each drone
        drone_election_node = Node(
            package='drones_intracom_py',  # Replace with your package name
            executable='election_node',
            name=f'drone_election_{drone_id}_node',
            parameters=[{
                'drone_id': drone_id,
                'priority': priority,
                'cluster_drones_number':num_drones
            }]
        )
        drone_nodes.append(drone_election_node)
    
    return drone_nodes

def generate_launch_description():
    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones in the swarm'
    )
    
    priorities_arg = DeclareLaunchArgument(
        'priorities',
        default_value='[]',
        description='List of priorities for each drone'
    )
    
    # Use OpaqueFunction to delay evaluation of LaunchConfiguration
    return LaunchDescription([
        num_drones_arg,
        priorities_arg,
        OpaqueFunction(function=launch_setup)
    ])