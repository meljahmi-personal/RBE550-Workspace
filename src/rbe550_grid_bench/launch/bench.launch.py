from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='rbe550_grid_bench', executable='bench', name='grid_bench', output='screen')])
