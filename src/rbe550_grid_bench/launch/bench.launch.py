from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    algo = LaunchConfiguration("algo")
    moves = LaunchConfiguration("moves")
    weight = LaunchConfiguration("weight")
    grid = LaunchConfiguration("grid")
    fill = LaunchConfiguration("fill")
    seed = LaunchConfiguration("seed")
    map_path = LaunchConfiguration("map_path") 
    start_goal = LaunchConfiguration("start_goal")
    use_rviz = LaunchConfiguration("use_rviz")

    bench_proc = ExecuteProcess(
        cmd=[
            "bench",
            "--algo", algo,
            "--moves", moves,
            "--grid", grid,
            "--fill", fill,
            "--seed", seed,
            "--map", map_path,
            "--start-goal", start_goal,
            "--weight", weight,
            "--show",
        ],
        output="screen",
    )
    
    
    planner_node = Node(
        package="rbe550_grid_bench",
        executable="planner_node",  # This now exists in setup.py
        name="planner_node",
        output="screen",
        parameters=[{
            "algo": algo,
            "moves": moves,
            "weight": weight,
            "grid": grid,
            "fill": fill,
            "seed": seed,
            "map_path": map_path,  # Changed to match planner_node
            "start_goal": start_goal,
            # Note: resolution and frame_id use default values from planner_node
        }],
    )


    # Get the path to the RViz config
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('rbe550_grid_bench'),
        'config',
        'rbe550.rviz'
    ])

    
    rviz_proc = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=["rviz2", "-d", rviz_config_path],
        output="screen",
    )
    

    return LaunchDescription([
        DeclareLaunchArgument("algo", default_value="astar"),
        DeclareLaunchArgument("moves", default_value="8"),
        DeclareLaunchArgument("weight", default_value="1.0"),
        DeclareLaunchArgument("grid", default_value="64"),
        DeclareLaunchArgument("fill", default_value="0.20"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("map_path", default_value=""),  # Changed to match planner_node
        DeclareLaunchArgument("start_goal", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        planner_node,
        rviz_proc,
    ])

