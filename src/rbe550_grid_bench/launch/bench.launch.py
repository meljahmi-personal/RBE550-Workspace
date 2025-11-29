from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    algo = LaunchConfiguration("algo")
    moves = LaunchConfiguration("moves")
    weight = LaunchConfiguration("weight")
    grid = LaunchConfiguration("grid")
    fill = LaunchConfiguration("fill")
    seed = LaunchConfiguration("seed")
    map_path = LaunchConfiguration("map")
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


    rviz_proc = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=["rviz2"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("algo", default_value="astar"),
        DeclareLaunchArgument("moves", default_value="8"),
        DeclareLaunchArgument("weight", default_value="1.0"),
        DeclareLaunchArgument("grid", default_value="64"),
        DeclareLaunchArgument("fill", default_value="0.20"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("start_goal", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        bench_proc,
        rviz_proc,
    ])
