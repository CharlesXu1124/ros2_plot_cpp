from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_plot_cpp', executable="ros2_plot_template", output="screen", emulate_tty=True),
    ])
