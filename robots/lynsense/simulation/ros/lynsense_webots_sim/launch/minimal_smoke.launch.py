from launch import ExecuteProcess, LaunchDescription


CONTROLLER_PATH = (
    "/workspace/ws/install/lynsense_webots_sim/bin/lynsense_webots_controller"
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[CONTROLLER_PATH],
                output="screen",
            )
        ]
    )
