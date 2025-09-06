import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


LAUNCH = pathlib.Path("launch")
LAUNCH_OUSTER = LAUNCH / "ouster.py"
LAUNCH_IMAGING = LAUNCH / "imaging_source.py"

DESTINATION_NAME = 'destination'


def generate_launch_description():
    # Providing a filename and a path is mandatory for the script to launch
    destination = LaunchConfiguration(DESTINATION_NAME)
    destination_arg = DeclareLaunchArgument(
            DESTINATION_NAME,
            description="destination of the bagfile")

    # set up the sensors
    ouster = IncludeLaunchDescription(str(LAUNCH_OUSTER))
    imaging_source = IncludeLaunchDescription(str(LAUNCH_IMAGING))

    # set up the recorder
    recorder = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', destination],
            output='screen')


    return LaunchDescription([
        destination_arg,
        ouster,
        imaging_source,
        recorder
    ])
