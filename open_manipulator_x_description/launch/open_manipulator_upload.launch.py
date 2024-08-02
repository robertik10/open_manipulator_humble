import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('open_manipulator_x_description'),
                              'urdf', 'open_manipulator_x_robot.urdf.xacro')

    # Run the xacro command to generate the URDF file
    command = [
               'xacro',
               xacro_file
               ]
    completed_process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if completed_process.returncode != 0:
        raise RuntimeError(f"Command '{' '.join(command)}' failed with: {completed_process.stderr.decode()}")

    robot_description = completed_process.stdout
    # print(robot_description.decode())
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description.decode()}],  # Decode bytes to string
        ),
    ])
