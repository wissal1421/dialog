import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    dialogflow_dir = get_package_share_directory('dialogflow_ros2')

    params_file = os.path.join(dialogflow_dir, 'config', 'params.yaml')
    google_application_credentials = os.getenv('GOOGLE_APPLICATION_CREDENTIALS')
    languaje = os.getenv('LANG')

    dialogflow_client_node = Node(
        package='dialogflow_ros2',
        executable='dialogflow_client',
        output='screen',
        parameters=[{
                    'google_application_credentials': google_application_credentials,
                    'default_language': languaje,
                    }, params_file],
    )

    ld = LaunchDescription()
    ld.add_action(dialogflow_client_node)

    return ld
