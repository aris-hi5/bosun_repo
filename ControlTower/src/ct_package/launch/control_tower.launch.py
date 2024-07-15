from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     namespace= '',
            #     package= 'ct_package',
            #     executable= 'taskManager',
            #     output= 'screen'
            # ),
                        Node(
                namespace= '',
                package= 'ct_package',
                executable= 'cam1',
                output= 'screen'
            ),            Node(
                namespace= '',
                package= 'ct_package',
                executable= 'cam2',
                output= 'screen'
            ),
                        Node(
                namespace= '',
                package= 'ct_package',
                executable= 'vision_cam2',
                output= 'screen'
            ),                   
        ]
    )