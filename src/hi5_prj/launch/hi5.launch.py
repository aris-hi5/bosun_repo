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
                package= 'hi5_prj',
                executable= 'main',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'cam1',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'cam2',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'vision_cam2',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'vision_cam1',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'kiosk_manager',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'db_manager',
                output= 'both'
            ),            Node(
                namespace= '',
                package= 'hi5_prj',
                executable= 'yolo',
                output= 'both'
            )
        ]
    )