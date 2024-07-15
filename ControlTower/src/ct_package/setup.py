from setuptools import find_packages, setup
import os
import glob

package_name = 'ct_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bo',
    maintainer_email='bo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam1 = ct_package.cam1:main',
            'cam2 = ct_package.cam2:main',
            'vision_cam2 = ct_package.vision_cam2:main'
        ],
    },
)
