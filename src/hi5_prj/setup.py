from setuptools import find_packages, setup
import glob
import os

package_name = 'hi5_prj'

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
    maintainer_email='mybosun99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main = hi5_prj.main:main",
            'cam1 = hi5_prj.cam1:main',
            'cam2 = hi5_prj.cam2:main',
            'vision_cam2 = hi5_prj.vision_cam2:main',
            'vision_cam1 = hi5_prj.vision_cam1:main',
            'kiosk_manager = hi5_prj.KioskManager:main',
            'test3 = hi5_hi5_prjpkg.test3:main',
            'db_manager = hi5_prj.DBManager:main',
            'yolo = hi5_prj.yolo:main'
        ],
    },
)
