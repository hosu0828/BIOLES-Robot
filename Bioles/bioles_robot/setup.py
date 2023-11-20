from setuptools import setup
import os
from glob import glob

package_name = 'bioles_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seho',
    maintainer_email='hosu0828@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balancing = bioles_robot.balancing:main',
            'imu_filtering = bioles_robot.imu_filtering:main',
            'cam_filtering = bioles_robot.cam_filtering:main',
            'serial_left = bioles_robot.serial_left:main',
            'serial_right = bioles_robot.serial_right:main',
            'jumping = bioles_robot.jumping:main',
        ],
    },
)
