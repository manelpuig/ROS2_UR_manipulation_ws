from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_kinematics_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='UR5e forward and inverse kinematics demo using MoveIt services.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5e_forward_kinematics_node = ur5e_kinematics_demo.ur5e_forward_kinematics_node:main',
            'ur5e_inverse_kinematics_node = ur5e_kinematics_demo.ur5e_inverse_kinematics_node:main',
            'move_to_pose_exe = ur5e_kinematics_demo.move_to_pose:main',
            'pick_place_exe = ur5e_kinematics_demo.pick_place:main',
        ],
    },
)
