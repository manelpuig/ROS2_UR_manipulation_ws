from setuptools import setup

package_name = 'ur5e_kinematics_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fk_ur5e.launch.py',
                                               'launch/ik_ur5e.launch.py']),
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
        ],
    },
)