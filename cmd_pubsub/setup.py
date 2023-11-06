from setuptools import find_packages, setup

package_name = 'cmd_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alvin',
    maintainer_email='alvin870203@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fws_joint_gazebo_control_cmd = cmd_pubsub.fws_joint_gazebo_control_cmd:main',
            'fws_joint_ros2_control_cmd = cmd_pubsub.fws_joint_ros2_control_cmd:main',
            '4ws_joint_gazebo_control_cmd = cmd_pubsub.4ws_joint_gazebo_control_cmd:main',
            '4ws_joint_ros2_control_cmd = cmd_pubsub.4ws_joint_ros2_control_cmd:main',
        ],
    },
)
