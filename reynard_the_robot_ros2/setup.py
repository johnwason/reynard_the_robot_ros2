from setuptools import setup

package_name = "reynard_the_robot_ros2"

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/reynard_the_robot_ros2.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='John Wason',
    maintainer_email='wason@wasontech.com',
    description='Reynard the Robot ROS2 package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'reynard_the_robot_ros2 = reynard_the_robot_ros2.reynard_the_robot_ros2:main'
        ],
    },
)