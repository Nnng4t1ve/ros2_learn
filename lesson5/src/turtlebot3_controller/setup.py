from setuptools import find_packages, setup

package_name = 'turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 controller with waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_mover = turtlebot3_controller.simple_mover:main',
            'odom_subscriber = turtlebot3_controller.odom_subscriber:main',
            'waypoint_nav = turtlebot3_controller.waypoint_nav:main',
            'virtual_teleop = turtlebot3_controller.virtual_teleop:main',
        ],
    },
)
