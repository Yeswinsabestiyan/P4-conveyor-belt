from setuptools import find_packages, setup

package_name = 'conveyor_sorter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sorter.launch.py']),
        ('share/' + package_name + '/config', ['config/sorter_params.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/conveyor_belt.urdf.xacro',
                                              'urdf/robot_arm.urdf.xacro',
                                              'urdf/bins.urdf.xacro',
                                              'urdf/main.urdf.xacro']),
        ('share/' + package_name + '/rviz', ['rviz/sorter.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 simulation of a sorting conveyor belt with two robot arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'box_spawner = conveyor_sorter.box_spawner:main',
            'conveyor = conveyor_sorter.conveyor:main',
            'color_sensor = conveyor_sorter.color_sensor:main',
            'arm_controller = conveyor_sorter.arm_controller:main',
            'stats_display = conveyor_sorter.stats_display:main',
        ],
    },
)