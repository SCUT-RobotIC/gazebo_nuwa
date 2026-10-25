from setuptools import find_packages, setup

package_name = 'mini_car_control'

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
    maintainer='wyq',
    maintainer_email='wyq@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swerve_controller = mini_car_control.swerve_controller:main',
            'teleop_wasd = mini_car_control.teleop_wasd:main',
            'tower_check = mini_car_control.tower_check:main',
            'odom_to_tf = mini_car_control.odom_to_tf:main',
            'br_mission = mini_car_control.br_mission:main',
            'place_block = mini_car_control.place_block:main',
            'add_point_time = mini_car_control.add_point_time:main',
            'goto_pose = mini_car_control.goto_pose:main',
        ],
    },
)
