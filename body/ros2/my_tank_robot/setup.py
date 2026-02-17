from setuptools import setup
import os
from glob import glob

package_name = 'my_tank_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Параметры установки
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Добавим наши launch-файлы
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Добавим папку urdf (если нужно доставлять .xacro/.urdf)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
          # Чтобы скопировать *.yaml из папки config/
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Добавим папку meshes (3D-модели)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # Добавим папку worlds 
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My tank robot for Gazebo Classic in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_control_test = my_tank_robot.ros2_control_test:main',
            'ldr = my_tank_robot.lidar_data_record:main',
            'ldv = my_tank_robot.lidar_data_visual:main',
            'or = my_tank_robot.odom_rviz:main',
            'cart = my_tank_robot.cart:main',
            'alg = my_tank_robot.alg:main',
            'VLP_16_publisher_points = my_tank_robot.VLP_16_publisher_points:main',
            'simple_3d_map = my_tank_robot.simple_3d_map:main',
            'publisher_angle = my_tank_robot.publisher_angle:main',
        ],
    },
)
