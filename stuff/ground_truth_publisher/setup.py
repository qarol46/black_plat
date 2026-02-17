from setuptools import setup
import os
from glob import glob

package_name = 'ground_truth_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Если у вас есть launch файлы, добавьте эту строку:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Ground truth publisher from Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ИСПРАВЛЕНО: изменил simple_gz_ground_truth на simple_gazebo_ground_truth
            'simple_ground_truth = ground_truth_publisher.simple_gazebo_ground_truth:main',
        ],
    },
)