from setuptools import setup
import os
from glob import glob

package_name = 'bag_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Добавляем launch файлы (если будут)
        #(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Добавляем конфигурационные файлы (если будут)
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='A package for parsing ROS2 bag files and extracting trajectory data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_parser_node = bag_parser.bag_parser_node:main',
        ],
    },
)