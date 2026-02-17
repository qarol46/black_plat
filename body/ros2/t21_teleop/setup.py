from setuptools import find_packages, setup

package_name = 't21_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joy_flipper_teleop.launch.py']),
        ('share/' + package_name + '/launch', ['launch/joy_full_teleop.launch.py']),
        ('share/' + package_name + '/launch', ['launch/joy_platform_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danil',
    maintainer_email='podkolzindanil@yandex.ru',
    description='Teleoperation package for T21 robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_flipper_teleop = t21_teleop.joy_flipper_teleop:main',
            'joy_full_teleop = t21_teleop.joy_full_teleop:main',
            'drive_test = t21_teleop.drive_test:main',
            'joy_platform_teleop = t21_teleop.joy_platform_teleop:main',
            'flipper_teleop = t21_teleop.flipper_teleop:main',  # Добавлен новый узел
        ],
    },
)