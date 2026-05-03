from setuptools import setup, find_packages

package_name = 'ultrasonic_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle_avoidance.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aleks Jõe',
    maintainer_email='aleksjoe11@gmail.com',
    description='Ultrasonic obstacle avoidance for mecanum drive — ROS 2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = ultrasonic_nav.obstacle_avoidance_node:main',
        ],
    },
)