from setuptools import find_packages, setup

package_name = 'drone_offboard'

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
    maintainer='seokjunkang',
    maintainer_email='owljun2@gmail.com',
    description='PX4 Offboard waypoint follower with PID velocity control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_node = drone_offboard.offboard_node:main',
        ],
    },
)
