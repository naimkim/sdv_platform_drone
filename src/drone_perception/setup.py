from setuptools import find_packages, setup

package_name = 'drone_perception'

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
    maintainer='naimkim',
    maintainer_email='naimkim1013@gmail.com',
    description='YOLO detection to navigation obstacle bridge for the Sentinel Swarm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception_node = drone_perception.perception_node:main',
        ],
    },
)
