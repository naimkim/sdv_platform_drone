import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seokjunkang',
    maintainer_email='owljun2@gmail.com',
    description='RViz visualization bridge for the Sentinel Swarm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'viz_node = swarm_viz.viz_node:main',
        ],
    },
)
