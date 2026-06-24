from setuptools import find_packages, setup

package_name = 'swarm_consensus'

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
    description='Byzantine-resilient quarantine consensus for the Sentinel Swarm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'consensus_node = swarm_consensus.consensus_node:main',
        ],
    },
)
