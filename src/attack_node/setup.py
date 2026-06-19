from setuptools import find_packages, setup

package_name = 'attack_node'

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
    description='Battery spoofing attack generator for SDV security demos',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'attack_node = attack_node.attack_node:main',
        ],
    },
)
