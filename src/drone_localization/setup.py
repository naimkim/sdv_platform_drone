from setuptools import find_packages, setup

package_name = 'drone_localization'

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
    description='GPS-denied VIO/GPS fusion with a GPS integrity monitor.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localization_node = drone_localization.localization_node:main',
        ],
    },
)
