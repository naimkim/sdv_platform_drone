from setuptools import find_packages, setup

package_name = 'diagnostics_ecu'

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
    description='Diagnostics ECU node for SDV platform',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diagnostics_node = diagnostics_ecu.diagnostics_node:main',
        ],
    },
)
