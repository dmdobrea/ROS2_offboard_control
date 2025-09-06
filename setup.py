from setuptools import find_packages, setup

package_name = 'offboard_control'

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
    maintainer='Dobrea Dan-Marius',
    maintainer_email='mdobrea@gmail.com',
    description='UAV control through an XBOX remote',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'xbox = offboard_control.xbox:main',
             'fly  = offboard_control.uav_ctrl:main',
        ],
    },
)
