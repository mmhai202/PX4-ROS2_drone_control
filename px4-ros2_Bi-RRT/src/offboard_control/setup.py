from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['package.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='hai',
    maintainer_email='hai372002@gmail.com',
    description='ROS 2 package for PX4 offboard control using Bi-RRT',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = offboard_control.main:main',
        ],
    },
)

