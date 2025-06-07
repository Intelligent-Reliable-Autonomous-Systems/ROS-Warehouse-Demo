from setuptools import setup
import os
from glob import glob

package_name = 'kinova_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yashwanthi Anand',
    maintainer_email='anandy@oregonstate.edu',
    description='Pick and place functionality for Kinova arm',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control = kinova_pick_place.arm_control:main',
            'block_pick_place = kinova_pick_place.block_pick_place:main',
        ],
    },
)
