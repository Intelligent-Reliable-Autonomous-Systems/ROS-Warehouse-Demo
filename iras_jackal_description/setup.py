from setuptools import find_packages, setup
import os

package_name = 'iras_jackal_description'
data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
config_dir = "launch/"
for dirpath, dirnames, filenames in os.walk(config_dir):
    for f in filenames:
        full_path = os.path.join(dirpath, f)
        install_path = os.path.join('share', package_name, dirpath)
        data_files.append((install_path, [full_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will-solow',
    maintainer_email='soloww@oregonstate.edu',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
