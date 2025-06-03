from setuptools import find_packages, setup

package_name = 'camera_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/test_calibration.py', 
            'launch/calibrate.py',
            'launch/manipulation_camera_real.py',
            'launch/manipulation_camera_gz.py',
        ]),
        ('share/' + package_name + '/config', ['config/camera_info.yaml']),
    ],
    install_requires=['setuptools', 'cv_bridge', 'opencv-python', 'transforms3d'],
    zip_safe=True,
    maintainer='jjewett',
    maintainer_email='jewettje@oregonstate.edu',
    description='An external camera for detecting objects in a manipulation workspace',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_overlay = camera_detection.grid_overlay:main',
            'camera_tf = camera_detection.camera_tf:main',
        ],
    },
)
