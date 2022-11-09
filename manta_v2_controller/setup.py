from setuptools import setup
from glob import glob

import os


package_name = 'manta_v2_controller'
submodules = 'manta_v2_controller/control_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takumi Asada',
    maintainer_email='tasada038@gmail.com',
    description='manta v2 controller',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wing_motion_node = manta_v2_controller.wing_motion_node:main',
            'walking_motion_node = manta_v2_controller.walking_motion_node:main',
            'servo_node = manta_v2_controller.servo_node:main',
        ],
    },
)
