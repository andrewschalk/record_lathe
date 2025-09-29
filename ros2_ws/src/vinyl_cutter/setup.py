from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vinyl_cutter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_interface = vinyl_cutter.web_interface:main',
            'vinyl_cutter = vinyl_cutter.vinyl_cutter:main',
            'speakers = vinyl_cutter.speakers:main',
            'r_motor = vinyl_cutter.r_motor:main',
            'theta_motor = vinyl_cutter.theta_motor:main',
            'limit_switch = vinyl_cutter.limit_switch:main',
        ],
    },
        package_data={
        "vinyl_cutter": ["templates/*"]
    },
)
