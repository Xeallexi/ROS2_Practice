import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_tf2_chase'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xeallex',
    maintainer_email='xeallex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcast = turtle_tf2_chase.turtle_broadcast:main',
            'listener = turtle_tf2_chase.turtle_tf2_listener:main',
        ],
    },
)
