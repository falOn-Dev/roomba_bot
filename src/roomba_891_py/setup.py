from setuptools import find_packages, setup
from glob import glob

package_name = 'roomba_891_py'

launch_files = glob('launch/*.launch.py')
description_files = glob('description/*.xacro')
world_files = glob('worlds/*.sdf')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/description', description_files),
        ('share/' + package_name + '/worlds', world_files),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='falon',
    maintainer_email='falon@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_controller = roomba_891_py.differential_drive_controller:main',
            'pid_controller = roomba_891_py.pid_controller:main',
        ],
    },
)
