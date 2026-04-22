import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ssmr_slip_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'pyserial', 'smbus2'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Slip-aware odometry for skid-steer mobile robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slip_odom_node = ssmr_slip_odom.slip_odom_node:main',
            'experiment_runner = ssmr_slip_odom.experiment_runner:main',
            'yahboom_base_node = ssmr_slip_odom.yahboom_base_node:main',
            'bno055_node = ssmr_slip_odom.bno055_node:main',
        ],
    },
)
