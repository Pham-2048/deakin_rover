from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deakin Rover Team',
    maintainer_email='rover@deakin.edu.au',
    description='Launch files and configuration for Deakin Rover',
    license='MIT',
)
