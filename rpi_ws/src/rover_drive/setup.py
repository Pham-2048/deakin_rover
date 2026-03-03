from setuptools import find_packages, setup

package_name = 'rover_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deakin Rover Team',
    maintainer_email='rover@deakin.edu.au',
    description='Skid steer drive mixer for 6-wheel rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = rover_drive.drive_node:main',
        ],
    },
)
