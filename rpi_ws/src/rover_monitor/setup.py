from setuptools import find_packages, setup

package_name = 'rover_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deakin Rover Team',
    maintainer_email='rover@deakin.edu.au',
    description='System telemetry monitor for Deakin Rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'system_monitor_node = rover_monitor.system_monitor_node:main',
        ],
    },
)
