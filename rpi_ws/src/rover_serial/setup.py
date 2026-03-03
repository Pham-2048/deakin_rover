from setuptools import find_packages, setup

package_name = 'rover_serial'

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
    description='RS485/Modbus RTU bridge for drive motors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rs485_bridge_node = rover_serial.rs485_bridge_node:main',
        ],
    },
)
