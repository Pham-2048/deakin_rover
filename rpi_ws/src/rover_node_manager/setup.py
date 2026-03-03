from setuptools import find_packages, setup

package_name = 'rover_node_manager'

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
    description='Node launch/stop service handler for GUI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node_manager_node = rover_node_manager.node_manager_node:main',
        ],
    },
)
