from setuptools import find_packages, setup

package_name = 'rover_arm'

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
    description='6-DOF arm controller with safety limits',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller_node = rover_arm.arm_controller_node:main',
        ],
    },
)
