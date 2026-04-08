from setuptools import find_packages, setup

package_name = 'dcr_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/BLD_305s.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan',
    maintainer_email='nathan@c0refailure.com',
    description='TODO: Package description',
    license='For Deakin Competitive Robotics Usage Only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = dcr_motor_controller.main:main'
        ],
    },
)
