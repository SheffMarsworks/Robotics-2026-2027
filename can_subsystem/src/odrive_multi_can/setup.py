from glob import glob

from setuptools import find_packages, setup

package_name = 'odrive_multi_can'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MarsWorks Software Team',
    description='Multi-motor ODrive CAN control node for the MarsWorks rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_motor_node = odrive_multi_can.multi_motor_node:main',
            'multi_motor_test = odrive_multi_can.multi_motor_test:main',
        ],
    },
)
