import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_data_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # ou find_packages()
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cj',
    maintainer_email='eudescodo00@gmail.com',
    description='Publisher/subscriber for random sensor data with custom msg',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_data_evaluation.sensor_publisher:main',
            'sensor_subscriber = sensor_data_evaluation.sensor_subscriber:main',
        ],
    },
)
