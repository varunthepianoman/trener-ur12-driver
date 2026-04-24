from setuptools import find_packages, setup

package_name = 'ur12_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Varun',
    maintainer_email='varunkamat23@gmail.com',
    description='Minimal ROS2 driver for UR12 E-series',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ur_driver_node = ur12_driver.ur_driver_node:main',
        ],
    },
)
