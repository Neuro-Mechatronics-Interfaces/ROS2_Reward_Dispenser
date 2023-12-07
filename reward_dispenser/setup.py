from setuptools import setup

package_name = 'reward_dispenser'

setup(
    name=package_name,
    version='0.1.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Shulgach',
    maintainer_email='jonathan@shulgach.com',
    description='A ROS2 node for dispensing reward during a behavioral experiment.',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
#            'reward = reward_dispenser.reward_bluetooth_node:main',
            'node = reward_dispenser.serial_node:main',
            'run_line = reward_dispenser.run_line_node:main',
        ],
    },
)
