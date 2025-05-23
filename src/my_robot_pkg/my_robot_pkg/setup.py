from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='futill',
    maintainer_email='358181022@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_move = my_robot_pkg.arm_move:main',
            'gui = my_robot_pkg.gui:main',
            'chess_vision_node = my_robot_pkg.chess_vision_node:main',
        ],
    },
)
