from setuptools import find_packages
from setuptools import setup

setup(
    name='my_robot_pkg_msg',
    version='0.5.0',
    packages=find_packages(
        include=('my_robot_pkg_msg', 'my_robot_pkg_msg.*')),
)
