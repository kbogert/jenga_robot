#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['src/load_block.py'],
    packages=['jenga_robot_gazebo'],
    package_dir={'': 'src'},
    )

setup(**d)
