#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['semantic_region_handler'],
    package_dir={'':'src'},
    scripts = [ 
                'scripts/table_handler.py',
                'scripts/table_poller.py',
                'scripts/table_loader.py',
                'scripts/alvar_ar_handler.py',
                'scripts/alvar_ar_poller.py',
                'scripts/alvar_ar_loader.py',
              ]
#    requires=['actionlib','rospy_message_converter','world_msgs','worldlib']
)

setup(**d)
