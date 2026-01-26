#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['asr_sdm_rqt'],
    package_dir={'': 'src'},
    scripts=['scripts/asr_sdm_rqt']
)

setup(**d)