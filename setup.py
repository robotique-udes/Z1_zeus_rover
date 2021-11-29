#!/usr/bin/env python

from distutils.core import setup
from glob import glob

from setuptools import find_packages

setup(name='RoboticArm',
      version='1.0',
      description='Python Distribution Utilities',
      author='Santiago Moya',
      packages=find_packages('zeus_arm'),
      package_dir={'': 'zeus_arm'},
      py_modules=[splitext(basename(path))[0] for path in glob('zeus_arm/scripts/*.py')],
     )