#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

import textwrap

setup(name='pybot_dynamixel',
      version='1.0',
      description='Dynamixel servos control',
      install_requires=['pybot_core'],
      license='LGPL',
      long_description=textwrap.dedent("""
            This sub-package contains classes allowing controlling
            Dynamixel servos.
      """),
      author='Eric Pascual',
      author_email='eric@pobot.org',
      url='http://www.pobot.org',
      download_url='https://github.com/Pobot/PyBot',
      packages=find_packages("src"),
      package_dir={'': 'src'}
      )
