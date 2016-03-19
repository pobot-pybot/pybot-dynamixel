# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

import textwrap

setup(
    name='pybot-dynamixel',
    namespace_packages=['pybot'],
    version='1.1.0',
    description='Dynamixel servos control',
    install_requires=['pybot-core'],
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
    package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'dmxl-setid = pybot.dynamixel.cli.dmxl_setid:main',
            'dmxl-scan = pybot.dynamixel.cli.dmxl_scan:main',
            'dmxl-regs = pybot.dynamixel.cli.dmxl_regs:main'
        ]
    }
)
