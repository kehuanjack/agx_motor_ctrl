#!/usr/bin/env python
import sys
from pathlib import Path

from setuptools import setup

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

from motor_install_hook import get_cmdclass  # noqa: E402

setup(cmdclass=get_cmdclass())
