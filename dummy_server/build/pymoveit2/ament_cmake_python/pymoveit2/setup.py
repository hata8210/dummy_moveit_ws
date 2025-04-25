from setuptools import find_packages
from setuptools import setup

setup(
    name='pymoveit2',
    version='4.0.0',
    packages=find_packages(
        include=('pymoveit2', 'pymoveit2.*')),
)
