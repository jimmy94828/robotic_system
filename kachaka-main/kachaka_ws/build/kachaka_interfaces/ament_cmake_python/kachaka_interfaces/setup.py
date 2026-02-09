from setuptools import find_packages
from setuptools import setup

setup(
    name='kachaka_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('kachaka_interfaces', 'kachaka_interfaces.*')),
)
