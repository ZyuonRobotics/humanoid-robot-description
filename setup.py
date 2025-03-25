from setuptools import find_packages
from distutils.core import setup

setup(
    name='hurodes',
    version='1.0.0',
    packages=find_packages(),
    install_requires=["pandas", "colorama", "click", "mujoco"]
)
