from setuptools import setup, find_packages

setup(
    name="hurodes",
    version="1.0.0",
    packages=find_packages(include=["hurodes*"]),
    install_requires=[
        "pandas>=2.0",
        "colorama>=0.4.6",
        "click>=8.0",
        "mujoco>=2.3.3"
    ],
    python_requires=">=3.8",
)