from setuptools import setup, find_packages
import os

def get_long_description():
    with open('README.md', encoding='utf-8') as f:
        return f.read()

def get_version():
    version_file = os.path.join(os.path.dirname(__file__), 'hurodes', '__init__.py')
    with open(version_file, encoding='utf-8') as f:
        for line in f:
            if line.startswith('VERSION'):
                return line.split('=')[1].strip().strip('"').strip("'")
    raise RuntimeError("Can't find version")

setup(
    name="hurodes",
    version=get_version(),
    description="hurodes (Humanoid Robot Description) is a Python toolkit for describing, converting, and processing humanoid robot models.",
    long_description=get_long_description(),
    long_description_content_type='text/markdown',
    author="ZyuonRobotics",
    maintainer="Honglong Tian",
    license="MIT",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords=["robotics", "humanoid", "urdf", "mujoco"],
    url="https://github.com/ZyuonRobotics/humanoid-robot-description",
    packages=find_packages(include=["hurodes", "hurodes.*"]),
    python_requires=">=3.9",
    install_requires=[
        "pandas>=2.0",
        "numpy>=1.22.4",
        "colorama>=0.4.6",
        "click>=8.0",
        "tqdm>=4.67.1",
        "bidict",
        "PyYAML>=6.0",
        "pydantic>=2.0",
        "mujoco>=3.3.0",
        "scipy>=1.10.0"
    ],
    extras_require={
        "dev": [
            "setuptools==68.2.2",
            "wheel==0.43.0",
            "pytest",
            "build", 
            "twine"
        ],
        "mesh": [
            "trimesh>=4.5.10",
            "fast-simplification>=0.1.11",
        ],
        "hal": [
            "casadi>=3.5.0",
            "numba>=0.57.0"
        ],
        "all": [
            "trimesh>=4.5.10",
            "fast-simplification>=0.1.11",
            "casadi>=3.5.0",
            "numba>=0.57.0"
        ]
    },
    entry_points={
        "console_scripts": [
            "hurodes-generate=hurodes.scripts.generate:main",
            "hurodes-generate-composite=hurodes.scripts.generate_composite:main",
            "hurodes-parse=hurodes.scripts.parse:main",
        ],
    },
    include_package_data=True,
)
