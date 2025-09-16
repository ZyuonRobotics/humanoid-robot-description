from setuptools import setup, find_packages
import os

# 读取 README.md 作为长描述
def get_long_description():
    with open('README.md', encoding='utf-8') as f:
        return f.read()

setup(
    name="hurodes",
    version="0.0.2",  # 也可以从 hurodes/__init__.py 动态获取
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
        "trimesh>=4.5.10",
        "fast-simplification>=0.1.11",
        "bidict",
        "PyYAML>=6.0",
        "pydantic>=2.0"
    ],
    extras_require={
        "dev": [
            "setuptools==68.2.2",
            "wheel==0.43.0",
            "pytest",
            "build", 
            "twine"
        ],
        "physics": ["mujoco>=3.3.0"],
    },
    entry_points={
        "console_scripts": [
            "hurodes-generate=hurodes.scripts.generate:main",
            "hurodes-generate-composite=hurodes.scripts.generate_composite:main",
            "hurodes-parse=hurodes.scripts.parse:main",
        ],
    },
    include_package_data=True,  # 包含非Python文件（如LICENSE）
)