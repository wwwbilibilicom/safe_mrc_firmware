#!/usr/bin/env python
"""
安装脚本，用于安装SafeMRC应用程序
"""
from setuptools import setup, find_packages

setup(
    name="safemrc-app",
    version="1.0.0",
    description="SafeMRC上位机应用程序",
    author="SafeMRC团队",
    author_email="safemrc@example.com",
    packages=find_packages(),
    install_requires=[
        "PyQt5>=5.15.0",
        "pyqtgraph>=0.12.0",
        "numpy>=1.19.0",
        "pyserial>=3.5",
    ],
    entry_points={
        "console_scripts": [
            "safemrc-app=run_app:main",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    python_requires=">=3.6",
) 