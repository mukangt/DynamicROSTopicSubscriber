"""
Author       : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
Date         : 2024-08-19 23:22:47
LastEditors  : HANG Tao (BCSC-EPA1, XC-DX/PJ-W3-PMT) Tao.HANG@cn.bosch.com
LastEditTime : 2024-08-19 23:51:47
FilePath     : /DynamicROSTopicSubscriber/setup.py
Description  : 

Copyright (c) 2024 by Robert Bosch GmbH, All Rights Reserved. 
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="dynamic_rostopic_subscriber",
    version="0.1.0",
    author="HANG Tao",
    author_email="mukangt@qq.com",
    description="A dynamic ROS topic subscriber library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/mukangt/DynamicROSTopicSubscriber",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    python_requires=">=3.6",
    install_requires=[
        "rospy",
        "genpy",
    ],
)
