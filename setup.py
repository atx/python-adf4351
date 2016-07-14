#! /usr/bin/env python3

try:
    from setuptools import setup
except:
    from distutils.core import setup

setup(
    name = "adf4351",
    version = "0.1",
    packages = ["adf4351"],
    description = "Python ADF4351 library",
    author = "Josef Gajdusek",
    author_email = "atx@atx.name",
    url = "https://github.com/atalax/python-adf4351",
    license = "MIT",
    classifiers = [
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Topic :: Utilities",
        ]
    )
