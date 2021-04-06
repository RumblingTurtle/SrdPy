import sys
from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(name="SrdPy",
    version="0.1",
    description="Serial Robot Dynamics port to python",
    long_description=long_description,
    url="https://github.com/RumblingTurtle/SrdPy",
    author="Eduard Zalyaev",
    author_email="e.zalyaev@innopolis.ru",
    packages=find_packages("src"),
    package_dir={"": "src"},

    install_requires=[
      "numpy",
      "casadi",
      "control",
      "slycot",
      "urdf-parser-py",
      "pyngrok",
      "qpsolvers"
    ]
)