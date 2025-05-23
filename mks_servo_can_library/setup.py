"""
Setup script for the mks-servo-can Python library.

This script uses setuptools to package and distribute the mks-servo-can library,
which provides tools for controlling MKS SERVO42D and MKS SERVO57D motors
via a CAN interface. It includes metadata such as package name, version, author,
dependencies, and classifiers.
"""

# mks_servo_can_project/mks_servo_can_library/setup.py
import re
import os
from setuptools import find_packages, setup


def get_version_from_init():
    """Reads the __version__ string from mks_servo_can/__init__.py."""
    init_py_path = os.path.join(
        os.path.dirname(__file__), "mks_servo_can", "__init__.py"
    )
    try:
        with open(init_py_path, "r", encoding="utf-8") as f_version:
            version_file_content = f_version.read()
        version_match = re.search(
            r"^__version__\s*=\s*['\"]([^'\"]*)['\"]",
            version_file_content,
            re.M,
        )
        if version_match:
            return version_match.group(1)
        raise RuntimeError(
            f"Unable to find __version__ string in {init_py_path}."
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"{init_py_path} not found. Ensure you are in the correct "
            f"directory."
        ) from exc


# Attempt to read long description from root README.md
# Adjust the path if your setup.py is not in 'mks_servo_can_library'
# relative to the root README
try:
    with open(
        "../README.md", "r", encoding="utf-8"
    ) as readme_file:  # Path relative to this setup.py
        long_description = readme_file.read()
except FileNotFoundError:
    # Fallback if README.md is not found at the expected relative path
    long_description = (
        "Python library to control MKS SERVO42D/57D motors via CAN bus."
    )


setup(
    name="mks-servo-can",
    version=get_version_from_init(),  # Use the new function to get version
    author="Ricardo de Azambuja",  # Please update this
    description="Python library to control MKS SERVO42D/57D motors "
    "via CAN bus.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ricardodeazambuja/mks_servo_can",
    packages=find_packages(
        where="."
    ),  # Finds mks_servo_can and mks_servo_can.kinematics
    # package_dir={'': '.'}, # Usually not needed if setup.py is in the
    # same dir as the main package folder
    classifiers=[
        "Development Status :: 3 - Alpha",  # Or Beta, Production/Stable
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",  # Choose your license
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    python_requires=">=3.8",
    install_requires=[
        "python-can>=3.0.0,<5.0.0",  # Specify a version range
        # Add other direct dependencies here if any
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-asyncio>=0.15",
            "pytest-mock>=3.0",
            "flake8>=3.9",
            "black>=21.0",
            "mypy>=0.900",
            "pylint>=2.0",
            "tox",  # if you use tox for testing matrix
        ],
        "docs": [
            "sphinx>=4.0",
            "sphinx-rtd-theme>=1.0",
            "myst-parser>=0.15",  # For Markdown support in Sphinx
            # other sphinx extensions
        ],
    },
    project_urls={
        "Bug Reports": "https://github.com/ricardodeazambuja/mks_servo_can",
        "Source": "https://github.com/ricardodeazambuja/mks_servo_can",
        "Documentation": "https://github.com/ricardodeazambuja/"
        "mks_servo_can/docs",
    },
    keywords="mks servo canbus motor control robotics automation servo42d "
    "servo57d asyncio",
)
