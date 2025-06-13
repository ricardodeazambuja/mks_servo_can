"""
Setup script for the mks-servo-simulator package.

This script uses setuptools to package and distribute the mks-servo-simulator,
a command-line tool that emulates MKS SERVO42D/57D CAN motors.
The simulator allows for testing the mks-servo-can library without
requiring physical hardware. It defines metadata, dependencies, and the
entry point for the simulator's command-line interface.
"""
import os
import re
from setuptools import find_packages, setup


def get_version_from_init():
    """Reads the __version__ string from mks_simulator/__init__.py."""
    init_py_path = os.path.join(
        os.path.dirname(__file__), "mks_simulator", "__init__.py"
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


# Try to get long description from a shared README, adjust path as necessary
try:
    with open("../../README.md", "r", encoding="utf-8") as f_readme:
        long_description = f_readme.read()
except FileNotFoundError:
    long_description = (
        "MKS Servo CAN Simulator for testing the mks-servo-can library."
    )


setup(
    name="mks-servo-simulator",
    version=get_version_from_init(),
    author="Ricardo de Azambuja",  # Same as library or specific for sim
    description="CLI-based simulator for MKS SERVO42D/57D CAN motors.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ricardodeazambuja/mks_servo_can",
    packages=find_packages(where="."),  # Finds mks_simulator
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",  # Match main library license
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Testing :: Simulation",
        "Topic :: System :: Emulators",
    ],
    python_requires=">=3.8",
    install_requires=[
        "click>=7.0",  # For the CLI
        "rich>=10.0.0",  # For the rich console dashboard
        "fastapi>=0.68.0",  # For the HTTP debug server
        "uvicorn>=0.15.0",  # For running the FastAPI server
        # If the simulator directly uses the mks-servo-can library as a
        # dependency:
        # "mks-servo-can @ path=../mks_servo_can_library" # For local dev
        # Or if library is installed: "mks-servo-can>=0.1.0"
        # For now, it tries to import directly for constants, which is okay
        # if run from project root or if mks_servo_can_library is in
        # PYTHONPATH.
        # No direct install of the library for the simulator to avoid
        # circular dependency issues if packaged separately.
        # It's more of a development tool.
    ],
    entry_points={
        "console_scripts": [
            "mks_servo_simulator=mks_simulator.main:main",
            "mks-servo-simulator=mks_simulator.main:main",  # Keep hyphenated version for compatibility
        ],
    },
    project_urls={
        "Bug Reports": "https://github.com/ricardodeazambuja/mks_servo_can",
        "Source": "https://github.com/ricardodeazambuja/mks_servo_can",
        "Documentation": "https://github.com/ricardodeazambuja/"
        "mks_servo_can/docs",
    },
    keywords="mks servo canbus motor simulator testing development",
)
