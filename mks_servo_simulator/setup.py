# mks_servo_can/mks_servo_simulator/setup.py
from setuptools import setup, find_packages

# Read version from __init__.py of the simulator package
version = {}
with open("mks_simulator/__init__.py") as fp:
    exec(fp.read(), version)

# Try to get long description from a shared README, adjust path as necessary
try:
    with open('../../README.md', 'r', encoding='utf-8') as f:
        long_description = f.read()
except FileNotFoundError:
    long_description = "MKS Servo CAN Simulator for testing the mks-servo-can library."


setup(
    name="mks-servo-simulator",
    version=version['__version__'],
    author="[Your Name/Organization]", # Same as library or specific for simulator
    author_email="[your.email@example.com]",
    description="CLI-based simulator for MKS SERVO42D/57D CAN motors.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://docs.github.com/en/get-started/start-your-journey/uploading-a-project-to-github", # Link to the main project
    packages=find_packages(where="."), # Finds mks_simulator
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License", # Match main library license
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
        "click>=7.0", # For the CLI
        # If the simulator directly uses the mks-servo-can library as a dependency:
        # "mks-servo-can @ path=../mks_servo_can_library" # For local development
        # Or if library is installed: "mks-servo-can>=0.1.0"
        # For now, it tries to import directly for constants, which is okay if run from project root
        # or if mks_servo_can_library is in PYTHONPATH.
        # No direct install of the library for the simulator to avoid circular dependency issues if packaged separately.
        # It's more of a development tool.
    ],
    entry_points={
        'console_scripts': [
            'mks-servo-simulator=mks_simulator.main:main',
        ],
    },
    project_urls={
        "Bug Reports": "https://www.projectmanagement.com/wikis/233063/issue-tracking",
        "Source": "https://www.quora.com/How-can-I-understand-the-source-code-of-a-project-I-am-a-fresher-and-my-senior-expects-me-to-understand-the-code-properly-and-add-a-feature-The-project-is-huge-What-should-I-do",
    },
    keywords="mks servo canbus motor simulator testing development",
)