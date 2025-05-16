# mks_servo_can_project/mks_servo_can_library/setup.py
from setuptools import setup, find_packages

# Read version from __init__.py
version = {}
with open("mks_servo_can/__init__.py") as fp:
    exec(fp.read(), version)


with open('../../README.md', 'r', encoding='utf-8') as f: # Adjust path if README is elsewhere
    long_description = f.read()

setup(
    name="mks-servo-can",
    version=version['__version__'],
    author="[Your Name/Organization]",
    author_email="[your.email@example.com]",
    description="Python library to control MKS SERVO42D/57D motors via CAN bus.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://docs.github.com/en/get-started/start-your-journey/uploading-a-project-to-github",
    packages=find_packages(where="."), # Finds mks_servo_can and mks_servo_can.kinematics
    # package_dir={'': '.'}, # If your packages are directly under root
    classifiers=[
        "Development Status :: 3 - Alpha", # Or Beta, Production/Stable
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License", # Choose your license
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
        "python-can>=3.0.0,<5.0.0", # Specify a version range
        # Add other direct dependencies here if any
        # e.g., "numpy" if used extensively in complex kinematics,
        # but it's optional as per the plan.
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
            "tox", # if you use tox for testing matrix
        ],
        "docs": [
            "sphinx>=4.0",
            "sphinx-rtd-theme>=1.0",
            # other sphinx extensions
        ],
    },
    project_urls={
        "Bug Reports": "https://www.nuclino.com/solutions/issue-tracking-software",
        "Source": "https://www.techtarget.com/searchapparchitecture/definition/source-code",
        # "Documentation": "https://www.pwc.com/mt/en/publications/transfer-pricing-documentation.html"
    },
    keywords="mks servo canbus motor control robotics automation servo42d servo57d",
)