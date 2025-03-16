from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="droneops",
    version="0.1.0",
    author="Rui Afonso Martins",
    author_email="martins.ruiafonso@gmail.com",
    description="Advanced drone control and mission planning library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/droneops/droneops",
    project_urls={
        "Bug Tracker": "https://github.com/droneops/droneops/issues",
        "Documentation": "https://droneops.readthedocs.io/",
        "Source": "https://github.com/rafonsomartins/droneops",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: GIS",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    packages=find_packages(),
    python_requires=">=3.7",
    install_requires=[
        "numpy>=1.19.0",
        "dronekit>=2.9.2",
        "pymavlink>=2.4.0",
        "shapely>=1.7.0",
    ],
    extras_require={
        "kml": ["fastkml>=0.11"],
        "dev": [
            "pytest>=6.0.0",
            "pytest-cov>=2.0.0",
            "black>=21.0.0",
            "flake8>=3.9.0",
            "sphinx>=4.0.0",
            "sphinx-rtd-theme>=0.5.0",
        ],
    },
    include_package_data=True,
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'droneops=droneops.cli:main',
        ],
    },
)