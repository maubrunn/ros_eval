from setuptools import setup, find_packages

def parse_requirements(filename):
    """Load requirements from a pip requirements file."""
    with open(filename, 'r') as f:
        return [line.strip() for line in f.readlines() if line.strip() and not line.startswith("#")]


setup(
    name='ros_eval',  # Replace with your project name
    version='0.1.0',  # Version of your package
    packages=find_packages(where='src'),  # Finds all packages in the src directory
    package_dir={'': 'src'},  # Tells setuptools to look for packages inside the src directory
    install_requires=parse_requirements('.docker_utils/requirements.txt'),
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.8',  # Adjust to your required Python version
)
