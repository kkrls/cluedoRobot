from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages    = ['vision', 'readers', 'motion', 'environment', 'colour', 'integration'],
    package_dir = {'': 'scripts'}
)

setup(**setup_args)