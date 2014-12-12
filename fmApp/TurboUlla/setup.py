from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['oee_calculator/oee_node.py'],)
setup(**d)
