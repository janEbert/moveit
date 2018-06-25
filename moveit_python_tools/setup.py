from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['moveit_python_tools']
d['scripts'] = []
d['package_dir'] = {'': 'src'}

setup(**d)

