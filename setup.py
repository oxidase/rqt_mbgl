# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rqt_mbgl'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'roscpp', 'rqt_gui', 'rqt_gui_cpp', 'rosrm']
)

setup(**setup_args)
