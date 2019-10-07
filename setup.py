## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['crazyflie', 'crazyflie.sim'],
    package_dir={'': 'src'},
    scripts=['scripts/data_capture.py','scripts/ext_data_capture.py','scripts/ext_sanity_data_capture.py','scripts/view_camera.py']
)

setup(**setup_args)
