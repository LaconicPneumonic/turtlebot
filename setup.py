""" Configuration file for metis package """
from distutils.core import setup

try:
    from catkin_pkg.python_setup import generate_distutils_setup
    SETUP_ARGS = generate_distutils_setup()
except ImportError:
    #TODO this duplicates the version information
    # It would be nice to parse the version (at least) from the XML file
    SETUP_ARGS = {
        'name': 'iago',
        'version': '0.0.1',
        'description': '''A module for funding objects in unmapped ares''',
        'author': 'Anthony Rolland',
        'author_email': 'rolland@mit.edu',
    }

SETUP_ARGS.update({
    'packages': ['turtlebot'],
    'package_dir': {'': 'src'},
    'scripts': [],
})

setup(**SETUP_ARGS)