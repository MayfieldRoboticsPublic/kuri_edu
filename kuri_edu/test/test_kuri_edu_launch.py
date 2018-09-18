import roslib
import roslaunch

import os

from nose.plugins.attrib import attr


def check_launch_file(file_name):
    lf_path = roslib.packages.find_resource(
        "kuri_edu",
        file_name
    )[0]

    loader = roslaunch.xmlloader.XmlLoader()

    loader.load(lf_path, roslaunch.config.ROSLaunchConfig())


@attr("gbz")
def test_launch_files():
    # Check that all launch files can be imported
    test_path = os.path.dirname(__file__)           # ./kuri_edu/test/
    pkg_path = os.path.split(test_path)[0]          # ./kuri_edu
    launch_path = os.path.join(pkg_path, "launch")  # ./kuri_edu/launch

    for fn in os.listdir(launch_path):
        # For every file, emit a test to check that file
        def test_func(name): check_launch_file(name)
        test_func.description = "test_launch_file {}".format(fn)

        # Nose will treat this as N different tests - each for one
        # file found in the launch directory
        yield test_func, fn
