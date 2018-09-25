import roslib
import roslaunch

import os

from nose.plugins.attrib import attr


class NodeNotFoundException(Exception):
    pass


def check_launch_file(file_name):
    lf_path = roslib.packages.find_resource(
        "kuri_edu",
        file_name
    )[0]

    _check_launch_file_core(lf_path)


def _check_launch_file_core(full_path):
    # This is split out so we can test it with test data
    # that's not part of a ROS package
    loader = roslaunch.xmlloader.XmlLoader()

    cfg = roslaunch.config.ROSLaunchConfig()

    loader.load(full_path, cfg)

    # Check that all the nodes are the real deal:
    for node in cfg.nodes:
        if not roslib.packages.find_resource(
                   node.package,
                   node.type):
            raise NodeNotFoundException(node.type)


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


def test_bad_launch_file():
    # This is a sanity check for ISSUE-17.  Make sure our test notices
    # missing nodes
    path_to_me = os.path.realpath(__file__)
    path_to_dir = os.path.dirname(path_to_me)
    path_to_test_data = os.path.join(path_to_dir, "test_data")
    path_to_launch = os.path.join(path_to_test_data, "bad_node.launch")

    try:
        _check_launch_file_core(path_to_launch)
    except NodeNotFoundException:
        return

    assert False, "_check_launch_file_core did not raise NodeNotFound"
