import roslib
import roslaunch

from nose.plugins.attrib import attr


def check_launch_file(file_name):
    lf_path = roslib.packages.find_resource(
        "kuri_edu",
        file_name
    )[0]

    loader = roslaunch.xmlloader.XmlLoader()

    loader.load(lf_path, roslaunch.config.ROSLaunchConfig())


@attr("gbz")
def test_kuri_launch():
    check_launch_file("kuri_edu.launch")

@attr("gbz")
def test_kuri_remote_launch():
    check_launch_file("kuri_remote.launch")
