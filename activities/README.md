# Activities
<--description of each activity-->

## Getting Started:

### Downloading the Kuri EDU Source
Almost all of the kuri EDU activities require the user to set up a caktin
workspace to overlay their modifications to the kuri EDU software.  Start by
logging into your kuri EDU robot and cloning the kuri EDU github repo.

If your host machine is set up with your github credentials, you can forward
those credentials to by using the -A flag when you SSH.  See
https://help.github.com/articles/connecting-to-github-with-ssh/ for information

```
ssh -A mayfield@kuri-SERIAL_NUMBER
git clone git@github.com:MayfieldRoboticsPublic/kuri_edu.git
```

The result should be a kuri_edu folder in the home directory of the mayfield
user on the robot

```
mayfield@kuri-0001c:~$ ls -l ~
total 24
drwxrwxr-x 6 mayfield mayfield 4096 Sep 17 18:24 kuri_edu
```

### Setting up a catkin workspace
Catkin workspaces let you overlay changes to individual ROS packages onto your
robot.  The stock kuri_edu software is located at /opt/kuri_edu
To create a catkin workspace that overlays kuri_edu, start from your
home directory:

```
mkdir -p workspace/src
cd workspace/src
ln -s ~/kuri_edu
cd ~/workspace
```

To build the workspace, you must first tell your shell session where to
find the installed software that you are going to overlay.  This is called
'sourcing the installed software'

```
source /opt/edu/setup.sh
```

Finally, you are ready to build your workspace
```
catkin_make
```

Upon success, you should see a `build` and `devel` folder.  Sourcing
workspace/devel/setup.sh will tell your shell session where to find
your version of the software

```
source workspace/devel/setup.sh
```

See http://wiki.ros.org/catkin/Tutorials/create_a_workspace for more
information about catkin workspaces

### Running Your Changes
Now, when you do

```
roslaunch kuri_edu kuri_edu.launch
```

Kuri will run the software linked into your workspace instead of the
installed software.  You are now ready to start making changes to Kuri.  To
verify that your changes are being run, add the following to the 'run'
method of the head controller in 
~/kuri_edu/kuri_edu/src/kuri_edu/head_controller.py

```
def run (self):

    rospy.logerr("This is a test error in the head controller")  # <-- Add this line

```

Now when you start the kuri_edu software with
```
roslaunch kuri_edu kuri_edu.launch
```
You should see your error text output to the console.

You are now ready to start trying other activities!  Look at the other files
in the activities directory to get some ideas of things to do with Kuri
