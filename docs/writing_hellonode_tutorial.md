
# Stretch ROS Tutorial: Stretch_core Motion Interface

In this tutorial, we will write ROS software to move the Stretch RE1. By the end of this tutorial, we will have created a Python ROS node that sends simple motion commands to the robot.

Robotic Operating System (ROS) is a framework for robotics developers. It acts as a middleware that abstracts away non-robotics problems like processing, networking, and more. Over the years, an ecosystem of packages have been built on ROS, providing solutions to a wide range of robotics problems. The learning curve of ROS can be steep, but learning how to develop ROS software will enable you to take advantage of the ecosystem and quickly prototype robot applications.

Hello Robot provides a number of ROS packages for working with Stretch. These packages are actively developed on Github at the [stretch_ros repository](https://github.com/hello-robot/stretch_ros). In this tutorial, we will work primarily with two of these packages: [stretch_core](https://github.com/hello-robot/stretch_ros/tree/master/stretch_core) and [hello_helpers](https://github.com/hello-robot/stretch_ros/tree/master/hello_helpers).

## Setup

Stretch comes setup with the environment needed to begin developing ROS software. If you would like to learn how this environment was setup, you may refer to the [beginner tutorials](http://wiki.ros.org/ROS/Tutorials) written by the ROS organization.

This environment includes a ROS "workspace", which is where ROS software is developed and built. The workspace is simply a folder. On your robot, it is located in the home folder and called `catkin_ws`. The workspace has the following folder hierarchy:

```
/home/hello-robot/catkin_ws/
	build/
	csm/
	devel/
	install/
	src/
		stretch_ros/
			hello_helpers/
			stretch_core/
			<other Stretch ROS packages>/
		<other ROS packages>/
```

The `src/` folder is where ROS software is kept. ROS software is organized into ROS "[packages](http://wiki.ros.org/Packages)", which are comprised of configuration files and code. The actual ROS code we write resides within these packages as ROS "[nodes](http://wiki.ros.org/Nodes)". A ROS node is a C++ or Python file, and runs as a process with other ROS nodes to accomplish higher level tasks.

Before writing our own ROS node, let's review the most commonly used ROS nodes from the stretch_core package:
 * stretch_driver

We will develop a ROS node that communicates with the [stretch_driver](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/stretch_driver) node from the stretch_core ROS package. These two nodes will work together to make the robot move. The first step is to generate a ROS package. A package creation script will assist in setup of the folders and configuration files. The format of the script is `catkin_create_pkg <package_name> [<required_package1>] [<required_package1>]`.

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg hello_tutorials rospy stretch_core
```

The command generates a ROS package called hello_tutorials that requires the stretch_core and [rospy](wiki.ros.org/rospy) (Python interface to the ROS framework) packages to function. hello_tutorials has the following file hierarchy:

```
/home/hello-robot/catkin_ws/src/hello_tutorials/
	CMakeLists.txt
	package.xml
	src/
```

To understand how to customize the `CMakeLists.txt` and `package.xml` files, you may follow the [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) tutorial. We will continue with the default setup and create our ROS node. The next step is to navigate to the `hello_tutorials/src/` folder and create a Python node file called `example_node`.

```
$ cd ~/catkin_ws/src/hello_tutorials/src/
$ touch example_node
$ chmod +x example_node
```

With the empty node created, we will use the Catkin build system to inform ROS about the package and its nodes.

```
$ cd ~/catkin_ws
$ catkin_make
```

## Architecture


