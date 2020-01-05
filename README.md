# ROS TurtleSim with Docker and JS

ROS ([Robot Operating System](http://wiki.ros.org/ROS/Introduction)) is an open-source, meta-operating system for robots. It provides the services you would expect from an operating system. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

[Catkin](http://wiki.ros.org/catkin) is a low-level build system macros and infrastructure for ROS. [Catkin Command Line Tools](https://catkin-tools.readthedocs.io).

[Rosbridge](http://wiki.ros.org/rosbridge_suite) provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more. Rosbridge supports a variety of transport layers, including WebSockets and TCP.

[Roslibjs](http://wiki.ros.org/roslibjs) is the core JavaScript library for interacting with ROS from the browser. It uses WebSockets to connect with rosbridge.

`Rosbridge` and `roslibjs` are released as part of the [Robot Web Tools](http://robotwebtools.org/) effort.

[Turtlesim](http://wiki.ros.org/turtlesim) is a tool made for teaching ROS and ROS packages.

[Docker](https://docker-curriculum.com) is a tool that allows developers, sys-admins etc. to easily deploy their applications in a sandbox (called containers) to run on the host operating system i.e. Linux.

You can create ROS programs mainly in two programming languages: [Python and C++](https://www.theconstructsim.com/learn-ros-python-or-cpp/). I will prefer Python in this tutorial, but pull requests are welcome for cpp.

## Turtlesim Docker Architecture

![turtlesim-docker-architecture](turtlesim_docker.png)

## Environment

OS: Ubuntu 18.04.3 LTS \
ROS: melodic \
Docker version: 19.03

## Create a project directory

```sh
cd ~
mkdir turtlesim
```

## ROS master node

First we need a private Docker network for Turtlesim:
```sh
docker network create turtlesim
```

Start a ROS master:

```sh
docker container run -dit \
    --net turtlesim \
    --name ros-turtlesim-master \
    ros:melodic-ros-base \
    roscore
```

## Catkin Workspace

On host computer in the project directory:

```sh
mkdir catkin_ws
```

Start a container for Catkin:

```sh
docker run -it \
    --net turtlesim \
    --env ROS_HOSTNAME=ros-turtlesim-catkin \
    --env ROS_MASTER_URI=http://ros-turtlesim-master:11311 \
    -v ~/turtlesim/catkin_ws:/root/turtlesim/catkin_ws \
    --name ros-turtlesim-catkin \
    ros:melodic-ros-core \
    /bin/bash
```

Catkin setup in the container:

```sh
# Environment setup
source /opt/ros/melodic/setup.bash

# Build Catkin
cd /root/turtlesim/catkin_ws
mkdir src
catkin_make

# Devel environment setup
source devel/setup.bash
```

## Turtlesim package

```sh
# Create package with rospy and std_msg
cd src/
catkin_create_pkg turtlesim rospy std_msgs

# TODO - set up proper user rights
sudo chmod -R 777 turtlesim
mkdir scripts
cd ..

# Rebuild Catkin
catkin_make
```

## Parameters with launch file

Documentation: [Parameter server](http://wiki.ros.org/Parameter%20Server), [Launch](http://wiki.ros.org/roslaunch)

In Catkin container:

```sh
cd ~/turtlesim/catkin_ws
catkin_create_pkg turtlesim_init
cd ..
catkin_make
cd src/turtlesim_init
mkdir launch && cd launch
vim turtlesim.launch
```

Edit launch file:

```xml
<launch>
    <param name="/velocity" type="int" value="50" />
    <param name="/bg_color_b" type="int" value="255" />
    <param name="/bg_color_g" type="int" value="255" />
    <param name="/bg_color_r" type="int" value="255" />
</launch>
```

> You can also set and get params from CLI. Help: `rosparam -h`. But be aware that all params are removing when ROS master restarted.

Start launch file:

```sh
roslaunch turtlesim_init tutlesim.launch
```

> Note: ROS launch also start a ROS master if it does not exist.

## Service (srv) files

Documentation: [ROS Services](http://wiki.ros.org/Services), [srv files](http://wiki.ros.org/srv).

Create service (srv) files:

```sh
cd ~/turtlesim/catkin_ws/src/turtlesim
mkdir srv && cd srv
vim Velocity.srv
vim BgColor.srv
```

Velocity.srv:

```txt
uint8 velocity
---
bool
```

BgColor.srv:

```txt
uint8 bg_color_b
uint8 bg_color_g
uint8 bg_color_r
---
bool
```

Primitive types: [http://wiki.ros.org/msg](http://wiki.ros.org/msg)

Config package.xml and CMakeLists.txt:

```sh
cd ~/turtlesim/catkin_ws/src/turtlesim
vim package.xml
vim CMakeLists.txt
```

In package.xml:

```xml
<!-- Add after last <build_depend> line -->
<build_depend>message_generation</build_depend>
<!-- Add after last <exec_depend> line -->
<exec_depend>message_runtime</exec_depend>
```

In CMakeLists.txt:

```sh
# Add message_generation package
find_package(
  # Other packages
  message_generation
)
# Uncomment add_service_files and change the .srv lines
add_service_files(
    FILES
    Velocity.srv
    BgColor.srv
)
# Uncomment generate_messages
generate_messages(
  # Messages
)
# Uncomment the CATKIN_DEPENDS line
catkin_package(
  # Other packages
  CATKIN_DEPENDS ... message_runtime
  # Other packages
)
```

Build service files:

```sh
cd ~/turtlesim/catkin_ws
catkin_make
```

## Service (server) node

Create node py file on the host computer in the project directory:

```sh
cd ~/turtlesim/catkin_ws/src/scripts
vim turtlesim-ros-node-service.py
```

Edit: [turtlesim-ros-node-service.py](catkin_ws/src/turtlesim/scripts/turtlesim-ros-node-service.py)

> Note: In ROS, nodes are uniquely named. If two nodes with the same name are launched, the previous one is kicked off. The anonymous=True flag means that rospy will choose a unique name for our node so that multiple node can run simultaneously. E.g.: `rospy.init_node('name_of_the_node', anonymous=True)` and the name will be `name_of_the_node_<random_numbers>`.

Start a container for Service node:

```sh
docker container run -it \
    --net turtlesim \
    --env ROS_HOSTNAME=ros-turtlesim-service \
    --env ROS_MASTER_URI=http://ros-turtlesim-master:11311 \
    -v ~/turtlesim/catkin_ws:/root/turtlesim/catkin_ws \
    --name ros-turtlesim-service \
    ros:melodic-ros-base \
    /bin/bash
```

Start the Service node:

```sh
cd /root/turtlesim/catkin_ws/src/turtlesim/scripts
python turtlesim-ros-node-service.py
```

> TODO: Try run node with rosrun (With and without .py):
`rosrun turtlesim turtlesim-ros-node-service.py`.

List and info service server:

```sh
rosservice list
rosservice info /bg_color
```

Call service server manually:

```sh
rosservice call /bg_color "bg_color_g: 255
bg_color_b: 255
bg_color_r: 255"
```

## Client (Publisher and Service client) Node

Create node py file on the host computer in the project directory:

```sh
cd ~/turtlesim/catkin_ws/src/scripts
vim turtlesim-ros-node-client.py
```

Edit: [turtlesim-ros-node-client.py](catkin_ws/src/turtlesim/scripts/turtlesim-ros-node-client.py)

Start a container for Client node:

```sh
docker container run -it \
    --net turtlesim \
    --env ROS_HOSTNAME=ros-turtlesim-client \
    --env ROS_MASTER_URI=http://ros-turtlesim-master:11311 \
    -v ~/turtlesim/catkin_ws:/root/turtlesim/catkin_ws \
    --name ros-turtlesim-client \
    ros:melodic-ros-base \
    /bin/bash
```

Start the Client node:

```sh
cd /root/turtlesim/catkin_ws/src/turtlesim/scripts
python turtlesim-ros-node-client.py
```

## Check publisher node in the Catkin container

```sh
# Node list
# /turtlesim_client
rosnode list

# Node info
rosnode info /turtlesim_client

# List active topics
# /turtlesim_commands
rostopic list

# Echo publisher messages
# TODO - Commands from STDIN
rostopic echo /turtlesim_commands
```

## Listener (Subscriber) Node

Create node py file on the host computer in the project directory:

```sh
cd ~/turtlesim/catkin_ws/src/turtlesim/scripts
vim turtlesim-ros-node-listener.py
```

Edit: [turtlesim-ros-node-listener.py](catkin_ws/src/turtlesim/scripts/turtlesim-ros-node-listener.py)

Start a container for Listener node:

```sh
docker container run -it \
    --net turtlesim \
    --env ROS_HOSTNAME=ros-turtlesim-listener \
    --env ROS_MASTER_URI=http://ros-turtlesim-master:11311 \
    -v ~/turtlesim/catkin_ws:/root/turtlesim/catkin_ws \
    --name ros-turtlesim-listener \
    ros:melodic-ros-base \
    /bin/bash
```

Start the Listener node:

```sh
cd /root/turtlesim/catkin_ws/src/turtlesim/scripts
python turtlesim-ros-node-listener.py
```

## Check listener node in the Catkin container

```sh
# List nodes
rosnode list
# List topics
rostopic list
# Show topic info
rostopic info /turtlesim_commands
```

If you want check your subscriber node without a publisher, can publish manually:

```sh
# Olny one message will be published
rostopic pub -1 /turtlesim_commands std_msgs/String "data: 'Command test'"
# Publish messages continuously (5 Hz) - rate mode
rostopic pub -r 5 /turtlesim_commands std_msgs/String "data: 'Command test'"
```

## Bridge Node

Create a ROS Melodic Bridge Docker image:

```sh
# Change directory into the repo
cd /path/to/repo/turtlesim-ros-docker-js/ros-melodic-bridge
# Build image from Dockerfile
docker build -t ros-melodic-bridge .
# List images
docker images
```

Start a container for Bridge node:

```sh
docker container run -it \
    --net turtlesim \
    -p 9094:9090 \
    --env ROS_HOSTNAME=ros-turtlesim-bridge \
    --env ROS_MASTER_URI=http://ros-turtlesim-master:11311 \
    -v ~/turtlesim/catkin_ws:/root/turtlesim/catkin_ws \
    --name ros-turtlesim-bridge \
    ros-melodic-bridge \
    /bin/bash
```

Run rosbridge:

```sh
rosrun rosbridge_server rosbridge_websocket
```

## Webserver

Copy the source code of web application into the project direcctory:

```sh
cd ~/turtlesim
cp -a /path/to/repo/turtlesim-ros-docker-js/webserver .
```

Or create symbolic link:

```sh
ln -s /path/to/repo/turtlesim-ros-docker-js/webserver .
```

Start a container for Webserver:

```sh
docker container run -d \
    --net turtlesim \
    -p 8084:80 \
    -v ~/turtlesim/webserver:/usr/share/nginx/html:ro \
    --name turtlesim-nginx \
    nginx
```

Check the webserver: [http://<host_ip>:8084](http://localhost:8084).

![turtlesim-inprogress](turtlesim_inprogress.png)

## ROS Bag

You can record and replay a topic stream by [rosbag](http://wiki.ros.org/rosbag).

```sh
rosbag -h
rosbag record /turtlesim/<topic_name>
rosbag info <rosbag_filename>.bag
rosbag play <rosbag_filename>.bag
```

## ROS Action

In some cases, however, if the service takes a long time to execute, the user might want the ability to cancel the request during execution or get periodic feedback about how the request is progressing.
Parts: ActionClient, ActionServer and Action. Action Specification: Goal, Feedback, Result.

Further information: [actionlib](http://wiki.ros.org/actionlib).

## Troubleshooting

List all container IP addresses:

```sh
docker ps -q | xargs -n 1 docker inspect --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}} {{ .Name }}' | sed 's/ \// /'
```

Install `ping` and `telnet` command inside a container:

```sh
apt-get update
apt-get install -y iputils-ping telnet
```
