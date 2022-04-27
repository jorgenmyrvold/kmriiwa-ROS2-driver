# ROS2-OPCUA-MIDDLEWARE
This repository is part of the Master's project conducted by Andreas Chanon Arnholm and Mathias Neslow Henriksen during the spring of 2021.

The ROS2-OPCUA-MIDDLEWARE is what connects a [ROS2-ENTITY](https://github.com/TPK4960-RoboticsAndAutomation-Master/ROS2-ENTITY) with the [AAS](https://github.com/TPK4960-RoboticsAndAutomation-Master/AAS). It is a ROS 2 program that spins a *publisher-subscriber hybrid* node that is used to receive (*subscribe* to) status updates from an entity in the form of ROS messages and forward (*publish*) it over OPC UA to the AAS, and to receive commands from the AAS over OPC UA and forward it in the form of ROS messages to an entity.   

## Structure
Project folders:
* [ros2](ros2): ROS 2 workspace. 

Bash scripts:
* [ros2/auto.sh](ros2/auto.sh): the main bash script that starts everything.
* [ros2/build.sh](ros2/build.sh): bash script for configuring, building and running the ROS 2 program.

ROS 2 Package:
* [ros2/kmr_communication/kmr_communication](ros2/kmr_communication/kmr_communication): middleware-related code. This currently only consists of code related to the KMR iiwa, but can be expanded for other robots. It should really be named "entity_communication".
* [ros2/kmr_communication/package.xml](ros2/kmr_communication/package.xml): dependencies for the ROS 2 package.
* [ros2/kmr_communication/setup.py](ros2/kmr_communication/setup.py): this is where one speciefies which files should be included when the package is built and prepared for execution. 

Middleware-related code:
* [ros2/kmr_communication/kmr_communication/config](ros2/kmr_communication/kmr_communication/config): yaml-files with parameters for the hybrid node. Not a lot here currently, but it might be expanded in the future with the introduction of more robots.
* [ros2/kmr_communication/kmr_communication/launch](ros2/kmr_communication/kmr_communication/launch): launch script that spins the hybrid node with correct parameters.
* [ros2/kmr_communication/kmr_communication/nodes](ros2/kmr_communication/kmr_communication/nodes): all ROS nodes relating to middleware communication, which currently only includes the one hybrid node.


## Setup
There are dependencies that need to be installed on the system before the program can be executed. 

### ROS 2
The middleware can be run on any operating system that supports ROS 2 and OPC UA. If the purpose is to run the middleware on a Raspberry Pi with an attached [SIM8200EA-M2 5G HAT](https://www.waveshare.com/wiki/SIM8200EA-M2_5G_HAT), this Raspberry Pi needs to run Raspberry Pi OS in order to support the HAT's drivers (as of April 2021). The reason as to why this specific information is of importance is that ROS 2 is installed differently depending on operating system.

#### ROS 2 on Raspberry Pi OS (from source)
The steps outlined here are inspired by [this guide](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304).

1. First, follow the steps from the [guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) on the official ROS 2 website until the step **Install dependencies using rosdep**.
2. Try doing the step, and if an error occurs regarding *libgl-dev*, do this:
    1. `$ ls /var/cache/apt/archives/*libgl*` 
    2. Copy the package name 
    3. `$ sudo dpkg –i ––force–overwrite /var/cache/apt/archives/<full_name_of_package_from_copy>`
3. Do the following to ignore some unnecessary features:
    1. `$ cd ~/ros2_foxy/`
    2. `$ touch src/ros2/rviz/AMENT_IGNORE` 
    3. `$ touch src/ros-visualization/AMENT_IGNORE`
    4. `$ touch src/ros2/system_tests/AMENT_IGNORE`
4. Set some additional build flags to make all builds succeed. These needs to be saved as Colcon defaults so that they are used automatically:
    1. `$ mkdir ~/.colcon && cd. ~/.colcon`
    2. `$ touch defaults.yaml`
    3. `$ sudo nano defaults.yaml`
    4. Insert the following:
      ```
      build: 
      cmake-args: 
      - -DCMAKE_SHARED_LINKER_FLAGS='-latomic -lpython3.7m' 
      - -DCMAKE_EXE_LINKER_FLAGS='-latomic -lpython3.7m' 
      - -DCMAKE_BUILD_TYPE=RelWithDebInfo
      ```
5. Continue the ROS 2 guide from the step **build the code in workspace**.

#### ROS 2 on linux distributions that support ROS 2 binaries
This is more straightforward. We recommend installing ROS 2 via [Debian Packages](https://docs.ros.org/en/crystal/Installation/Linux-Install-Debians.html).

### Other dependencies
1. `$ sudo apt-get install jq`
2. `$ sudo apt-get moreutils`
3. `$ pip3 install yq`

## Usage
`$ cd ros2`

### With auto.sh
The auto.sh script is meant to be run on start-up. It includes a *git pull*, and you should therefore clone the repo using ssh to avoid failure.

If you are certain that everything is set up correctly, do the following: 
1. `$ chmod a+x build.sh`
2. `$ chmod a+x auto.sh`
3. `$ bash auto.sh`

### With build.sh
1. `$ chmod a+x build.sh`
2. `$ bash build.sh <build> <mode>`

With ROS 2 binary build and a local OPC UA server (0.0.0.0):

`$ bash build.sh binary test`

With ROS 2 source build and public OPC UA server (public IP):

`$ bash build.sh source_ prod`

### Manual usage
For this, make sure that the [config file](ros2/kmr_communication/kmr_communication/config/bringup.yaml) has the correct parameters for whatever it is you want to do.

While in the `ros2` folder, do the following:
1. `$ colcon build --symlink-install`
2. `$ source install/setup.bash`
3. `$ ros2 launch kmr_communication hybrid.launch.py`
