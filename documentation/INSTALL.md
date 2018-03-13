# Install notes

## Ubuntu Mate

Begin by installing ubuntu mate for Raspberry Pi

## ROS

Install ROS for Ubuntu Mate on Raspberry Pi

## Installing dependencies

### Git

```
sudo apt-get install git
```

Note: if we want to automate we need to pass a flag for no-confirmation

## Configure the RPi

### I2C

Change the file `/boot/config.txt` to add the line:

```
dtparam=i2c1=on
```

Then reboot.

Normally, smbus and i2c-tools are included in the installation
Ubuntu Mate. (needs confirmation)

## ROS environement

After the installation of ROS, you need to source the environment
setup file: `/opt/ros/<ros_version>/setup.bash`.

The easiest way, to have persistent sourcing even after reboot,
is to add the sourcing command to the `.bashrc` file.

```
source /opt/ros/kinetic
```

### Catkin Workspace
You need to create a catkin workspace for ROS:

```
mkdir -p ~/eurobot_ws/src
cd ~/eurobot_ws/
catkin_make
```

#### Source the environment
Once that is done a new setup file has been created that we need
to source. For convenience, add the following line to your `.bashrc`

```
source ~/eurobot_ws/devel/setup.bash
```

### ROS dependencies

The following dependencies need to be installed:

#### [rosserial](http://wiki.ros.org/rosserial)
```
sudo apt-get install ros-kinetic-rosserial
```

#### [rosserial-arduino](http://wiki.ros.org/rosserial_arduino)
[(setup tutorial)](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
```
sudo apt-get install ros-kinetic-rosserial-arduino
```
#### [map_server](http://wiki.ros.org/map_server)
```
sudo apt-get install ros-kinetic-map-server
```

#### [move_base](http://wiki.ros.org/move_base)
```
sudo apt-get install ros-kinetic-move-base
```

## Source code

### Cloning the repository
The next step is to clone the repository containing our code:
```
cd ~/eurobot_ws/src/
git clone https://github.com/Ecam-Eurobot/Eurobot-2018.git
```

