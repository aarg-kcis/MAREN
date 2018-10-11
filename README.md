# MAREN
Multi Agent Reinforcement Learning Environment Based on Gazebo and ROS

## Prerequisites
1. ROS Kinetic  
   Install ROS kinetic by following the instructions [here](http://wiki.ros.org/kinetic/Installation).  
   If you installed the minimal version of ROS, then make sure you install the following packages as well.  
   
   ```bash
   sudo pip3 install rospkg catkin_pkg
   
   sudo apt install \
   ros-kinetic-octomap-msgs \
   ros-kinetic-joy \
   ros-kinetic-geodesy \
   ros-kinetic-octomap-ros \
   ros-kinetic-control-toolbox \
   ros-kinetic-pluginlib \
   ros-kinetic-trajectory-msgs \
   ros-kinetic-control-msgs \
   ros-kinetic-std-srvs \
   ros-kinetic-nodelet \
   ros-kinetic-urdf \
   ros-kinetic-rviz \
   ros-kinetic-kdl-conversions \
   ros-kinetic-eigen-conversions \
   ros-kinetic-tf2-sensor-msgs \
   ros-kinetic-pcl-ros \
   ros-kinetic-navigation
   ```
2. Classic Dependencies  
   ```bash
   sudo apt install libbullet-dev \
   python3-defusedxml python3-vcstool \
   cmake gcc g++ qt4-qmake libqt4-dev \
   libusb-dev libftdi-dev python3-pyqt4
   ```
3. Gazebo (>=7.0.0)  
   Gazebo is already installed with `ros-kinetic-desktop-full`.  
   If you installed some other configuration for ROS make sure to install Gazebo as well as Gazebo-ROS pkgs.
   - Install gazebo from [here](http://gazebosim.org/tutorials?cat=install).
   - Gazebo-ROS packages from [here](http://gazebosim.org/tutorials?tut=ros_installing).
4. OpenAI Gym  
   - If you are using a virtual environment the source the environment and install gym using  
     `pip install gym`  
     or else
   - install gym system-wide using  
     `pip install --user gym`

## Setting up

## Creating your own environments (God Mode!)
To create your own environments in MAREN, edit the `packages.repos` 
to include all the packages that you'll need for the environment. 
#### Example:
```yaml
repositories:
  <pkg1-name>:
    type: git
    url: <github-repo-link-to-pkg1>
    version: <branch-name>
    
  <pkg2-name>:
    type: git
    url: <github-repo-link-to-pkg2>
    version: <branch-name>
```
> NOTE: If you have packages installed in your main ros workspace @(`/opt/ros/kinetic/share`), 
  they will still work with MAREN. But its advised that you specify all the packages that your 
  environment depends on, in your project's `packages.repos`
  so that its easier for your teammates to set MAREN up and they don't end up hating you forever. :grin:
