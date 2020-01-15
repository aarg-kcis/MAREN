# MAREN
Multi Agent Robotics Environment Based on Gazebo and ROS. For Gym integration refer [here](https://github.com/aarg-kcis/MAREN-GYM)

## Prerequisites
1. ROS Kinetic  
   Install ROS kinetic by following the instructions [here](http://wiki.ros.org/kinetic/Installation).  
   If you installed the minimal version of ROS, then make sure you install the following packages as well.  
   
   ```bash
   pip install --user python-rospkg catkin_pkg
   
   sudo apt install \
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
   ```
2. Classic Dependencies  
   ```bash
   sudo apt install python3-defusedxml python3-vcstool
   ```
3. Gazebo (>=7.0.0)  
   Gazebo is already installed with `ros-kinetic-desktop-full`.  
   If you installed some other configuration for ROS make sure to install Gazebo as well as Gazebo-ROS pkgs.
   - Install gazebo from [here](http://gazebosim.org/tutorials?cat=install).
   - Gazebo-ROS packages from [here](http://gazebosim.org/tutorials?tut=ros_installing).
<!-- 4. OpenAI Gym  
   - If you are using a virtual environment the source the environment and install gym using  
     `pip install gym`  
     or else
   - install gym system-wide using  
     `pip install --user gym` -->

## Setting up
Setting up is easy. Just run 
```bash
python setup_catkin.py <environment-name> #kobuki_simple_world
```
here `<environment-name>` corresponds to the environment folder's name that you want to setup.  
As of now there is only one environment in the repository, but you can always get creative and make your own. You can also contribute by sending a PR (pull request) with your environment and we'll make sure to merge it if it checks out.   
Running this script will automatically create a catkin workspace in the `<environment-name>` folder.  
Be sure to source the `devel/setup.bash` script after this step.

When this is done you can simply run the `env_spawner.py` script:
```bash
python env_spawner.py <environment-name> #kobuki_simple_world
```

You should now see a gazebo window popup with three kobuki bots.

## Creating your own environments (God Mode!)
To create your own environments in MAREN, follow the steps below (Here we are going to assume your current working directory is MAREN):
- Create a new folder with your environment name (seperate words with underscores for consistent naming convention)
  ```bash
  mkdir Environment/some_awesome_environment
  ```

- Add a `packages.repos` file consisting of all your ros packages that you'll need for the environment or, edit edit the `packages.repos` from the default `kobuki_simple_world` environment.  
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

- Sometimes you may want to make some changes to the files inside the packages. To do so, just create a folder called `mods` inside your environment and add the files that are to be modified. Make sure you follow the exact tree stucture of the files. Please refer to the mods folder in `kobuki_simple_world/assets` as an example.
#### Example:
If you want to make changes to the file `kobuki.urdf.xacro` thats resides in `catkin/src/kobuki/kobuki_description/urdf/kobuki.urdf.xacro`, just put your modified file to `mods/kobuki/kobuki_description/urdf/kobuki.urdf.xacro`. Here `catkin_ws` and `mods` are directory siblings.


#### Config Dissection:
- Add a `config.yaml` file to your awesome environment. It should look like this:
```yaml
num_agents: 3
agent_ns: kobuki
world_launchfile: kobuki_simple_world.launch
world_file: simple.world

roslaunch:
    onetime: null
    roslaunch_per_agent:
        $launch/kobuki.launch:
            model: $NS
            robotID: $ID
            y: $ID
            x: 0

run_scripts:
    onetime: null
    run_scripts_per_agent:
        $scripts/KobukiTfBroadcaster.py:
            - $NS_$ID
catkin_ignore:
    kobuki: 
        - kobuki_keyop
        - kobuki_controller_tutorial
        - kobuki_random_walker
        - kobuki_auto_docking
        - kobuki_controller_tutorial
        - kobuki_node
        - kobuki_testsuite
        - kobuki_safety_controller
    kobuki_desktop:
        - kobuki_qtestsuite
```
 - `num_agents`: self explanatory, the number of agents that you need to spawan in the environment
 - `agent_ns`: the namespace of the agents. Every agent would be given this namespace as prefix, followed by its id. Eg. `kobuki_1`, `kobuki_2`.
 - `world_launchfile`: the launchfile of the world, we recommend that you use an `empty_world.launch` file. Needs to be in `<env_name>/assets/launch` directory.
 - `world_file`: the world file to pass to yhe above launch file. Needs to be in `<env_name>/assets/worlds` directory.
 - `roslaunch` and `run_scripts`: mention all the launchfiles/ python scripts that you need to launch here, either in `onetime` or in `roslaunch(run_scripts)_per_agent` category. If you want to launch something from another package mention it as `<pkg>/<launchfile>` followed by parameters. For files placed inside the `<env_name>/assets` folder, you can access them by adding `$` to whatever needs to be accessed. Eg. `$launch/a.launch` would refer to a file with path `<env_name>/assets/launch/a.launch`.
   - parameters to launchfiles and scripts can be provided as:
      - keyword `a: 4`
      - unnamed `- a`
      - no params required `null`
   - `onetime`: launchfiles/scripts placed inside are launched only once per environment.
   - `roslaunch_per_agent`: launchfiles/scripts placed here are launched for each agent spawned in the environment.
  - If for some reasons you want to keep your environment workspace lean and want to include only a bunch of subpackages from the repositories mentioned in your `packages.repos` then you can add the names of packages to ignore inside your `config.yaml` file (shown above). 
