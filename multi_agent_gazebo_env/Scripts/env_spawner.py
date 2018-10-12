import os
import sys
import time
import rospy
import argparse
import subprocess as sp
from setup_catkin import get_configuration

def check_dir():
  os.chdir(os.path.join("/", *(os.path.abspath(__file__).split("/")[:-1])))
  print ("Path cahnged to {}".format(os.getcwd()))
  
def try_spawning_environment(env_path, env):
  config = get_configuration(env_path)
  try:
    config["world_launchfile"] = os.path.join(env_path, "assets", "launch", config["world_launchfile"])
    config["agent_launchfile"] = os.path.join(env_path, "assets", "launch", config["agent_launchfile"])
    config['world_file'] = os.path.join(env_path, "assets", "worlds", config["world_file"])
  except Exception as e:
    print ("Make sure that (world/agent)_launchfile is present in config.yaml")
    print (e)
    kill_all()
    sys.exit(1)
  try:
    ros_path, ros_port = spawn_world(config['world_launchfile'], config['world_file'])
  except Exception as e:
    print ("Error launching world file ...")
    print (e)
    kill_all()
    sys.exit(1)
  try:
    spawn_agents(ros_path, ros_port, config["num_agents"], config["agent_launchfile"], config["agent_ns"])
  except Exception as e:
    print ("Error spawning agents ...")
    print (e)
    kill_all()
    sys.exit(1)

def kill_all():
  for i in processes:
    os.system("kill -9 {}".format(i))

def spawn_world(launchfile, worldfile):
  ros_port = os.environ.get("ROS_PORT_SIM", "11311")
  ros_path = os.path.dirname(sp.check_output(["which", "roscore"]))
  roscore  = sp.Popen([sys.executable, os.path.join(ros_path, b"roscore"), \
                               "-p", ros_port])
  processes.append(roscore)
  time.sleep(2)
  print ("ROSCORE LAUNCHED ...")
  if not os.path.exists(launchfile):
    raise IOError("Launch file: [{}] does not exist".format(launchfile))
    kill_all()
  print ("LAUNCHING GAZEBO ...")
  roslaunch = sp.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), \
                                "-p", ros_port, launchfile, "world_file:={}".format(worldfile)])
  processes.append(roslaunch)
  time.sleep(2)
  print ("GAZEBO LAUNCHED ...")
  return ros_path, ros_port


def spawn_agents(ros_path, ros_port, num_agents, agent_launchfile, model_name):
  print ("LAUNCHING AGENTS NOW ...")
  for i in range(num_agents):
    try:
      p = sp.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), \
                    "-p", ros_port, agent_launchfile, \
                    "model:={}".format(model_name), \
                    "robotID:={}".format(i), \
                    "x:={}".format(i), "y:={}".format(i)])
      processes.append(p)
      time.sleep(2)
    except Exception as e:
      print (e)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="Environment to setup", type=str)
  args = parser.parse_args()
  processes = []
  env_path = os.path.join(os.getcwd(), "..", "Environments", args.env)
  try:
    try_spawning_environment(env_path, args.env)
  except Exception as e:
    print (e)
    kill_all()