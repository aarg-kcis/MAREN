import os
import sys
import yaml
import argparse
import shutil
import subprocess as sp

def check_dir():
  if not (os.getcwd()).endswith("MAREN/multi_agent_gazebo_env/Scripts"):
    print ("Script run from outside the desired location.")
    print ("Changing path")
    os.chdir(os.path.join('/', *(os.path.abspath(__file__).split('/')[:-1])))
    print ("path cahnged to {}".format(os.getcwd()))

def create_catkin_ws(env_path):
  os.chdir(env_path)
  if not os.path.exists("catkin_ws/src"):
    print ("Creating catkin workspace ...")
    os.makedirs("catkin_ws/src")
  else:
    print ("Catkin workspace already exists ...")
  os.chdir("catkin_ws/src")
  return os.getcwd()

def init_catkin_ws(env_path, catkin_path):
  os.chdir(catkin_path)
  if not os.path.isfile("CMakeLists.txt"):
    print ("Initializing catkin workspace ...")
    sp.Popen(["catkin_init_workspace"])
    print ("Added CMakeLists.txt to {}".format(catkin_path))
  print ("Downloading packages ...")
  try:
    package_file = os.path.join(env_path, "packages.repos")
    sp.call("vcs import < {}".format(package_file), shell=True)
    print ("Done downloading ...")
  except Exception as e:
    print (e)
  os.chdir(env_path)

def apply_modifications(env_path, catkin_path):
  mod_path = os.path.join(env_path, "mods")
  files = [os.path.join(dp, f) for dp, dn, fnames in os.walk(mod_path) for f in fnames]
  for file in files:
    dest = os.path.join(catkin_path, *(file.split("mods")[-1].split('/')))
    if os.path.isfile(dest):
      print ("Replacing file: {}".format(dest))
    else:
      print ("Creating file: {}".format(dest))
    shutil.copy2(file, dest)
    print ()

def get_configuration(env_path):
  try:
    with open(os.path.join(env_path, "config.yaml")) as c_file:
      config = yaml.load(c_file)
      return config
  except Exception as e:
    print ("Problem in loading/parsing config file ...")

def add_catkin_ignore(ignore, catkin_path):
  try:
    for k, v in ignore.items():
      for pkg in v:
        print ("Adding CATKIN_IGNORE to package -> {}:{}".format(k, pkg))
        f = os.path.join(catkin_path, k, pkg, "CATKIN_IGNORE")
        p = open(f, "w+").close()
  except Exception as e:
    print ("Error in adding CATKIN_IGNORE for package -> {}:{}".format(k, pkg))
    print (e)

def catkin_make(catkin_path):
  os.chdir(catkin_path)
  os.chdir("../")
  print ("Running catkin_make at: {}".format(catkin_path))
  sp.call("catkin_make", shell=True)

def setup_environment(env):
  s = 0
  check_dir()
  env_path = os.path.join(os.getcwd(), "..", "Environments", env)
  catkin_ws = create_catkin_ws(env_path)
  init_catkin_ws(env_path, catkin_ws)
  apply_modifications(env_path, catkin_ws)
  config = get_configuration(env_path)
  add_catkin_ignore(config["catkin_ignore"], catkin_ws)
  catkin_make(catkin_ws)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="Environment to setup", type=str)
  args = parser.parse_args()
  setup_environment(args.env)
