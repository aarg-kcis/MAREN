import os
import sys
import time
import rospy
import argparse
import subprocess as sp
from copy import deepcopy
from setup_catkin import get_configuration

def check_dir():
    os.chdir(os.path.join("/", *(os.path.abspath(__file__).split("/")[:-1])))
    print ("Path cahnged to {}".format(os.getcwd()))
  
def try_spawning_environment(env_path, env):
    config = get_configuration(env_path)
    try:
        assets_path = os.path.join(env_path, "assets")
        config["world_launchfile"] = os.path.join(assets_path, "launch", config["world_launchfile"])
        config["agent_launchfile"] = os.path.join(assets_path, "launch", config["agent_launchfile"])
        config['world_file'] = os.path.join(assets_path, "worlds", config["world_file"])
        agent_launchfile_params = config["agent_launchfile_params"] or {}
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
        spawn_agents(ros_path, ros_port, config["num_agents"], config["agent_launchfile"], agent_launchfile_params, config["agent_ns"])
    except Exception as e:
        print ("Error spawning agents ...")
        print (e)
        kill_all()
        sys.exit(1)

    try:
        run_per_agent(assets_path, config["num_agents"], config["run_per_agent"], config["agent_ns"])
    except Exception as e:
        print ("Error running custom scripts per agent ...")
        print (e)
        kill_all()
        sys.exit(1)

    try:
        time.sleep(2)
        roslaunch_per_agent(ros_path, ros_port, config["roslaunch_per_agent"] or {}, config["num_agents"], config["agent_ns"])
    except Exception as e:
        print ("Error running custom launchfiles per agent ...")
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


def spawn_agents(ros_path, ros_port, num_agents, agent_launchfile, params, agent_ns):
    print ("LAUNCHING AGENTS NOW ...")
    param_key_list = list(params.keys())
    agent_params = deepcopy(params if '$ID' not in params else params['$ID'])
    for i in range(1, num_agents + 1):
        params[i] = agent_params
    for k in param_key_list:
        del params[k]


    for i in range(1, num_agents + 1):
        try:
            p = sp.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), \
                    "-p", ros_port, agent_launchfile] + prepare_params(params[i], i, agent_ns))
            processes.append(p)
            time.sleep(2)
        except Exception as e:
            print (e)


def prepare_params(params, agent_id, agent_ns):
    param = ['{}:={}'.format(k, v) for k, v in prepare_args(
            params, agent_id, agent_ns).items()]
    return param


def prepare_args(args, agent_id, agent_ns):
    new_args = deepcopy(args)
    if isinstance(args, list):
        iterator = enumerate(args)
    if isinstance(args, dict):
        iterator = args.items()
    for k, v in iterator:
        try:
            s = args[k].replace("$ID", str(agent_id))
            s = s.replace("$NS", str(agent_ns))
            new_args[k] = s
        except Exception as e:
            print('Ignoring key [{}], setting default value = [{}]'.format(k, v))
            # print(e)
            pass 
    return new_args


def run_per_agent(assets_path, num_agents, run_files, agent_ns):
    for file, args in run_files.items():
        for i in range(1, num_agents + 1):
            print("LAUNCHING {} for {}_{}".format(file, agent_ns, i))
            file_path = os.path.join(assets_path, file)
            try:
                p = sp.Popen(
                    [sys.executable, file_path] +
                    prepare_args(args, i, agent_ns)
                )
            except Exception as e:
                print(e)

def roslaunch_per_agent(ros_path, ros_port, launchfiles, num_agents, agent_ns):

    for lf, params in launchfiles.items():
        param_key_list = list(params.keys())
        agent_params = deepcopy(
            params if '$ID' not in params else params['$ID']
        )
        for i in range(1, num_agents + 1):
            params[i] = agent_params
        for k in param_key_list:
            del params[k]
        print(params)
        pkg, lf = lf.split('/')
        for i in range(1, num_agents + 1):
            print("LAUNCHING {} {} for {}_{}".format(pkg, lf, agent_ns, i))
            try:
                p = sp.Popen([
                    sys.executable,
                    os.path.join(ros_path, b"roslaunch"),
                    "-p", ros_port, pkg, lf
                    ] + prepare_params(params[i], i, agent_ns)
                )
                processes.append(p)
                time.sleep(2)
            except Exception as e:
                print(e)

def roslaunch_extras(ros_path, ros_port, launchfiles):



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="Environment to setup", type=str)
    args = parser.parse_args()
    processes = []
    check_dir()
    env_path = os.path.join(os.getcwd(), "..", "Environments", args.env)
    try:
        try_spawning_environment(env_path, args.env)
    except Exception as e:
        print (e)
        kill_all()