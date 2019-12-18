import os
import sys
import time
import rospy
import argparse
import subprocess as sp
from copy import deepcopy
from setup_catkin import get_configuration
from initiator import ROS_Initiator

def check_dir():
    os.chdir(os.path.join("/", *(os.path.abspath(__file__).split("/")[:-1])))
    print ("Path cahnged to {}".format(os.getcwd()))


if __name__ == "__main__":
    from pprint import pprint
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="Environment to setup", type=str)
    args = parser.parse_args()
    processes = []
    check_dir()
    env_path = os.path.join(os.getcwd(), "..", "Environments", args.env)
    ros_port = os.environ.get("ROS_PORT_SIM", "11311")
    ros_path = os.path.dirname(sp.check_output(["which", "roscore"]))
    config = get_configuration(env_path)
    initiator = ROS_Initiator(config, ros_path, ros_port, env_path)
    initiator.launch()
    pprint({k: v.pid for k, v in initiator.pids.items()})
    # pprint(initiator.procs)
    try:
        while(input() != 'k'):
            pass
    except KeyboardInterrupt:
        initiator.kill_all()
        print('KeyboardInterruptzjknfjnkidan')
        initiator.wait_for_kill()
    except Exception as e:
        print(e)
        print('asdnjabjfbujb')