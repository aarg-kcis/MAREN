import os
import sys
import time
import rospy
import argparse
import subprocess as sp
from copy import deepcopy

class ROS_Initiator():
    def __init__(self, config, rospath, rosport, env_path):
        self.config = config
        self.rospath = rospath
        self.rosport = rosport
        self.env_path = env_path
        self.assets = os.path.join(self.env_path, 'assets')
        self.roslaunch = os.path.join(self.rospath, b'roslaunch')
        self.roscore = os.path.join(self.rospath, b'roscore')

        self.agent_ns = self.config['agent_ns']
        self.num_agents = self.config['num_agents']
        self.agents = [(i, '{}_{}'.format(self.agent_ns, i)) 
                        for i in range(1, self.num_agents + 1)]
        self.pids = {}
        self.procs = {}
        self.setup_world()

    def setup_world(self):
        params = [sys.executable, self.roscore, '-p', self.rosport]
        self.run_once(params, 'roscore')
        params = [sys.executable, self.roslaunch, '-p', self.rosport,
                    os.path.join(
                        self.assets, 'launch',
                        self.config['world_launchfile']),
                    'world_file:={}'.format(
                        os.path.join(
                            self.assets, 'worlds',
                            self.config['world_file'])
                    )]
        self.run_once(params, 'world')

    def run_once(self, run_params, p_name):
        print('='*100)
        print("Running {}".format(p_name.replace('$', '')))
        print("\tParams: {}".format(run_params))
        print('='*100)
        self.pids[p_name] = sp.Popen(run_params, preexec_fn=os.setsid)
        time.sleep(1)

    def kill_all(self):
        k = 'roscore'
        v = self.pids[k]
        print('Killing {} with PID {}'.format(k, v.pid))
        os.killpg(os.getpgid(v.pid), 15)
        for k, v in self.pids.items():
            if k == 'roscore': continue
            print('Killing {} with PID {}'.format(k, v.pid))
            os.killpg(os.getpgid(v.pid), 15)

    def wait_for_kill(self):
        import time
        k = self.pids.values()
        for i in k:
            ctr = 1
            t = time.time()
            print("Waiting for process with pid {} to die".format(i.pid))
            while i.poll() is None:
                if time.time() - t > 1:
                    sys.stdout.write('{:<10}'.format('.'*ctr)+'\r')
                    ctr = ctr % 10 + 1
                    t = time.time()
                sys.stdout.flush()


    def launch(self):
        self.procs['roslaunch'] = self.launch_with_type('roslaunch')
        self.procs['run_scripts'] = self.launch_with_type('run_scripts')
        params = [sys.executable, self.roslaunch, '-p', self.rosport]
        for key, val in self.procs['roslaunch'].items():
            self.run_once(params + val, key)
        params = [sys.executable]
        for key, val in self.procs['run_scripts'].items():
            self.run_once(params + val, key) 

    def launch_with_type(self, _type):
        p = {}
        for k, v in self.config[_type]['onetime'].items():
            p['onetime->'+k] = self.parse_args(k, v)
        for k, v in self.config[_type]['{}_per_agent'.format(_type)].items():
            for agent_id, agent in self.agents:
                key = '{}->{}'.format(agent, k)
                p[key] = self.parse_args(k, v, agent_id)
        return p

    def parse_args(self, key, val, agent_id=None):
        args = []
        if key[0] == '$':
            key = key.replace('$', '')
            args.append(os.path.join(self.assets, key))
        else:
            ks = key.split('/')
            assert len(ks) == 2
            args.extend(ks)
        if val is None:
            return args
        if isinstance(val, list):
            return args + self.args_from_list(val, agent_id)
        elif isinstance(val, dict):
            return args + self.args_from_dict(val, agent_id)

    def prep_args(self, args, agent_id):
        new_args = deepcopy(args)
        if isinstance(args, list):
            iterator = enumerate(args)
        if isinstance(args, dict):
            iterator = args.items()
        for k, v in iterator:
            try:
                s = args[k].replace("$ID", str(agent_id))
                s = s.replace("$NS", str(self.agent_ns))
                new_args[k] = s
            except Exception as e:
                print('assssssssssss')
                pass 
        return new_args

    def args_from_dict(self, args, agent_id=None):
        return ['{}:={}'.format(k, v) for k, v in
                self.prep_args(args, agent_id).items()]

    def args_from_list(self, args, agent_id=None):
        return self.prep_args(args, agent_id)
