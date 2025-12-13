#!/usr/bin/env python3

import subprocess
import sys


class RosNode:
    """
    Helper class to launch ROS2 nodes with parameters.
    """
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable
        self._param_list = []

    def add_parameters(self, namespace, parameter_dictionary):
        """
        Add parameters from a dictionary, supporting nested dictionaries.
        """
        for key in parameter_dictionary.keys():
            if isinstance(parameter_dictionary[key], dict):
                self.add_parameters(namespace + key + '.', parameter_dictionary[key])
            else:
                param_name = namespace + key if namespace else key
                self._param_list.append(f'{param_name}:={parameter_dictionary[key]}')

    def run(self, parameter_dictionary, namespace=''):
        """
        Run the ROS2 node with the specified parameters.
        """
        self.add_parameters(namespace, parameter_dictionary)

        # Build ros2 run command
        cmd = ['ros2', 'run', self._package, self._executable]

        # Add parameters using --ros-args -p syntax
        if self._param_list:
            cmd.append('--ros-args')
            for param in self._param_list:
                cmd.extend(['-p', param])

        print('Starting ROS2 node with command: ' + ' '.join(cmd))

        result = subprocess.run(cmd)
        print('ROS2 node finished processing.')
        return result.returncode
