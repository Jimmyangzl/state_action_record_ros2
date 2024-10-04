import numpy as np
# from . import ROBOT_JOINT_LIMIT
import yaml
import os
from ament_index_python.packages import get_package_share_directory



def load_robot_model(robot_name):
    '''
    Return the robot model of class RobotFK with the parameters in yaml file
    '''
    # Get the current script's directory
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # Specify the path to the YAML file in the config_folder
    yaml_path = os.path.join(
        get_package_share_directory('visual_joint_collect'),
        'config',
        'config.yaml'
    )
    # yaml_path = os.path.join(current_dir, '../config/config.yaml')
    with open(yaml_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    return RobotFK(np.array(yaml_data[robot_name]['robot_model']), np.array(yaml_data[robot_name]['robot_joint_limit']), yaml_data[robot_name]['length_robot'])


def dh_transform(dh_parameters):
    '''
    Return the dh transform matrix T regarding dh parameters
    dh_parameters: alpha - a - d - theta, np.array(4)
    '''
    # extract the parameters
    alpha = dh_parameters[0]
    a = dh_parameters[1]
    d = dh_parameters[2]
    theta = dh_parameters[3]

    T = np.array([
        [np.cos(theta), - np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), - np.sin(alpha), - np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return T


class RobotFK:
    '''
    Class for computing robot forward kinematic and required parameters
    '''
    def __init__(self, robot_model, joints_limit, robot_length):
        # Robot DH model
        self.model = robot_model
        # Set the joint limit of robot
        self.joint_limits = joints_limit
        # Robot arm length for length scaling
        self.robot_length = robot_length
    def __repr__(self):
        return "Robot DH Table:\n%s" % self.model + "\nRobot Joint Limits:\n%s" % self.joint_limits
    
    def set_configuration(self, q):
        '''
        Set the configuration theta
        q: new configuration, np.array()
        '''
        self.model[:, -1] = q
        # Check if configuration exceeds joint limits
        for i in range(7):
            self.model[i, -1] = np.maximum(self.joint_limits[0, i], self.model[i, -1])
            self.model[i, -1] = np.minimum(self.joint_limits[1, i], self.model[i, -1])

    def get_ith_pose(self, i):
        '''
        Return the pose of the i th joint
        '''
        pose = np.eye(4)
        for j in range(i):
            pose = pose @ dh_transform(self.model[j, :])
        return pose
