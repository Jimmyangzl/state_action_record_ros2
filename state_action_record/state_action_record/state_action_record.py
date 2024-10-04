import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

from ament_index_python.packages import get_package_share_directory
from state_action_record.record_funcs import get_parser, get_config, get_msg_type
import os
import yaml
import h5py
import numpy as np


class ObsActSubscriber(Node):
    def __init__(self, args_parser):
        super().__init__('obs_act_subscriber')
        self.parser_args = args_parser.parse_args()
        # self.wrench_pub = self.create_publisher(Float64MultiArray,
        #                             '/wrench', 1)
        # self.wrench_filter = [OneEuroFilter(min_cutoff=1.0, beta=0.01, d_cutoff=1.0, freq=50.0) for _ in range(6)]
        self.sub_dict = [] # subscribers list
        self.signal_groups = {}
        self.signal_current_ = {}
        self.obs_current_ = {}
        self.act_current_ = {}
        self.configs = {}
    
    def set_args(self):
        parser_args_dict = vars(self.parser_args)
        config_path = os.path.join(
            get_package_share_directory('state_action_record'),
            'config',
            'config.yaml'
        )
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        for key, value in configs.items():
            if parser_args_dict[key]:
                value = parser_args_dict[key]
            setattr(self, key, value)
            self.get_logger().info(key + ': ' + str(value))
        return True

    def set_configuration(self):
        assert self.save_path, "No save path defined!"
        self.record_file = h5py.File(self.save_path, "w")
        for data_class in self.data_classes:
            self.configs[data_class] = get_config(data_class)
            self.signal_groups[data_class] = self.record_file.create_group('/'+data_class)
            self.signal_current_[data_class] = {}
            for sub_name, sub_config in self.configs[data_class].items():
                self.create_sub(sub_name, sub_config, data_class)
                self.create_datasets(sub_name, sub_config, data_class)
    
    def start_node(self):
        #----------- Starting Condition ---------------#
        if self.need_start_condition:
            self.robot_start_pos = np.zeros(3)
            self.start_pos_flag = False
            self.sub_start_pos = self.create_subscription(
                Float64MultiArray,
                "/franka_start_pos",
                self.start_pos_callback,
                1)
            self.get_logger().info(f'Waiting for start position of EE...')
            while not self.start_pos_flag:
                rclpy.spin_once(self)
        #-----------------------------------------------------------------#
        self.get_logger().info(f'Recording will start in {self.start_waiting_time}s...')
        time.sleep(self.start_waiting_time) 
        self.get_logger().info(f'Start recording!')
        timer_period = 1.0 / self.freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_sub(self, sub_name, sub_config, data_class):
        msg_type = get_msg_type(sub_config['msg_type'])
        assert msg_type, "Can not import " + sub_config['msg_type']
        # callbackMethod = getattr(self, 'obs_'+ setup_name + '_callback')
        self.sub_dict.append(self.create_subscription(
            msg_type,
            sub_config['topic'],
            lambda msg, sub_name=sub_name, data_class=data_class: self.sub_callback(
                msg, sub_name, data_class),
        1))
        self.get_logger().info(f'Subscribing {data_class}: {sub_name}')

    def create_datasets(self, sub_name, sub_config, data_class):
        for type_key, type_value in sub_config['signal_type'].items():
            signal_name = sub_name+"_"+type_key
            ds_shape = tuple([0]+type_value['dim'])
            max_shape = tuple([None]+type_value['dim'])
            ds_dtype = eval(type_value['dtype'])
            self.signal_groups[data_class].create_dataset(
                    signal_name, ds_shape, maxshape=max_shape, dtype=ds_dtype)
            self.signal_current_[data_class][signal_name] = None
    
    def sub_callback(self, msg, sub_name, data_class):
        for type_key, type_value in self.configs[data_class][sub_name]['signal_type'].items():
            signal_name = sub_name+"_"+type_key
            self.signal_current_[data_class][signal_name] = np.array(
                getattr(msg,type_value['msg_attr']), dtype=eval(type_value['dtype']))
        
    def timer_callback(self):
        signal2log = self.signal_current_
        for data_class in self.data_classes:
            for signal_name, value in signal2log[data_class].items():
                dset_shape = self.signal_groups[data_class][signal_name].shape
                self.signal_groups[data_class][signal_name].resize(dset_shape[0]+1, axis=0)
                #----------- Handle specific conditions of signal data -----------#
                if data_class == 'act' and 'pose' in signal_name:
                    value[:3] += self.robot_start_pos
                #-----------------------------------------------------------------#
                self.signal_groups[data_class][signal_name][-1, :len(value)] = value
                #----------- Handle specific conditions of dataset ---------------#
                if 'cam' in signal_name:
                    self.signal_groups[data_class][signal_name][-1, len(value)] = np.array(
                        -1, dtype=np.int32)
                #-----------------------------------------------------------------#
    
    def start_pos_callback(self, msg):
        self.robot_start_pos = np.array(msg.data)
        self.get_logger().info(f'Robot start position: {self.robot_start_pos}')
        self.start_pos_flag = True
            
            
class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0, freq=50.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.freq = freq
        self.x_prev = None
        self.dx_prev = None
        self.alpha = self.compute_alpha(min_cutoff)

    def compute_alpha(self, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * self.freq)

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = 0.0
            return x

        dx = (x - self.x_prev) * self.freq
        self.dx_prev = self.dx_prev + self.compute_alpha(self.d_cutoff) * (dx - self.dx_prev)

        cutoff = self.min_cutoff + self.beta * abs(self.dx_prev)
        self.alpha = self.compute_alpha(cutoff)

        x_hat = self.x_prev + self.alpha * (x - self.x_prev)

        self.x_prev = x_hat

        return x_hat 


def main(args=None):
    rclpy.init(args=args)
    args_parser = get_parser()
    obs_act_subscriber = ObsActSubscriber(args_parser)
    obs_act_subscriber.set_args()
    obs_act_subscriber.set_configuration()
    obs_act_subscriber.start_node()
    try:
        rclpy.spin(obs_act_subscriber)
    except KeyboardInterrupt:
        obs_act_subscriber.record_file.close()
        obs_act_subscriber.get_logger().info('Data record stopped.')
        obs_act_subscriber.destroy_node()    


if __name__ == '__main__':
    main()